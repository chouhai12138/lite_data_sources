#!/usr/bin/env python3
"""
力矩前馈补偿模块 — 为 MIT 控制提供 friction / dynamics feedforward torque.

核心思路:
  MIT 控制律: τ_actuator = K*(q_des - q) + D*(qd_des - qd) + effort
  本模块负责计算 effort（前馈力矩），抵消:
    1. 关节摩擦: τ_fric = viscous * qd + coulomb * sign(qd) + offset
    2. (未来) 刚体动力学: τ_dyn = M(q)*qdd + C(q,qd)*qd + g(q)

用法:
  from torque_compensation import TorqueCompensation

  tc = TorqueCompensation('/path/to/identified_params.json')
  effort = tc.compute_friction(joint_names, velocities)
"""

import json
import math
import os
from pathlib import Path
from typing import List, Optional


class TorqueCompensation:
    """关节力矩前馈补偿器。

    从辨识结果 JSON 加载摩擦参数，根据期望速度计算前馈力矩。
    当 Pinocchio 可用时，可额外叠加刚体动力学前馈。
    """

    def __init__(
        self,
        params_path: Optional[str] = None,
        coulomb_epsilon: float = 0.01,
    ):
        """
        Args:
            params_path: identified_params.json 路径。
                         为 None 时自动在包内查找。
            coulomb_epsilon: Coulomb 摩擦平滑参数 (tanh 过渡区宽度, rad/s)。
        """
        self._coulomb_epsilon = coulomb_epsilon

        if params_path is None:
            # 自动查找: 与当前脚本同目录的 ../config/identified_params.json
            script_dir = Path(__file__).resolve().parent
            params_path = script_dir / '..' / 'config' / 'identified_params.json'

        # ------------------------ 加载辨识参数 ------------------------
        if not os.path.exists(params_path):
            raise FileNotFoundError(
                f'identified_params.json not found at {params_path}. '
                f'Pass params_path= to specify the file location.'
            )

        with open(params_path) as f:
            data = json.load(f)

        params = data['parameters']
        drive_params = params['drive']

        # 构建快速查找表: {joint_name: {viscous, coulomb, offset}}
        self._drive = {}
        for name, p in drive_params.items():
            self._drive[name] = {
                'viscous': float(p['viscous']),
                'coulomb': float(p['coulomb']),
                'offset': float(p['offset']),
            }

        # 惯性参数（供将来 Pinocchio 动力学使用）
        self._inertial = params.get('inertial', {})

        # 元信息
        self._identified_joints = data['data']['joints']
        self._fit_rmse_nm = data['solver']['fit_rmse_nm']
        self._per_joint_rmse = data['solver']['per_joint_rmse_nm']

        # ------------------------ 尝试加载 Pinocchio ------------------------
        self._pinocchio_available = False
        self._pin_model = None
        self._pin_data = None

        try:
            import pinocchio as pin
            import numpy as np
            self._pinocchio_available = True
        except ImportError:
            pass

        # 自动加载 URDF 用于重力补偿
        if self._pinocchio_available:
            urdf_path = script_dir / '..' / 'config' / 'lite_000_asm_resolved.urdf'
            if urdf_path.exists():
                try:
                    self.load_urdf(str(urdf_path))
                except Exception as e:
                    print(f'[TorqueCompensation] Failed to load URDF for gravity: {e}')
                    self._pin_model = None

        # 打印加载信息
        n_drive = len(self._drive)
        arms_joints = [j for j in self._drive if 'gripper' not in j and 'passive' not in j]
        print(f'[TorqueCompensation] Loaded {n_drive} joint drive params '
              f'({len(arms_joints)} arms, RMSE={self._fit_rmse_nm:.3f} Nm)')
        if self._pinocchio_available and self._pin_model is not None:
            print(f'[TorqueCompensation] Pinocchio model loaded — gravity compensation active '
                  f'({self._pin_model.njoints} links)')
        elif self._pinocchio_available:
            print('[TorqueCompensation] Pinocchio available but URDF not loaded')
        else:
            print('[TorqueCompensation] Pinocchio not available — friction-only mode')

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def compute_friction(
        self,
        joint_names: List[str],
        velocities: List[float],
    ) -> List[float]:
        """计算关节摩擦力矩前馈 (Nm)。

        τ_fric = viscous * qd + coulomb * sign(qd) + offset

        Coulomb 摩擦用 tanh 平滑以避免在零速附近震荡。

        Args:
            joint_names: 关节名列表。
            velocities:  期望速度 (rad/s)，与 joint_names 一一对应。

        Returns:
            力矩列表 (Nm)，与 joint_names 一一对应。
            不在辨识结果中的关节返回 0.0。
        """
        eps = self._coulomb_epsilon
        tau = []

        for name, qd in zip(joint_names, velocities):
            p = self._drive.get(name)
            if p is None:
                tau.append(0.0)
                continue

            viscous = p['viscous'] * qd
            # tanh 平滑 Coulomb: sign(qd) ≈ tanh(qd / epsilon)
            coulomb = p['coulomb'] * math.tanh(qd / eps)
            offset = p['offset']

            tau.append(viscous + coulomb + offset)

        return tau

    def compute_dynamics(
        self,
        joint_names: List[str],
        positions: List[float],
        velocities: List[float],
        accelerations: List[float],
    ) -> List[float]:
        """计算刚体动力学前馈力矩 (需要 Pinocchio)。

        τ_dyn = M(q)*qdd + C(q,qd)*qd + g(q)

        如果 Pinocchio 不可用，返回全零并打印警告。

        Args:
            joint_names:    关节名列表。
            positions:      关节位置 (rad)。
            velocities:     关节速度 (rad/s)。
            accelerations:  关节加速度 (rad/s²)。

        Returns:
            力矩列表 (Nm)。
        """
        if not self._pinocchio_available:
            raise RuntimeError(
                'Pinocchio not installed. Install with: '
                'pip install pinocchio'
            )
        if self._pin_model is None:
            raise RuntimeError(
                'Pinocchio model not loaded. Call load_urdf() first.'
            )

        n = len(joint_names)
        q = self._to_pinocchio_order(joint_names, positions)
        qd = self._to_pinocchio_order(joint_names, velocities)
        qdd = self._to_pinocchio_order(joint_names, accelerations)

        import pinocchio as pin
        tau_pin = pin.rnea(self._pin_model, self._pin_data, q, qd, qdd)

        return self._from_pinocchio_order(joint_names, tau_pin)

    def load_urdf(self, urdf_path: str):
        """加载 URDF 并构建 Pinocchio 模型，注入辨识的惯性参数。"""
        if not self._pinocchio_available:
            raise RuntimeError('Pinocchio not available.')

        import pinocchio as pin
        import numpy as np

        self._pin_model = pin.buildModelFromUrdf(urdf_path)
        self._pin_data = self._pin_model.createData()

        # --- 注入辨识惯性参数 ---
        # inertial params keyed as "inertial/{joint_name}/mass", etc.
        param_order = ['mass', 'mx', 'my', 'mz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz']
        injected = 0
        for pin_id in range(2, self._pin_model.njoints):  # skip universe, root
            pin_name = self._pin_model.names[pin_id]
            # 匹配 BAR 关节名
            matched_bar_name = None
            for bar_name in self._drive.keys():
                # Pinocchio joint name 可能是 "left_shoulder_pitch_joint"
                # BAR name 是 "left_shoulder_pitch"
                if bar_name in pin_name:
                    matched_bar_name = bar_name
                    break

            if matched_bar_name is None:
                continue

            # 读取辨识的惯性参数
            inert = {}
            has_any = False
            for pname in param_order:
                key = f'inertial/{matched_bar_name}_joint/{pname}'
                val = self._inertial.get(key)
                if val is not None:
                    inert[pname] = float(val)
                    has_any = True

            if not has_any:
                continue

            # 注入到 Pinocchio 模型
            mass = inert.get('mass', 0.0)
            com = np.array([inert.get('mx', 0.0), inert.get('my', 0.0), inert.get('mz', 0.0)])
            inertia_mat = np.array([
                [inert.get('Ixx', 0.0), inert.get('Ixy', 0.0), inert.get('Ixz', 0.0)],
                [inert.get('Ixy', 0.0), inert.get('Iyy', 0.0), inert.get('Iyz', 0.0)],
                [inert.get('Ixz', 0.0), inert.get('Iyz', 0.0), inert.get('Izz', 0.0)],
            ])

            # 只注入有意义的惯性参数（重力只需 mass + CoM）
            if abs(mass) > 1e-6:
                # 保留 URDF 原有的 inertia 矩阵，只替换 mass 和 CoM
                old_inertia = self._pin_model.inertias[pin_id]
                # Pinocchio 4.x: inertia 是属性不是方法
                try:
                    old_I = old_inertia.inertia()
                except TypeError:
                    old_I = old_inertia.inertia  # 属性直接访问
                new_inertia = pin.Inertia(mass, com, old_I)
                self._pin_model.inertias[pin_id] = new_inertia
                injected += 1

        print(f'[TorqueCompensation] Injected identified inertias to {injected} links')

        # --- 构建关节名映射 ---
        self._bar_to_pin_idx = {}
        self._pin_nq = self._pin_model.nq
        self._pin_nv = self._pin_model.nv

        for pin_id in range(2, self._pin_model.njoints):
            pin_name = self._pin_model.names[pin_id]
            joint = self._pin_model.joints[pin_id]
            if joint.nq > 0:  # 非固定关节
                for bar_name in self._drive.keys():
                    if bar_name in pin_name:
                        self._bar_to_pin_idx[bar_name] = joint.idx_v
                        break

        n_mapped = len(self._bar_to_pin_idx)
        print(f'[TorqueCompensation] Pinocchio model loaded: '
              f'{self._pin_model.njoints} joints, {n_mapped} mapped to BAR joints')

    def compute_gravity(self, joint_names: List[str], positions: List[float]) -> List[float]:
        """计算重力力矩 (Nm)。需要已调用 load_urdf()。

        τ_g = g(q) — 仅重力项，不含 Coriolis/惯性。
        """
        if self._pin_model is None:
            return [0.0] * len(joint_names)

        import pinocchio as pin
        import numpy as np

        # 构建 Pinocchio 顺序的 q 向量
        q = np.zeros(self._pin_nq)
        unmapped = 0
        for name, pos in zip(joint_names, positions):
            if name in self._bar_to_pin_idx:
                idx = self._bar_to_pin_idx[name]
                # idx_v 是 velocity 索引，idx_q 是 position 索引
                # 对于 revolute joint，通常是同一个
                q_idx = idx  # revolute joints: idx_q == idx_v
                if q_idx < self._pin_nq:
                    q[q_idx] = pos
            else:
                unmapped += 1

        if unmapped > 0:
            print(f'[TorqueCompensation] WARNING: {unmapped} joints not mapped to Pinocchio model')

        # RNEA with zero velocity and acceleration → pure gravity
        qd = np.zeros(self._pin_nv)
        qdd = np.zeros(self._pin_nv)
        tau_pin = pin.rnea(self._pin_model, self._pin_data, q, qd, qdd)

        # 映射回 BAR 关节顺序
        tau = []
        for name in joint_names:
            if name in self._bar_to_pin_idx:
                idx = self._bar_to_pin_idx[name]
                tau.append(float(tau_pin[idx]) if idx < len(tau_pin) else 0.0)
            else:
                tau.append(0.0)

        return tau

    def compute_full_feedforward(
        self,
        joint_names: List[str],
        positions: List[float],
        velocities: List[float],
        accelerations: Optional[List[float]] = None,
    ) -> List[float]:
        """计算完整前馈力矩: friction + dynamics (如果可用)。

        Args:
            joint_names:    关节名列表。
            positions:      当前/期望位置 (rad)。
            velocities:     当前/期望速度 (rad/s)。
            accelerations:  期望加速度 (rad/s²)。None 时仅使用摩擦补偿。

        Returns:
            总前馈力矩列表 (Nm)。
        """
        tau = self.compute_friction(joint_names, velocities)

        if accelerations is not None and self._pinocchio_available and self._pin_model is not None:
            tau_dyn = self.compute_dynamics(joint_names, positions, velocities, accelerations)
            tau = [t_f + t_d for t_f, t_d in zip(tau, tau_dyn)]

        return tau

    # ------------------------------------------------------------------
    # 内部辅助
    # ------------------------------------------------------------------

    def _to_pinocchio_order(
        self, joint_names: List[str], values: List[float]
    ) -> 'np.ndarray':
        """将 BAR 关节顺序的值映射到 Pinocchio 模型顺序。"""
        import numpy as np
        nq = self._pin_model.nq
        arr = np.zeros(nq)
        for name, val in zip(joint_names, values):
            if name in self._pin_joint_map:
                idx = self._pin_model.joints[self._pin_joint_map[name]].idx_q
                arr[idx] = val
        return arr

    def _from_pinocchio_order(
        self, joint_names: List[str], tau_pin: 'np.ndarray'
    ) -> List[float]:
        """将 Pinocchio 顺序的力矩映射回 BAR 关节顺序。"""
        result = [0.0] * len(joint_names)
        for i, name in enumerate(joint_names):
            if name in self._pin_joint_map:
                idx = self._pin_model.joints[self._pin_joint_map[name]].idx_v
                result[i] = float(tau_pin[idx])
        return result

    @property
    def identified_joints(self) -> List[str]:
        """辨识覆盖的关节名列表。"""
        return list(self._identified_joints)

    @property
    def uses_pinocchio(self) -> bool:
        """是否加载了 Pinocchio 模型。"""
        return self._pinocchio_available and self._pin_model is not None

    @property
    def fit_rmse_nm(self) -> float:
        """辨识拟合 RMSE (Nm)。"""
        return self._fit_rmse_nm

    def get_params(self, joint_name: str) -> Optional[dict]:
        """获取单个关节的摩擦参数。"""
        return self._drive.get(joint_name)
