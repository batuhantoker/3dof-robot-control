"""
3-DOF Spatial Robot Kinematics

Forward and inverse kinematics for a 3-DOF RRR spatial manipulator.
DH parameters and equations match the MATLAB implementation.
"""

import numpy as np
from typing import NamedTuple


# --- Robot Parameters ---

L1 = 106.0      # mm, link 1 (base to shoulder)
L2 = 98.7       # mm, link 2 (shoulder to elbow)
L3 = 140.25     # mm, link 3 (elbow to end-effector)

# DH parameters: [theta, d, a, alpha]
# theta is the joint variable (not stored here)
DH_D = np.array([106.0, 0.0, 0.0])
DH_A = np.array([0.0, 99.0, 140.0])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0])

# Joint limits (radians)
Q_MIN = np.array([np.radians(-180), np.radians(-45), np.radians(-135)])
Q_MAX = np.array([np.radians(180), np.radians(135), np.radians(-45)])


class FKResult(NamedTuple):
    """Forward kinematics result."""
    position: np.ndarray     # (3,) end-effector [x, y, z] in mm
    joint_positions: np.ndarray  # (4, 3) positions of base, shoulder, elbow, EE
    transforms: list         # list of 4x4 homogeneous transforms per joint


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Compute 4x4 homogeneous transformation matrix from DH parameters.
    Matches matlab/htf.m exactly.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,       ca,      d],
        [0.0,    0.0,      0.0,    1.0],
    ])


def forward_kinematics(q: np.ndarray) -> FKResult:
    """
    Compute forward kinematics for joint angles q = [q1, q2, q3].

    Returns positions of all joints and the end-effector, plus the
    intermediate homogeneous transforms.
    """
    q = np.asarray(q, dtype=float)
    q1, q2, q3 = q

    # Build chain of transforms
    transforms = []
    T = np.eye(4)
    for i in range(3):
        Ti = dh_transform(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        T = T @ Ti
        transforms.append(T.copy())

    # Joint positions (from the analytical expressions in MATLAB)
    p_base = np.array([0.0, 0.0, 0.0])
    p_shoulder = np.array([0.0, 0.0, L1])
    p_elbow = np.array([
        L2 * np.cos(q1) * np.cos(q2),
        L2 * np.sin(q1) * np.cos(q2),
        L2 * np.sin(q2) + L1,
    ])
    p_ee = np.array([
        L3 * np.cos(q2 + q3) * np.cos(q1) + L2 * np.cos(q1) * np.cos(q2),
        L3 * np.cos(q2 + q3) * np.sin(q1) + L2 * np.sin(q1) * np.cos(q2),
        L3 * np.sin(q2 + q3) + L2 * np.sin(q2) + L1,
    ])

    joint_positions = np.array([p_base, p_shoulder, p_elbow, p_ee])

    return FKResult(
        position=p_ee,
        joint_positions=joint_positions,
        transforms=transforms,
    )


def jacobian(q: np.ndarray) -> np.ndarray:
    """
    Compute the 3x3 analytical (position) Jacobian.
    Derived from the MATLAB Reachableworkspace.m code.

    Returns the 3x3 matrix mapping joint velocities to end-effector
    linear velocity: v = J(q) @ q_dot
    """
    q = np.asarray(q, dtype=float)
    q1, q2, q3 = q

    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

    J = np.array([
        [-L3 * c23 * s1 - L2 * c2 * s1,  -L3 * s23 * c1 - L2 * c1 * s2,  -L3 * s23 * c1],
        [ L3 * c23 * c1 + L2 * c1 * c2,  -L3 * s23 * s1 - L2 * s1 * s2,  -L3 * s23 * s1],
        [0.0,                              L3 * c23 + L2 * c2,              L3 * c23],
    ])
    return J


def inverse_kinematics(
    target: np.ndarray,
    q0: np.ndarray | None = None,
    tol: float = 1e-4,
    max_iter: int = 200,
    damping: float = 0.5,
) -> np.ndarray:
    """
    Numerical inverse kinematics using damped least-squares (Levenberg-Marquardt).

    Args:
        target: Desired end-effector position [x, y, z] in mm.
        q0: Initial joint angle guess (radians). Defaults to zeros.
        tol: Position error tolerance in mm.
        max_iter: Maximum iterations.
        damping: Damping factor (lambda) for singularity robustness.

    Returns:
        Joint angles [q1, q2, q3] in radians.

    Raises:
        RuntimeError: If solution doesn't converge.
    """
    target = np.asarray(target, dtype=float)
    q = np.array(q0, dtype=float) if q0 is not None else np.zeros(3)

    for i in range(max_iter):
        fk = forward_kinematics(q)
        error = target - fk.position

        if np.linalg.norm(error) < tol:
            return q

        J = jacobian(q)
        # Damped least-squares: dq = J^T (J J^T + lambda^2 I)^-1 e
        JJt = J @ J.T + (damping ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, error)

        q = q + dq
        # Wrap q1 to [-pi, pi], clamp q2 and q3
        q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
        q[1] = np.clip(q[1], Q_MIN[1], Q_MAX[1])
        q[2] = np.clip(q[2], Q_MIN[2], Q_MAX[2])

    raise RuntimeError(
        f"IK did not converge after {max_iter} iterations. "
        f"Final error: {np.linalg.norm(error):.4f} mm"
    )


def manipulability(q: np.ndarray) -> float:
    """
    Yoshikawa manipulability index: w = sqrt(det(J @ J.T)).
    Higher values indicate better dexterity; zero at singularities.
    """
    J = jacobian(q)
    return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))


def workspace_points(
    n_samples: int = 20,
) -> np.ndarray:
    """
    Compute reachable workspace by sweeping all joints through their limits.

    Args:
        n_samples: Number of samples per joint.

    Returns:
        (N, 3) array of reachable end-effector positions in mm.
    """
    q1_range = np.linspace(Q_MIN[0], Q_MAX[0], n_samples)
    q2_range = np.linspace(Q_MIN[1], Q_MAX[1], n_samples)
    q3_range = np.linspace(Q_MIN[2], Q_MAX[2], n_samples)

    points = []
    for q1 in q1_range:
        for q2 in q2_range:
            for q3 in q3_range:
                fk = forward_kinematics(np.array([q1, q2, q3]))
                points.append(fk.position)

    return np.array(points)
