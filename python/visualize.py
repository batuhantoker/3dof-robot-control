"""
3-DOF Spatial Robot Visualization

3D arm rendering and workspace point cloud plotting.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from kinematics import forward_kinematics, workspace_points, jacobian


def plot_arm(
    q: np.ndarray,
    ax: plt.Axes | None = None,
    title: str = "3-DOF Spatial Manipulator",
    show: bool = True,
) -> plt.Axes:
    """
    Plot the robot arm in 3D for a given joint configuration.

    Args:
        q: Joint angles [q1, q2, q3] in radians.
        ax: Existing 3D axes. Created if None.
        title: Plot title.
        show: Whether to call plt.show().

    Returns:
        The matplotlib 3D axes.
    """
    fk = forward_kinematics(q)
    pts = fk.joint_positions  # (4, 3): base, shoulder, elbow, EE

    if ax is None:
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection="3d")

    # Draw links
    colors = ["#2196F3", "#4CAF50", "#F44336"]
    labels = ["Link 1", "Link 2", "Link 3"]
    for i in range(3):
        ax.plot(
            [pts[i, 0], pts[i + 1, 0]],
            [pts[i, 1], pts[i + 1, 1]],
            [pts[i, 2], pts[i + 1, 2]],
            color=colors[i],
            linewidth=3,
            label=labels[i],
        )

    # Draw joints
    ax.scatter(*pts[0], color="black", s=80, zorder=5, label="Base")
    ax.scatter(*pts[1:3].T, color="#333333", s=50, zorder=5)
    ax.scatter(*pts[3], color="#FF5722", s=60, zorder=5, marker="D", label="End-Effector")

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(title)

    # Equal aspect ratio
    limit = 300
    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([-limit, limit])
    ax.legend(loc="upper left", fontsize=8)

    if show:
        plt.tight_layout()
        plt.show()

    return ax


def plot_workspace(
    n_samples: int = 20,
    show: bool = True,
) -> plt.Axes:
    """
    Plot the reachable workspace as a 3D point cloud.

    Args:
        n_samples: Samples per joint for the sweep.
        show: Whether to call plt.show().
    """
    print(f"Computing workspace ({n_samples}^3 = {n_samples**3} points)...")
    pts = workspace_points(n_samples)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Color by distance from base
    dist = np.linalg.norm(pts, axis=1)
    sc = ax.scatter(
        pts[:, 0], pts[:, 1], pts[:, 2],
        c=dist, cmap="viridis", s=1, alpha=0.4,
    )
    plt.colorbar(sc, ax=ax, label="Distance from base (mm)", shrink=0.6)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("Reachable Workspace")

    if show:
        plt.tight_layout()
        plt.show()

    return ax


def plot_manipulability_map(
    n_samples: int = 15,
    q1: float = 0.0,
    show: bool = True,
) -> plt.Axes:
    """
    Plot manipulability index across q2-q3 space for a fixed q1.
    Bright = high dexterity, dark = near singularity.
    """
    from kinematics import manipulability, Q_MIN, Q_MAX

    q2_range = np.linspace(Q_MIN[1], Q_MAX[1], n_samples)
    q3_range = np.linspace(Q_MIN[2], Q_MAX[2], n_samples)
    Q2, Q3 = np.meshgrid(q2_range, q3_range)
    W = np.zeros_like(Q2)

    for i in range(n_samples):
        for j in range(n_samples):
            W[i, j] = manipulability(np.array([q1, Q2[i, j], Q3[i, j]]))

    fig, ax = plt.subplots(figsize=(8, 6))
    c = ax.pcolormesh(
        np.degrees(Q2), np.degrees(Q3), W,
        cmap="plasma", shading="auto",
    )
    plt.colorbar(c, ax=ax, label="Manipulability index")
    ax.set_xlabel("q₂ (deg)")
    ax.set_ylabel("q₃ (deg)")
    ax.set_title(f"Manipulability Map (q₁ = {np.degrees(q1):.0f}°)")

    if show:
        plt.tight_layout()
        plt.show()

    return ax
