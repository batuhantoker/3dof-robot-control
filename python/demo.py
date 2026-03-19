#!/usr/bin/env python3
"""
3-DOF Spatial Robot — Quick Demo

Demonstrates forward kinematics, inverse kinematics round-trip,
and 3D visualization.
"""

import numpy as np
from kinematics import forward_kinematics, inverse_kinematics, manipulability, jacobian
from visualize import plot_arm, plot_workspace, plot_manipulability_map
import matplotlib.pyplot as plt


def main():
    np.set_printoptions(precision=3, suppress=True)

    # --- Forward Kinematics ---
    print("=" * 50)
    print("FORWARD KINEMATICS")
    print("=" * 50)

    test_configs = [
        ("Home (all zeros)", [0, 0, 0]),
        ("q = [45°, 30°, -90°]", [45, 30, -90]),
        ("q = [0°, 90°, -90°]", [0, 90, -90]),
        ("q = [-30°, 60°, -60°]", [-30, 60, -60]),
    ]

    for name, q_deg in test_configs:
        q = np.radians(q_deg)
        fk = forward_kinematics(q)
        w = manipulability(q)
        print(f"\n{name}")
        print(f"  q = {q_deg}°")
        print(f"  EE position: [{fk.position[0]:.2f}, {fk.position[1]:.2f}, {fk.position[2]:.2f}] mm")
        print(f"  Manipulability: {w:.2f}")

    # --- Inverse Kinematics Round-Trip ---
    print("\n" + "=" * 50)
    print("INVERSE KINEMATICS (round-trip test)")
    print("=" * 50)

    q_original = np.radians([30, 45, -80])
    fk = forward_kinematics(q_original)
    target = fk.position
    print(f"\nOriginal q: {np.degrees(q_original).astype(int)}°")
    print(f"FK target:  [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}] mm")

    q_solved = inverse_kinematics(target, q0=np.zeros(3))
    fk_check = forward_kinematics(q_solved)
    error = np.linalg.norm(target - fk_check.position)

    print(f"IK solved:  {np.degrees(q_solved).round(2)}°")
    print(f"FK verify:  [{fk_check.position[0]:.2f}, {fk_check.position[1]:.2f}, {fk_check.position[2]:.2f}] mm")
    print(f"Error:      {error:.4f} mm")

    # --- Jacobian ---
    print("\n" + "=" * 50)
    print("JACOBIAN")
    print("=" * 50)

    q_test = np.radians([0, 45, -90])
    J = jacobian(q_test)
    print(f"\nJ(q) at q = [0°, 45°, -90°]:")
    print(J)
    print(f"\ndet(J) = {np.linalg.det(J):.2f}")
    print(f"Condition number: {np.linalg.cond(J):.2f}")

    # --- Visualization ---
    print("\n" + "=" * 50)
    print("PLOTTING")
    print("=" * 50)

    # Multi-config arm plot
    fig = plt.figure(figsize=(16, 5))
    configs = [
        ("Home", [0, 0, -90]),
        ("Reaching up", [0, 90, -90]),
        ("Reaching out", [0, 0, -45]),
        ("IK solution", np.degrees(q_solved).tolist()),
    ]
    for idx, (name, q_deg) in enumerate(configs, 1):
        ax = fig.add_subplot(1, 4, idx, projection="3d")
        plot_arm(np.radians(q_deg), ax=ax, title=name, show=False)
    plt.suptitle("Robot Configurations", fontsize=14, y=1.02)
    plt.tight_layout()
    plt.show()

    # Workspace
    plot_workspace(n_samples=20)

    # Manipulability
    plot_manipulability_map(n_samples=30)


if __name__ == "__main__":
    main()
