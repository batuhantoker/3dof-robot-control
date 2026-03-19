# 3-DOF Spatial Robot: Modeling, Control & Analysis

Full kinematic/dynamic modeling and multi-strategy control of a 3-DOF spatial manipulator.

![Screen Shot](https://user-images.githubusercontent.com/55883119/209437633-e039ac37-953b-4c20-8757-a5209711aa9c.png)

![MATLAB](https://img.shields.io/badge/MATLAB-R2022b-orange)
![Simulink](https://img.shields.io/badge/Simulink-Real--Time-blue)
![Python](https://img.shields.io/badge/Python-3.10+-yellow)
![License](https://img.shields.io/badge/License-MIT-green)

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Degrees of Freedom | 3 (spatial, RRR) |
| Link lengths | l₁ = 106 mm, l₂ = 98.7 mm, l₃ = 140.25 mm |
| DH — d | [106, 0, 0] |
| DH — a | [0, 99, 140] |
| DH — α | [π/2, 0, 0] |
| Joint limits | q₁: ±180°, q₂: −45° to 135°, q₃: −135° to −45° |

## Kinematic Analysis

- **Forward Kinematics** — DH convention transformation matrices
- **Inverse Kinematics** — Analytical and numerical (damped least-squares)
- **Analytical Jacobian** — 6×3 geometric Jacobian
- **Workspace Computation** — Full reachable space via joint sweep
- **Singularity Detection** — Jacobian rank analysis, duplicate point identification
- **Manipulability Ellipsoids** — Eigenvalue decomposition of J·Jᵀ
- **Max End-Effector Force** — Force mapping via Jacobian transpose for given joint torques

### End-Effector Position (Forward Kinematics)

```
x = l₃·cos(q₂+q₃)·cos(q₁) + l₂·cos(q₁)·cos(q₂)
y = l₃·cos(q₂+q₃)·sin(q₁) + l₂·sin(q₁)·cos(q₂)
z = l₃·sin(q₂+q₃) + l₂·sin(q₂) + l₁
```

## Dynamics

Full equations of motion derived symbolically and validated against the physical robot:

**M(q)·q̈ + C(q, q̇)·q̇ + G(q) = τ**

- **Inertia matrix M(q)** — 3×3, configuration-dependent
- **Coriolis matrix C(q, q̇)** — Christoffel symbols
- **Gravity vector G(q)**

Identified physical parameters from the real robot:

| Parameter | Value (mm) |
|-----------|-----------|
| lc₁ (link 1 CoM) | 38.29 |
| lc₂ (link 2 CoM) | 24.45 |
| lc₃ (link 3 CoM) | 37.90 |

## Control Strategies

All controllers were implemented on the physical robot using **Simulink Real-Time**.

| Controller | Description |
|-----------|-------------|
| **PD** | Proportional-derivative, joint space |
| **PD + Gravity Compensation** | PD with model-based gravity feedforward |
| **Computed Torque (Joint Space)** | Inverse dynamics / feedback linearization in joint space |
| **Computed Torque (Task Space)** | Inverse dynamics in Cartesian/configuration space |
| **Passivity-Based (Configuration Space)** | Energy-based control exploiting skew-symmetry of M̊−2C |

## Repository Structure

```
├── matlab/                  # Core MATLAB scripts
│   ├── htf.m                  # DH homogeneous transformation
│   ├── Denavit_hartenberg.m   # DH parameter setup & symbolic kinematics
│   ├── Reachableworkspace.m   # Workspace, singularity, manipulability
│   ├── MaxForceEndEff.m       # Max end-effector force computation
│   ├── eqnofmotion.m         # M(q), C(q,q̇), G(q) matrices
│   ├── Simulation_movie.m     # 3D animation of robot motion
│   └── Derivationofeqnofmotion.mlx  # Symbolic derivation (live script)
├── control/                 # Simulink models
│   ├── JointSpacePD.slx
│   ├── JointSpacePDGravity.slx
│   ├── ComputedTorqueJointSpace.slx
│   ├── ComputedTorqueConfigurationSpace.slx
│   ├── PassivityBasedConfigurationSpace.slx
│   ├── ConfigurationSpace.slx
│   ├── KinematicBlocks.slx
│   └── link3manip.slx
├── figures/                 # Simulation videos & saved figures
├── python/                  # Python port (kinematics & visualization)
│   ├── kinematics.py
│   ├── visualize.py
│   └── demo.py
├── Project_report.pdf
└── license.md
```

## Python Port

A lightweight Python implementation of the kinematics and visualization for quick experimentation. The MATLAB/Simulink code is the full implementation — the Python port covers forward/inverse kinematics and 3D visualization.

```bash
pip install numpy matplotlib
cd python
python demo.py
```

See [`python/README.md`](python/README.md) for details.

## References

- Spong, Hutchinson, Vidyasagar — *Robot Modeling and Control*
- Craig — *Introduction to Robotics: Mechanics and Control*
- Siciliano et al. — *Robotics: Modelling, Planning and Control*
