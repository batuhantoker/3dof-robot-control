# Python Port — Kinematics & Visualization

Lightweight Python scripts for playing with the 3-DOF robot kinematics. The full dynamics and control implementation lives in the MATLAB/Simulink code.

## What's Here

| File | Description |
|------|-------------|
| `kinematics.py` | FK, IK (damped least-squares), Jacobian, manipulability, workspace |
| `visualize.py` | 3D arm plotting, workspace point cloud, manipulability heatmap |
| `demo.py` | Quick demo: FK, IK round-trip verification, plots |

## Usage

```bash
pip install numpy matplotlib
python demo.py
```

## Notes

- All dimensions in **millimeters**, angles in **radians** (unless noted)
- DH parameters and FK equations match the MATLAB code exactly
- IK uses damped least-squares (Levenberg-Marquardt style) — not analytical
- Workspace computation sweeps all joints through their limits
