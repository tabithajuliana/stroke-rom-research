# Asymmetric ROM Study for Stroke Rehabilitation

## Project Overview
Investigating how asymmetric range of motion (ROM) in stroke patients affects upper-extremity workspace and task performance for robot-assisted rehabilitation.

## Research Question
How does reduced ROM in the affected limb (right arm) impact bilateral workspace accessibility and functional reach?

## Relationship to Figueroa Lab RGA System
This research extends the [Penn Figueroa Lab's assistive_robot_grasp](https://github.com/penn-figueroa-lab/assistive_robot_grasp) system by modeling patient-specific ROM limitations.

## URDF Variants

| Variant | ROM Level | Shoulder ROM | Elbow ROM |
|---------|-----------|--------------|-----------|
| Baseline | 100% | -90° to 135° | 0° to 145° |
| Mild Stroke | 80% | -72° to 108° | 0° to 116° |
| Moderate Stroke | 60% | -54° to 81° | 0° to 87° |
| Severe Stroke | 40% | -36° to 54° | 0° to 58° |
| Recovery | Variable | Progressive | Progressive |

## Modified Joints
- jRightShoulder_rotx - Shoulder flexion/extension
- jRightElbow_rotz - Elbow flexion
- Left arm remains at 100% ROM (unaffected limb)

## Repository Structure

```
stroke-rom-research/
├── urdf_variants/
│   ├── baseline/
│   ├── mild_stroke/
│   ├── moderate_stroke/
│   ├── severe_stroke/
│   └── recovery/
├── data/mocap/
├── scripts/
├── results/
│   ├── screenshots/
│   └── workspace_analysis/
└── docs/
```
## Next Steps
- [ ] Visualize all 5 variants in RViz
- [ ] Capture screenshots showing ROM differences
- [ ] Quantify reachable workspace volume
- [ ] Document ROM-workspace relationship
