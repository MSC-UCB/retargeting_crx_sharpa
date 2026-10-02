# CRX-5iA + LEAP Hand (left)

This bundle combines the existing FANUC CRX-5iA arm geometry with the official
left-hand LEAP V1 geometry from
`leap-hand/Bidex_VisionPro_Teleop`, preserving the repository's CRX+LEAP frame
and joint-name contract (`J1..J6`, then `joint_0..joint_15`). The left mount is
intentionally unflipped; the sibling right-hand bundle uses the physical-test
180-degree remount.

The URDF reuses arm visual and collision meshes from the sibling
`crx5ia_leap_paxini/meshes/crx5ia/` directory through relative paths. Keep both
robot directories together; only the left-hand meshes are stored locally.

The left hand uses its upstream left palm and finger meshes and joint origins.
The repository adds `wrist` and fingertip-center frames. The `flange_to_leap`
translation compensates for the different upstream palm origin: the middle
finger root (`joint_5`) uses the prior unflipped left-hand compensation. The
resulting translation is `[0.037336626243399, 0.047897767636037,
0.140188227821871]` metres with RPY `[0, -1.56, 0]`. The left CRX home is
configured separately from the right CRX home in
`configs/robots/crx5ia_leap_paxini_left.yaml`; its current arm vector is
`[0, 30, -30, 0, 60, 0]` in ROS / URDF degrees, matching CRX + Sharpa and
`dual_crx_control/config/initial_pose.yaml`. The left mount is deliberately independent of
the right-hand 180-degree remount and of the J6 home value.

## Mount orientation and physical validation

The left model intentionally retains the unflipped fixed-axis RPY `[0, -1.56, 0]`;
only the right-hand sibling uses the physical-test 180-degree flip. Validate the
left flange-to-wrist alignment, physical bracket dimensions, and
inter-arm/attached-hand clearance before hardware control.

Source: <https://github.com/leap-hand/Bidex_VisionPro_Teleop>,
`leap_hand_mesh_left/`. Source license text is retained in
`provenance_LICENSE.md`; verify any downstream asset licensing requirements
before redistribution.
