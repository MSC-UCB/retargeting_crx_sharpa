I am working on retargetting crx project, and i use this command to generate the retargetting joint pose to both crx and shapa hand.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py \
  --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50


  I want to change the initial pose of the crx-arm, where rightarm is -90 -30 240 0 -60 -90, and the left arm is 0 30 -60 0 60 90

your mehthod:

### 最终确认的初始关节目标

用户已确认修正后的预览姿态符合预期。以下数值均按 J1～J6 排列：

| 角度定义 | 左臂（°） | 右臂（°） |
| --- | --- | --- |
| FANUC 示教器 | `[0, 30, -60, 0, 60, 90]` | `[-90, -30, 240, 0, -60, -90]` |
| ROS / URDF | `[0, 30, -30, 0, 60, 90]` | `[-90, -30, 210, 0, -60, -90]` |

对应 YAML 中的机械臂初始值（弧度）：

```text
left:  [0, 0.5235987755982988, -0.5235987755982988, 0, 1.0471975511965976, 1.5707963267948966]
right: [-1.5707963267948966, -0.5235987755982988, 3.6651914291880923, 0, -1.0471975511965976, -1.5707963267948966]
```

示教器转 ROS 时使用 `J3_ROS = J3_示教器 + J2_示教器`。Retargeting 求解结果已经采用 ROS / URDF 定义，发布关节目标时不应再次转换 J3。以上是配置的初始姿态；ROS 模式以实机关节反馈作为遥操作起点，后续目标随手部动作更新。

### 方案与核对记录

我的意见：建议不要直接修改 retargeting 算法或手部映射逻辑，而是在启动 teleoperation 之前增加一个独立的“初始姿态”步骤。将右臂 `[-90, -30, 240, 0, -60, -90]` 和左臂 `[0, 30, -60, 0, 60, 90]` 作为配置参数传入，并在发送给 CRX 前统一确认以下事项：

1. 角度单位是否为度；如果底层 ROS 接口使用弧度，应只在接口边界处转换一次。
2. 六个数的顺序是否与 CRX 实际关节顺序一致，同时确认左右臂的关节正负方向没有反转。
3. teleoperation 开始前先让双臂以较低速度移动到该姿态，读取并检查实际关节反馈；确认姿态安全后，再启动手部 retargeting 和连续控制。

这样初始姿态只负责机器人启动位置，手部 retargeting 仍然负责后续动作，职责清晰，也便于以后更换初始姿态或单独测试某一只手臂。尤其是右臂的 `240°` 需要结合关节限位确认，不能只根据数值直接发送。

补充确认：项目中确实有对应的 YAML 配置。当前命令调用 `scripts/run_crx_sharpa_joint_teleop.py`，默认使用 `configs/bimanual/crx5ia_sharpa_wave.yaml`；该文件再分别引用：

- `configs/robots/crx5ia_sharpa_wave_right.yaml`
- `configs/robots/crx5ia_sharpa_wave_left.yaml`

这两份文件中的 `initial_qpos` 前六项就是 CRX 六个关节的初始值，单位是弧度。现已按以下数值更新前六项；后面 22 个手部关节值保留。下面仅展示机械臂部分，不能用六项数组覆盖完整的 `initial_qpos`：

```yaml
# right ROS/URDF: -90, -30, 210, 0, -60, -90 degrees
initial_qpos: [-1.5707963, -0.5235988, 3.6651914, 0.0, -1.0471976, -1.5707963]

# left ROS/URDF: 0, 30, -30, 0, 60, 90 degrees
initial_qpos: [0.0, 0.5235988, -0.5235988, 0.0, 1.0471976, 1.5707963]
```

角度定义修正：用户确认原始数值来自 FANUC 示教器。驱动 `fanuc_client.cpp` 的 `readJointAnglesRMI()` 使用 `J3_ROS = J3_示教器 + J2_示教器`，`writeJointTargetRMI()` 使用逆转换。因此右臂 J3 应为 `240 - 30 = 210°`，左臂应为 `-60 + 30 = -30°`，再转为弧度写入 YAML。此前直接将示教器 J3 转为弧度的做法有误；预览和求解器一致只说明两者使用同一模型，不能验证输入的角度定义正确。

URDF 中的 `<joint>` 配置用于描述关节轴和限位。无需修改 URDF；ROS 运行路径使用驱动转换后的关节值，不应再次叠加 J2。

运行行为说明：`--backend ros` 使用的 `SharpaJointBackend` 启动时会读取实际关节反馈，执行流也会以反馈值初始化 retargeter。因此这里的 YAML 修改更新的是配置和预览初始姿态，不会自动驱动实机回到该姿态。前文的启动前移动只是操作建议，当前脚本没有因此增加自动归位动作。
