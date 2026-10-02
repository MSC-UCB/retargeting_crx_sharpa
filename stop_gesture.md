# 雙手拇指與無名指捏合：停止手勢

Enabled by default in `scripts/run_crx_sharpa_joint_teleop.py` with a 2-second confirmation hold. Supports live Quest input with preview/ROS backends and `--no-viewer`. Use `--stop-gesture none` to disable it; synthetic CLI input requires this override. Hand-only and vertical demo modes do not provide this gesture.

## 操作方式

**左右手各自將拇指與無名指指尖捏合，兩手同時維持 2 秒。** 不是兩隻手互相接觸，也不是拇指與食指捏合。

1. 第一次辨識到完整雙手候選，立即鎖定停止新目標；該影格不進 IK。
2. 保持捏合，主控台與已啟用的 viewer 顯示確認進度。新有效影格累積滿 2 秒後正常退出。
3. 鎖定後若放開、缺手、過期或幾何無效，退出為「確認中斷」，**不恢復跟隨**。
4. 重新控制須重新執行程式、取得新 feedback 並重新校準。沒有自動回 home、張開機器手或重啟 SDK。

**2 秒是確認退出意圖，不是等待 2 秒才停止跟隨。** 完整候選即使很短暫，也會中斷會話。手勢形成前的過渡動作仍可能被機器人模仿。

這是軟體會話停止功能，不能取代硬體急停。「停止新目標」、「程式退出」及「機器人實際停穩」是不同事件；程式沒有驗證實際停穩或抓握力。

## 啟用指令與 ROS 前提

```bash
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py \
  --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50 \
  --stop-gesture dual-thumb-ring-pinch --stop-gesture-hold-s 2.0
```

不連 ROS 的 Quest 預覽可改為 `--backend preview`；不開 viewer 可加 `--no-viewer`。preview 仍會開啟 live Quest 輸入。

| 選項 | 行為 |
| --- | --- |
| `--stop-gesture` | `none`（預設）或 `dual-thumb-ring-pinch`。 |
| `--stop-gesture-hold-s` | 預設 `2.0`，必須有限且大於零。 |
| `--command-hz` | 啟用手勢時須大於 `1 / 0.15` Hz；建議保持目前的 20 Hz。 |

ROS 模式在發布任何命令前，透過 `/{crx_namespace}/joint_interpolation/get_parameters` 讀取下游 `method` 與 `ruckig_target_mode`，**必須為 `ruckig`／`waypoint`**。查詢失敗、逾時、參數缺失、型別錯誤或 stream／其他方法都拒絕啟用；constructor 清理也不發布 hold。

CRX launch 需明確包含 `method:=ruckig ruckig_target_mode:=waypoint`，不能僅依賴 launch 預設；本機檢查時其預設 target mode 是 stream。測試用的完整 mock 指令為：

```bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=true rviz:=false input_rate_hz:=100.0 \
  method:=ruckig ruckig_target_mode:=waypoint
```

程式不會修改 driver 參數。這是啟動時驗證，依賴目前 driver 的唯讀參數；操作期間不應替換或重啟為不同模式的 driver。retargeting 本身的 100 Hz **linear** 輸出與下游 Ruckig waypoint 是不同層，不由前者推測後者。

## 辨識與新影格規則

使用 mapping／`human_hand_scale` 之前的 `SensorHandSample.keypoints_wrist`，即 Quest 解碼後的 MANO21 腕部局部座標。純 NumPy 計算，不依賴 optimizer 或新增模型。

```text
w      = norm(p[5] - p[17])         # 食指根到小指根的掌寬
r_ring = norm(p[4] - p[16]) / w    # 拇指與無名指尖距離
r_i    = norm(p[4] - p[i]) / w     # 其他指尖 i = 8, 12, 20
```

| 設定（`StopGestureConfig`） | 預設 | 用途 |
| --- | ---: | --- |
| `pinch_enter_ratio` | 0.20 | 單手進入捏合：`r_ring <= 0.20`。 |
| `pinch_exit_ratio` | 0.30 | 已捏合時：`r_ring >= 0.30` 才視為放開。 |
| `other_tip_margin_ratio` | 0.08 | 其他三指距離須至少為 `r_ring + 0.08`。 |
| `fist_pip_angle_deg` | 110° | 食指、中指、小指 PIP 內角全部小於此值，排除為握拳候選。 |
| `max_sample_age_s` | 0.15 s | 資料 age 上限。 |
| `max_gap_s` | 0.15 s | 相鄰新影格／等待新影格的空窗上限。 |
| `hold_s` | 2.0 s | 雙手同時成立的確認時間。 |

PIP 內角取「指根－PIP－DIP」，伸直接近 180°。幾何檢查包含 `(21, 3)` 關鍵點、`(4, 4)` wrist pose、有限值、非退化掌寬與骨段。沒有加入初始化掌寬中位數／尺度突變濾波；這部分仍須真實資料決定門檻。

左右手必須具有相同 `source_index` 及 `raw.received_monotonic_ns`，並通過有效性檢查。以本機 monotonic 時鐘檢查 age，確認時間為最後與第一張候選影格的接收時間差：

- 兩手可先後捏合，從首次同時成立開始計時。
- 重複影格不累積進度；持續沒有新影格由 `poll()` 偵測逾時。
- 倒退 sequence／時間、相同 sequence 卻不同時間、新 sequence 卻時間未增加，都不能延續確認。
- 滿 2 秒須由一張新的有效雙手捏合影格觸發，不能只靠 timer。
- 確認中任何條件中斷即退出，不解除停止鎖定。

掌寬 7 cm 時，進入／離開距離約 1.4／2.1 cm。這是骨架距離，不保證指腹已接觸。**門檻尚未以真實 Quest、不同手型或一般抓握資料驗證誤觸率。**

## 程式流程與停止邊界

`BimanualExecutionFlow` 仍是唯一生命週期管理者。detector 只管理幾何、freshness 與確認狀態，不 import ROS、不直接控制 backend。

```text
RUNNING → STOP_LATCHED → EXIT_CONFIRMED
                     ↘ EXIT_UNCONFIRMED
```

每次 `step()` 先辨識，再允許 backend 初始化、校準與 IK。候選成立時先設不可恢復的 latch，再呼叫 backend 停止，並在 finally 中取消雙 process solver；之後只讀輸入確認手勢，不執行一般 tracking recovery。接受 solver 結果與 `execute()` 前也檢查 latch，丟棄停止時的晚到結果。

ROS backend 在同一把 lock 中取消 publication timer、清除 pending target、撤銷舊 generation，feedback 有效時送一次 measured-position hold。舊 timer 在鎖外算出的樣本必須通過 generation 複查才可發布；停止邊界之後不能發布舊移動目標。已在邊界之前發布的訊息無法收回，三個 topic 也不是原子傳輸。

停止具冪等性：後續 `request_stop()`／`close()` 不重複發 hold，也不能 `resume_tracking()`。若停止時 feedback／subscriber 不可用或發送失敗，仍撤銷輸出並回報錯誤；即使 solver 取消或個別清理失敗，也會嘗試清理其餘資源。

確認結束後關閉 backend、雙 solver process、Quest session 與 viewer。雙 process 架構及每側 30 ms optimizer 預算保持原設定。

### 延遲與下游限制

辨識在主流程中執行，並非獨立接收執行緒。正在等待 solver、Quest 啟動或 backend 初始化時不能立即處理手勢；30 ms optimizer 預算不是手勢反應上限。直接 `step()` 可在 backend 啟動前攔截候選，`run()` 則仍先啟動 solver 與輸入。

waypoint 的 measured hold 仍是位置目標，實際減速由下游決定；mock 不能證明實機停止後沒有額外偏移。現有 stream 的 off-cycle measured hold 可能被差分為高速終端狀態，故此功能不支援該模式，背景見 [錄製分析](docs/vertical-demo-recording-analysis-20260928.md)。

Sharpa SDK driver 的 command timeout 目前預設 0.5 秒，實際依 launch 設定。這 2 秒不再發布移動目標，SDK 可能因此停止並關閉 session；程式不靠重送舊目標避開 timeout，也不重連。這不是實機抓握力保持的保證。

## 提示與退出碼

主控台顯示狀態、進度及原因（約每 0.25 秒更新）；有 viewer 時同步顯示。尚未加入 Quest 頁面內提示或聲音。

| Exit code | 意義 |
| --- | --- |
| 0 | 手勢確認完成，或原有正常有限會話結束。 |
| 1 | backend、worker、啟動或清理失敗。 |
| 2 | 已鎖定停止，但手勢確認中斷。 |
| 130 | Ctrl+C。 |

清理錯誤不能被正常確認結果蓋過。中途 duration 到期或 Ctrl+C 不等待手勢滿 2 秒。

## 檔案與驗證

| 檔案 | 實作 |
| --- | --- |
| `src/teleoperation/stop_gesture.py` | 幾何、遲滯、新影格與不可恢復的確認狀態。 |
| `src/teleoperation/bimanual_execution.py` | IK 前攔截、停止鎖定、取消 solver、確認迴圈及清理。 |
| `src/retargeting_apps/sharpa_teleop.py` | CLI、支援範圍、進度與退出碼。兩個薄入口腳本傳回退出碼。 |
| `src/retargeting_ros/sharpa_joint.py` | waypoint 啟用檢查、一次性 hold、停止錯誤與清理。 |
| `tests/test_stop_gesture*.py` | 幾何、流程、CLI、雙 process 與隔離 ROS mock。 |
| `tests/test_sharpa_interpolation.py`、`tests/test_sharpa_joint.py` | timer race、單次 hold、失敗撤銷、資源清理。 |

2026-09-28 已驗證離線幾何／狀態機、preview 的真實雙 solver process，以及 localhost ROS domain 194 的隔離 CRX／Sharpa mock（無 RViz、無硬體）：

- waypoint 持續捏合：2.01 秒完成確認；5 筆 solver target 後鎖定，三個 topic 共涵蓋 56 軸，各 22 筆插值訊息加 1 筆 hold，邊界後沒有新訊息。
- waypoint 提早放開：約 0.31 秒確認中斷，不再送目標或恢復。
- stream：啟動拒絕，三個 command topic 零訊息，包含清理階段。
- 三個案例都確認雙 worker 正常退出、ROS feedback thread 關閉。

上述時間是單次 mock 觀測，非最壞延遲或實機停穩量測。**尚未執行真實 Quest 手勢或硬體驗證。**

最後相關回歸測試共 172 項通過，隔離 ROS mock 3 項通過，`compileall` 與 `git diff --check` 通過。完整 headless suite 本次為 428 passed、15 skipped、9 failed：其中 6 項 localhost socket 測試在允許本機 socket 後全部通過；剩餘 3 項是工作區原有 LEAP URDF 修改造成的 FK／manifest hash 不一致，未改動該模型或測試期望值。

離線測試：

```bash
env -u PYTHONPATH .venv/bin/python -m pytest \
  tests/test_stop_gesture.py tests/test_stop_gesture_backend.py \
  tests/test_sharpa_joint.py tests/test_sharpa_interpolation.py -q
```

隔離 ROS 測試須先 source ROS Jazzy 與 driver workspace，然後明確選用；測試自行啟動 mock 並固定 localhost domain 194：

```bash
STOP_GESTURE_ROS_TEST=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv/bin/python -m pytest tests/test_stop_gesture_ros.py -q -s
```
