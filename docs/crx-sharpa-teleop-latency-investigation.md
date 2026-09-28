# CRX + Sharpa 遙操作延遲與通訊調查

調查日期：2026-09-28。依據本機工作樹：`retargeting_crx` HEAD `b22e860`、`dual_crx_control` HEAD `4ab8c85`、`dual_sharpa_wave_ros2` HEAD `09e6feb`。HEAD 僅用於定位版本，實際分析包含當下工作樹內容。

**目前實作已更新：兩支 Sharpa 腳本皆使用雙 process，每側 NLopt 時限 30 ms。** 後文的依序 25 ms 與 thread／process 25 ms 表格保留為修改前調查及原型量測；最新行為與 ROS mock 驗證見「正式流程整合：雙 process／30 ms」。

## 結論

**原本延遲差異最有證據的來源是：加入 CRX 後，每側改成手臂與手部一起求解，兩側又依序執行；手指必須等整批求解完成才有新目標。** 修改前不連設備的比較中，純雙手求解中位數約 **18.7 ms**，CRX + 雙手約 **39.6 ms**，增加約 **21 ms**；合併模式約 21–23% 的測量樣本超過 20 Hz 對應的 50 ms 預算。

此外，手指指令本身還經過 **20 Hz 更新、alpha=0.3 平滑、50 ms 線性插值，以及 Sharpa SDK 插值／速度設定**。因此 `--publish-hz 100` 代表每 10 ms 發布一次中間值，並不代表每 10 ms 求解一次新手勢。

**手指沒有經過 FANUC 控制器或 CRX 的 Ruckig 插值。** 三個命令 topic 分開發布；CRX 對手指的影響主要是上游求解成本、共用程式資源及回授健康檢查，而不是手指資料繞過手臂控制器。

後續離線原型已比較雙 thread 與雙 process：雙 process 在保留每側 25 ms 時限時，整批中位數約 21 ms；詳見「雙 process 離線原型結果」。經使用者確認，正式 Sharpa 流程已採雙 process／30 ms，並在沒有硬體的 ROS mock 環境測試；未啟動 Quest、viewer 或實機。尚未取得實機時間序列，因此無法宣稱已測出整體延遲、DDS 傳輸延遲或 SDK 內部延遲。

## 調查範圍與比較基準

使用者的 retargeting 指令：

```bash
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py \
  --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50
```

機器人端指令：

```bash
ros2 launch dual_crx_control dual_arm.launch.py \
  mock:=false rviz:=true input_rate_hz:=100.0
ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=sharpa_sdk
```

本機對應工作區實際是 `~/ws_fanuc`，兩個 package 位於：

- `~/ws_fanuc/src/dual_crx_control`
- `~/ws_fanuc/src/dual_sharpa_wave_ros2`（ROS package 名稱是 `dual_sharpa_wave`）

本機 install 中的 CRX launch 連到 source；Sharpa hardware YAML 經 build 連回 source，且比對內容相同。但使用者執行終端的 overlay、參數覆寫及實際 ROS graph 尚未查證。

「單獨 Sharpa」的完整指令尚待確認，本報告先以 `scripts/run_sharpa_joint_teleop.py --backend ros` 為主要比較對象：它與合併腳本共用 `sharpa_teleop.main()`，只差 `with_arms=False/True`。若原比較對象是 SDK 直接控制或 ROS GUI／波形腳本，則那些路徑會省去 Quest、retargeting 求解及其平滑／插值，差異自然更大。單獨 `dual_sharpa.launch.py` 只啟動驅動，並不產生手勢目標。

## 現在如何建立通訊

```mermaid
flowchart TD
    Q[Quest Browser WebXR 雙手追蹤] -->|WebSocket JSON /ws；ADB reverse TCP 8765| I[Quest3Receiver：保存最新 frame]
    I --> F[BimanualExecutionFlow：目標 20 Hz]
    F --> L[左 process：6 arm + 22 hand；30 ms]
    F --> R[右 process：6 arm + 22 hand；30 ms]
    L --> J[同一幀左右結果匯合]
    R --> J
    J --> S[輸出平滑：arm 0.5 / hand 0.3]
    S --> B[SharpaJointBackend：50 ms 插值；100 Hz timer]
    B -->|JointState：12 joints| A["/crx5ia/joint_targets"]
    B -->|JointState：22 joints| LH["/sharpa/left_hand/joint_command"]
    B -->|JointState：22 joints| RH["/sharpa/right_hand/joint_command"]
    A --> C[CRX Ruckig stream；500 Hz]
    C --> P[左右 forward_position_controller]
    P --> H[FANUC hardware interface → 左右 CRX]
    LH --> SL[左 hand_node → Sharpa SDK → 左手]
    RH --> SR[右 hand_node → Sharpa SDK → 右手]
    H --> M["/crx5ia/joint_states：合併回授"]
    SL --> FL["/sharpa/left_hand/joint_states"]
    SR --> FR["/sharpa/right_hand/joint_states"]
    M --> B
    FL --> B
    FR --> B
```

### 1. Quest 到 retargeting

`Quest3BimanualOnlineInput.open()` 啟動 USB session、ADB reverse 與本機 WebXR receiver。Browser 以 WebSocket 傳送同一 frame 的左右手；decoder 將 WebXR 關節轉成 retargeting 使用的 21 點手部資料。接收端保存最新 frame，不在求解端逐筆補算舊 frame。

主迴圈以 `--command-hz` 排程讀取、映射、求解；兩手要同時有效。輸入 freshness 預設 150 ms，求解完成後也重新檢查。這個年齡由桌面端收到封包開始算，**不包含 Quest 追蹤、Browser 或收到封包以前的傳輸時間**。

### 2. retargeting 到 ROS

`SharpaJointBackend` 建立 `/retargeting_sharpa_joint` node，以背景 `SingleThreadedExecutor` 處理回授、watchdog 及輸出 timer。ROS 2 透過所選 RMW／DDS 做 discovery 與 topic 通訊；實際走本機共享記憶體或網路、選用哪種 RMW，不能僅從腳本判定。不同終端需有相容的 ROS domain、discovery 範圍與 QoS。

此路徑是 **直接發布 JointState**；沒有使用另一套 `dual_crx_ros2` gateway 的 lease、`TeleopCommand` 或 enable service。不可把其他 CRX backend 的說明套在這兩支 Sharpa 腳本上。

| 方向 | Topic | 資料與名義頻率 |
| --- | --- | --- |
| 發送雙臂 | `/crx5ia/joint_targets` | `sensor_msgs/msg/JointState`；`left_J1..J6`、`right_J1..J6`，12 個 rad；100 Hz |
| 發送左手 | `/sharpa/left_hand/joint_command` | `JointState`；22 個 `left_*` 關節，rad；100 Hz |
| 發送右手 | `/sharpa/right_hand/joint_command` | `JointState`；22 個 `right_*` 關節，rad；100 Hz |
| 接收雙臂 | `/crx5ia/joint_states` | 左右 controller 回授由 merger 合併；100 Hz 設定 |
| 接收雙手 | `/sharpa/{left,right}_hand/joint_states` | 每隻 hand node 的量測角度，rad；100 Hz 設定 |

retargeting pub/sub 與 Sharpa driver 使用 `KEEP_LAST(depth=1), RELIABLE, VOLATILE`。depth=1 減少應用層舊訊息排隊，但不保證零延遲或 real-time deadline，也不排除 DDS／SDK 內部等待。

啟動時最多等待 5 秒，要求所有使用中 channel 有完整且新鮮的回授，以及命令 subscriber。合併模式等待三路，純雙手模式等待兩路。校正及插值起點使用量測姿態。這是啟動條件，正常運行不會每筆等待機器人到位或回覆 ack。

每次求解完成只替換一個 pending target。100 Hz timer 對完整 56 維向量取樣，再依序發布三個 channel，使用同一 ROS timestamp；**不同 topic 並非原子、同步到達的傳輸**。timer 不會把錯過的時槽補發成一串舊命令。

### 3. ROS 到 Sharpa

launch 為左右手各啟動一個獨立 `hand_node` 程序，以 hardware YAML 中明確指定的 serial number 連接各自裝置。

連線流程：`SharpaWaveManager.get_instance()` → discovery 等待指定 serial → `connect(serial)` → POSITION mode → speed/current coefficient → SDK control source → `start()` → 讀取回授。

收到 `joint_command` 時，`HandNode._on_command()` 驗證並依關節名稱排序，立即呼叫 adapter；adapter 再轉成 SDK 關節順序，呼叫 `set_joint_position(radians, interpolation)`。**沒有刻意等下一個 100 Hz feedback timer 才寫入命令。**

feedback timer 執行 `get_joint_position_degree()`，轉成 rad 和 URDF 關節順序後發布；timestamp 是 node 讀回後蓋上的時間，並非已證明的馬達採樣時間。命令 callback 與 feedback timer 共用該 node 的 executor，因此 SDK 同步讀／寫若阻塞，另一個 callback 也會受影響。

目前 hardware YAML：

| 參數 | 左右手值 | 意義 |
| --- | --- | --- |
| `publish_rate_hz` | `100.0` | 回授／update timer，不是命令限速器 |
| `speed_coeff` | `0.3` | SDK 速度係數；不能直接換算成固定延遲或 rad/s |
| `current_coeff` | `0.6` | SDK 電流係數 |
| `interpolation` | `true` | 每次寫入都啟用 SDK 插值 |
| `command_timeout_sec` | `0.0` | driver 命令 timeout 關閉，與上游 watchdog 不同 |

ROS adapter 只指定 serial，裝置 discovery、實體網路連線與內部排程由 SDK 管理。安裝的 SDK header 有 Ethernet／TCP 相關介面與錯誤定義，但未提供足夠資訊證明本次關節命令的完整 wire protocol、port 或緩衝策略；本次也沒有初始化 SDK 或擷取封包。

### 4. ROS 到 CRX

`/crx5ia/joint_targets` → `joint_interpolation` → `/crx5ia/{left,right}/forward_position_controller/commands`（`Float64MultiArray`）→ 各自 500 Hz `controller_manager` → FANUC hardware interface。

**以目前 launch 的可執行設定為準，使用者指令實際選到 `method=ruckig`、`ruckig_target_mode=stream`、timeout 0.2 秒。** 檔頭仍有 waypoint 預設／stream opt-in 的舊說明，與 `DeclareLaunchArgument` 不一致。`input_rate_hz=100` 是上游參考週期 10 ms，並不把輸出設定為 100 Hz；插值與 controller 設定皆為 500 Hz。Ruckig 的速度、加速度、jerk 限制影響手臂，不直接處理手指。

實體 driver 使用 Stream Motion（程式中為 UDP，Xacro 預設 port `60015`）與 RMI（TCP，這份 Xacro 預設連線 port `16001`，RMI 握手後可轉到 controller 回傳的 port）。目前 launch 實際預設 IP 是左 `192.168.2.100`、右 `192.168.1.100`；檔頭示例 IP 也與實際宣告不同。以上是程式預設，沒有確認現場連線或控制器內部伺服頻率。

## 延遲來源：哪些已確認、哪些仍需量測

### A. 加入手臂後，手指等更久才取得新目標：已由離線比較支持

純雙手每側是 22 維，合併模式每側是 28 維。修改前 `BimanualRetargetingPipeline.step()` 先左側 `solve()`，再右側 `solve()`，兩側都完成後才返回。現在 Sharpa 模式委派兩個 process 平行求解，再匯合返回；每側仍是 joint retargeting，沒有獨立的高速手指求解迴圈。

原本 `build_flow()` 給每側 NLopt `maxtime=0.025` 秒，現在是 `0.030` 秒。這是求解器時間預算，不是整批硬性時間上限，也不是固定等待時間。mapping、Python callback、filter、viewer 等都有額外成本。主迴圈超時後採 `max(原本起點 + period, 現在時間)` 排下一輪，並有 2 ms sleep；不會為維持 20 Hz 補算漏掉的 frame。

合併 profile 還增加了世界座標拇指與腕部旋轉目標（左側例：`world_thumb` 0 → 10、`wrist_rotation` 0 → 0.1），所以不只是多傳 12 個數值，優化問題本身也不同。輸出平滑結果會作為下一輪 `previous_qpos`；objective 有對前一姿態的正則化，可能進一步影響快速變化時的收斂與追蹤，需要以真實手勢驗證。

### B. alpha=0.3 帶來明顯跟隨滯後：兩支 retargeting 腳本共同存在

手指每個新求解目標只濾波一次：

```text
filtered[k] = 0.3 * raw[k] + 0.7 * filtered[k-1]
```

假設 raw target 瞬間跳到固定新值，7 次更新才超過 90%，9 次更新才超過 95%。以 20 Hz 計，從第一次更新到第 7／9 次分別跨 300／400 ms，還需考慮第一次更新前的等待及後續插值。這是階躍響應的漸進收斂，不是每筆固定延後 300 ms。

對緩慢變化訊號，這個 filter 的低頻等效延遲約 `(1-alpha)/alpha * T`：20 Hz 時約 **117 ms**；若實際只剩 15 Hz，約 **156 ms**。因此合併求解一旦降低有效目標頻率，相同 alpha 在真實時間上會更慢。這些是 filter 模型推算，並非實機延遲量測。

兩份 bimanual YAML 都是 hand alpha=0.3、arm alpha=0.5，所以不能只以平滑存在解釋兩支腳本的差異。當前 `limit_joint_speed=false`，profile 內的 `max_joint_speed` 不會在這條 retargeting 路徑額外形成速度限制。

### C. 50 ms 插值與 SDK 插值：已確認有兩層，內部效應未量測

上游線性插值從最近發布的位置走向新目標，終點時間是 `backend.execute()` 收到目標的時間加 50 ms。正常 timer 下，第一筆變化通常等到接下來的 10 ms tick；新目標可中途取代原 segment。這不是固定 50 ms 的資料佇列，也不應把「tick 等待」和完整 50 ms 無條件相加。

SDK 收到的已是插值後目標，卻仍傳入 `interpolation=True`。SDK 是否重新規劃每個 target、是否有固定緩衝、100 Hz 更新與 speed coefficient 如何交互作用，目前 adapter／header 無法回答。若純雙手用相同 driver，此 SDK 層也是共同因素。

只提高 `--publish-hz` 不會縮短手部 filter 的時間尺度，也不會加速求解。CLI 要求 horizon 大於 0；不能用 `--interpolation-horizon-ms 0` 關閉插值。過短 horizon 若在 timer 處理前已到期，會被拒絕並暫停輸出，因此也不能任意降到小於 10 ms。

### D. Viewer／CPU／Python 排程：合理候選，尚未量測

原指令未指定 `--no-viewer`，所以 retargeting 預設啟用 Viser；CRX launch 又啟用 RViz。`BimanualExecutionVisualizer.update()` 直接掛在 flow observer，更新模型 FK、mesh pose 和人手顯示，位於本輪發布目標之後、下一輪求解之前。模型變大會增加工作量；solver 的 Python callback、背景 ROS timer 與 viewer 也可能競爭 CPU／GIL。

`solve_ms` 只涵蓋 `pipeline.step()`，不含 observer 或整個週期，因此只看它不能排除 viewer 造成的降頻。本次 benchmark 沒有啟動 viewer，也沒有量測 GIL 或 DDS callback 排程。

### E. 暫停／恢復可能被感覺成 delay：合併模式多一個 CRX 回授依賴

backend 檢查所有 channel：回授缺失、凍結或超過 500 ms，以及命令 subscriber 消失，都會暫停整組輸出；合併模式的 CRX 回授異常因此會連帶停手指。新求解目標超過 250 ms 未更新也會暫停，100 Hz 重複發布不會延長舊目標壽命。

Quest 任一手缺失／沒有新鮮 frame 會進入 hold；恢復要求持續有效約 300 ms，再重校正並讀取新 frame。求解後超過 150 ms 的 frame 會丟棄、增加 `stale`。這些情況更像間歇卡住或整組停頓，與持續平滑落後要分開診斷。

## 本次離線驗證

以現有 `SyntheticBimanualInput` 和真正的左右 solver，在本機 `.venv` 做純 CPU 比較。每輪 120 frame，前 20 frame 不計入耗時統計，後 100 frame 記錄 `flow.last_solve_ms`；順序為手、合併、合併、手。兩種模式均維持預設 25 ms／側預算及輸出平滑。

| 模式／輪次 | 求解 p50 | p95 | 最大 | 大於 50 ms |
| --- | --- | --- | --- | --- |
| 純雙手 1 | 18.64 ms | 25.84 ms | 29.98 ms | 0/100 |
| CRX + 雙手 1 | 39.44 ms | 51.60 ms | 51.82 ms | 21/100 |
| CRX + 雙手 2 | 39.78 ms | 51.60 ms | 51.78 ms | 23/100 |
| 純雙手 2 | 18.71 ms | 25.60 ms | 30.55 ms | 0/100 |

每輪均完成 120 commands，`stale=0`。這是直接呼叫 `flow.step()` 的計算耗時，沒有 20 Hz pacing、ROS timer、USB、SDK、機器人或 viewer；**不是實際 command Hz，也不是端到端延遲**。合成資料主要是固定手腕下的手指彎曲，真實手腕移動及追蹤品質會改變結果。

可在 repository 根目錄重跑原本的依序／25 ms 比較（明確停用此測試物件的 process，僅供離線比較）：

```bash
env -u PYTHONPATH .venv/bin/python - <<'PY'
from types import SimpleNamespace
import numpy as np
from retargeting_apps.sharpa_teleop import build_flow
from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput

args = SimpleNamespace(config=None, backend='preview', duration=0.,
    command_hz=20., adb=None, serial=None, viewer=False, viewer_port=9219)
for arms in (False, True, True, False):
    source = SyntheticBimanualInput(frames=120)
    flow, _ = build_flow(args, with_arms=arms, source=source)
    flow.pair_solver = None
    for retargeter in (flow.pipeline.left_retargeter, flow.pipeline.right_retargeter):
        retargeter.optimizer.opt._opt.set_maxtime(.025)
    source.open()
    times = []
    try:
        for i in range(120):
            flow.step(source.read())
            if i >= 20:
                times.append(flow.last_solve_ms)
    finally:
        source.close()
    print('arms=', arms, 'p50/p95/max=', np.percentile(times, [50, 95, 100]),
          'over50=', sum(t > 50 for t in times),
          'commands=', flow.command_count, 'stale=', flow.stale_count)
PY
```

另外執行以下既有 headless tests：**61 passed，無 skip**。

```bash
env -u PYTHONPATH .venv/bin/python -m pytest \
  tests/test_sharpa_execution.py tests/test_sharpa_interpolation.py \
  tests/test_bimanual_execution.py -q
```

它們驗證實際 solver 組合、平滑、插值與暫停／恢復邏輯，不驗證硬體或 ROS 網路時間。

## 平行求解評估與後續時限選擇

**初步 thread 評估決議：每側 35–40 ms 作為候選時限。** 後續在獨立離線原型中比較這兩個值與雙 process，結果見下一節；最後正式整合選擇雙 process／30 ms。35–40 ms 是 thread 階段的候選值，不是目前 runtime 設定，也不是已驗證的實機最佳值。

### 可平行化的範圍

左右側各自建立 retargeter、NLopt optimizer 和 Pinocchio model/data；本次離線檢查也確認左右不是同一個 optimizer／model instance。因此可以考慮用常駐 `ThreadPoolExecutor(max_workers=2)`，同時處理同一 Quest frame 的左側與右側求解，再匯合結果做平滑與發布。

這裡的工作單位是「左 CRX + 左手」及「右 CRX + 右手」，並未把同一側的手臂和手指拆成不同優化問題。單一 optimizer 內含可變 callback、FK data、暫存陣列與 `previous_qpos`，不能同時處理前後兩幀。

### 已完成的 thread 局部比較

在合併模型中，先用合成輸入執行 20 frame，取下一 frame 的左右 observation，保存各側 seed；每次比較都使用相同 observation 和 seed。以常駐兩個 worker 對照依序呼叫 `solve()`，順序皆為依序、thread、thread、依序。沒有 ROS、viewer 或設備，也沒有修改專案程式。

| 條件 | 依序整批 p50 | thread 整批 p50 | thread 整批 p95 | 每側 objective 評估次數中位數 |
| --- | --- | --- | --- | --- |
| 每側保留 25 ms 時限 | 39.58–39.82 ms | 26.13–26.26 ms | 26.40–26.51 ms | 依序左 42／右 41；thread 左右各 33 |
| 僅實驗物件取消時限，維持原收斂條件 | 40.45–40.52 ms | 35.21–35.56 ms | 36.42–37.20 ms | 兩種方式皆左 42／右 43 |

25 ms 組每輪執行 35 次、排除前 5 次；取消時限組每輪 25 次、排除前 5 次。表中範圍是兩輪結果，不是信賴區間。

保留 25 ms 時限時，依序 objective cost 約為左右各 `0.073917`，thread 約為左 `0.074215`／右 `0.074210`；較短整批時間伴隨較少評估及略高 cost，不能全部視為同品質的加速。取消時限組的評估次數和最終 cost 一致，與依序基準相比最大輸出角度差為 `0 rad`，局部耗時改善約 12–13%。這些數字只代表該固定合成姿態，不代表完整動作序列或實機表現。

### 為何 thread 下要評估 35–40 ms

NLopt 的 `maxtime` 是每個 solver 經過的 wall-clock 時間，不是獨占 CPU 計算時間；等待 GIL 和 CPU 排程也會消耗預算。此 objective 每輪會回到 Python，執行 FK／Jacobian、PyTorch loss 及梯度操作。部分原生運算能釋放 GIL，但 Python callback 仍會競爭，所以兩個 thread 無法保證全程同時運算。上述實驗支持存在競爭，但沒有對 GIL 等待時間做獨立 profiling。

同收斂結果的 thread 整批約 35–37 ms，支持將 **35 ms／側與 40 ms／側** 作為後續測試候選，保留 25 ms／側作對照。時限是允許的上限，提早收斂就會返回；它不是固定等待時間，也不保證嚴格 deadline，單次 callback 及排程可能使實際耗時超過設定。

兩側平行後，整批時間應以實際 submit 到兩側完成量測；有有效重疊時接近較慢一側的經過時間加上派工／匯合成本，不能直接把 40 + 40 當作 80 ms，也不能保證整批一定小於 40 ms。20 Hz 只有 50 ms 週期，必須另外保留 mapping、filter、viewer、ROS 競爭與排程餘裕。

後續驗證應同時比較：整批及完整週期的 p50／p95、超過 50 ms 的比例、有效目標 Hz、`stale`／pause、objective 評估次數，以及追蹤誤差和輸出品質。不要只用更短耗時判定方案成功；姿態變化序列、手腕運動及 viewer／ROS 負載也需涵蓋。

### 未來實作需要保留的行為

- 常駐 worker，避免每幀重新建立 thread；每側同時只允許一個 solve。
- 同一 frame 的左右結果完成後才匯合；reset、校正、filter 及 `previous_qpos` 更新不可與該側 solve 重疊。
- 維持最新 frame 語意，不累積舊工作；求解後 freshness 檢查、tracking pause、stop 與錯誤傳遞仍要生效。
- ROS timer／watchdog 保留原有責任；一側 worker 失敗時不可任意發布另一側未配對的結果。

兩個常駐 process 是另一個候選：各自建立一側 solver，可避開 Python GIL 的共用，但增加初始化、資料傳輸及狀態同步成本。初步 thread 階段尚未做 process benchmark；下一節記錄後續以 25 ms／側完成的離線比較。無論採用哪種並行方式，hand alpha=0.3 與 50 ms 插值的跟隨滯後仍需另外評估。

## 雙 process 離線原型結果（2026-09-28）

**原型結果支持選擇兩個常駐 process。** 本節記錄當時每側 25 ms 的獨立 benchmark；後續依使用者要求，以每側 30 ms 接入正式流程。35–40 ms 是 thread 的候選範圍，不需要直接套用到 process。

### 原型與比較方式

原型：[benchmark_sharpa_parallel.py](../scripts/benchmark_sharpa_parallel.py)。左右各一個 `spawn` worker，各自建立 solver，透過獨立 `Pipe` 收取 frame ID、observation、seed 和時限，回傳 qpos、objective、評估次數及 NLopt status。父端同步等待同幀左右結果，一次最多一筆工作／側，沒有跨幀佇列。worker 不建立 ROS node、SDK session 或 viewer。

為沿用正式組合邏輯，每個 worker 初始化時先呼叫既有雙側 composition，再釋放不用的另一側與 flow，只留下該側 solver。這是原型的啟動成本，不是每幀成本。

本機環境為 Python 3.12.3、NumPy 2.5.3、CPU PyTorch 2.14.0、NLopt 2.11.0，可用 CPU affinity 16 個邏輯 CPU。兩組比較分別限制數值函式庫為 1 thread，或保留環境預設（本機 PyTorch intra-op 8、inter-op 16）。每組各兩輪，第二輪反轉 case 順序；每個 case 先跑 20 frame，再量測 100 frame。

資料包含既有合成手指彎曲，加上約 1–1.5 cm 手腕平移與小幅旋轉。先用不設時間上限、維持既有收斂條件的依序求解產生參考，再依原 arm/hand alpha 建立下一幀 seed。所有模式收到**完全相同的 observation 與 seed**，比較的是相同工作，不讓不同模式前一幀的誤差改變下一幀難度。這是受控 replay，不是各模式獨立演進的完整遙操作回放。

`pair_ms` 從派工前量到兩側結果返回，process 包含 IPC；也包含有效性檢查及每側額外一次不求梯度的 objective 評估。後者用於衡量實際回傳的裁切／float32 qpos，不是正式 runtime 的必要步驟。mapping、reference 生成、啟動／關閉、記憶體快照、ROS、viewer 和 20 Hz pacing 均不在 `pair_ms` 內。`budget_ms=0` 僅在 benchmark 表示停用 NLopt maxtime，不是正式 teleop CLI 設定。

### 耗時與收斂結果

下表為數值 thread=1 的兩輪範圍，每個模式共 200 筆量測；RMSE 是相對不設時限參考的 **56 維原始關節角差異**，不是對真實手勢的準確度。

| 模式／每側時限 | 整批 p50 | 整批 p95 | 超過 50 ms | 原始 qpos RMSE |
| --- | --- | --- | --- | --- |
| 依序／25 ms | 39.02–39.19 ms | 51.71–51.93 ms | 24/200 | 0.0395–0.0398 rad |
| 雙 thread／25 ms | 26.63 ms | 26.99–27.12 ms | 0/200 | 0.0773–0.0786 rad |
| 雙 thread／35 ms | 35.11–35.24 ms | 36.89–36.95 ms | 0/200 | 0.0543–0.0572 rad |
| 雙 thread／40 ms | 35.09–35.15 ms | 41.83–41.89 ms | 0/200 | 0.0435–0.0450 rad |
| **雙 process／25 ms** | **20.67–20.84 ms** | **26.34–26.44 ms** | **0/200** | **0.0402–0.0403 rad** |
| 依序／不設時限 | 39.14–39.20 ms | 60.30–60.64 ms | 28/200 | 0 |
| 雙 thread／不設時限 | 34.91–34.93 ms | 51.87–52.39 ms | 14/200 | 0 |
| **雙 process／不設時限** | **20.67–20.82 ms** | **32.12–32.20 ms** | **0/200** | **0** |

雙 process／25 ms 相較依序／25 ms，整批中位數縮短約 47%。每側平均 objective 評估次數約 39.31，與依序約 39.37 接近；碰到 maxtime 的比例約 15–16%，也與依序約 15% 接近。雙 thread／25 ms 則約 31.7–31.9 次，約 83% 求解碰到 maxtime，因此不能只看它的 26.6 ms 就認定品質相當。

不設時限時，三種模式各幀 qpos 與 reference 完全一致，objective 差異為 0，每側平均評估次數 40.48。雙 process 的速度改善因此不只是提早終止求解。此組 process 最慢樣本約 47.9 ms，已接近 50 ms，仍不能拿這組結果當成現場的最壞時間保證。

**時間上限仍會影響個別姿態。** 相對不設時限參考，依序／25 ms 最大單一關節差約 0.843 rad，process／25 ms 約 0.857–0.869 rad；thread／35–40 ms 也有約 0.90–0.99 rad 的個別差異。平均 objective 差很小不代表每個關節都接近；這些是原始求解角差，不是平滑後實機動作。參考解本身也不是動作真值或全域最優保證，後續仍需檢查姿態、任務空間誤差與時序連續性。

### 程序生命週期與資源

保留函式庫預設的第二組比較中，process／25 ms 整批 p50 約 21.10 ms、p95 約 26.29–26.34 ms，仍為 0/200 超過 50 ms；process／不設時限 p50 約 20.98–20.99 ms、p95 約 32.23–32.89 ms，與參考 qpos／objective 仍完全一致。這次小型矩陣工作沒有顯示預設數值 thread 數造成明顯退化；不代表加上 ROS／viewer 或其他 CPU 負載後仍然相同。後續整合可先固定每個 worker 數值 thread=1，減少環境變動。

| 額外成本／清理 | 本次觀察 |
| --- | --- |
| 建立兩個 spawn worker 並等 ready | 約 0.999–1.035 秒；不含在每幀耗時 |
| 正常關閉並回收兩個 worker | 約 0.198–0.206 秒 |
| 每個 worker 的實際 RSS | 約 358–360 MiB（函式庫預設組，解完 case 後快照） |
| 每個 worker 的 PSS | 約 261 MiB；共享頁面按使用程序數分攤 |
| 父程序 + 兩個 worker 合計 PSS | 約 800–813 MiB；相鄰無 worker case 約 368–379 MiB，增加約 432–433 MiB |
| 兩組效能比較共 16 個 solver worker | 全部 graceful exit、exit code 0、`alive=false`，無需 terminate／kill |

記憶體以 `/proc/<pid>/smaps_rollup` 的當下 RSS/PSS 為準；JSON 的 `peak_rss_mib` 是程序歷史高水位，不可拿來估算新增的實體記憶體，也不可把兩個 worker 的 RSS 直接相加當作獨占記憶體。PSS 比較包含 Python／PyTorch、reference 資料與 benchmark 父程序，不是正式整合的精確記憶體預算。重複 case 的父程序配置快取及資料保留也會影響數字，本次不是長時間記憶體洩漏測試。

原型在正常結束送 stop，限時 `join()`；仍未結束才 `terminate()`，必要時 `kill()`，並再次 join，最後關閉 pipe 與 process handle。單側失敗或等待中 `KeyboardInterrupt` 會清理兩側。worker 在等待下一筆工作時若收到 pipe EOF 會退出。回傳 frame ID 不符則拒絕結果並關閉 worker，不發布未配對資料。

測試：[test_sharpa_parallel_prototype.py](../tests/test_sharpa_parallel_prototype.py)，涵蓋初始化失敗、單側求解例外、程序直接退出、卡住後強制終止、舊 frame、父端 pipe EOF、等待回覆時注入 `KeyboardInterrupt`，以及真正 solver 的三模式一致性和正常退出。

最終檢查：原型與既有 Sharpa／bimanual 相關測試共 **69 passed，無 skip**；27 個文件連結、兩個新 Python 檔的語法、`git diff --check` 均通過。兩份 JSON 共 32 個 case、3,200 筆量測 frame pair，記錄的 16 個效能測試 worker PID 在結束後皆已不存在。

```bash
env -u PYTHONPATH .venv/bin/python -m pytest \
  tests/test_sharpa_parallel_prototype.py tests/test_sharpa_execution.py \
  tests/test_sharpa_interpolation.py tests/test_bimanual_execution.py -q
```

上述原型只用 EOF 偵測父端中斷，沒有驗證父程序 SIGKILL 或 ROS shutdown；正式整合已另外加入 Linux parent-death signal 及相應測試，見下一節。原型等待回覆 timeout 是 10 秒、正常 close 的等待是 2 秒，屬 benchmark 清理設定，不是正式流程設定。

### 重現與原始資料

從 repository 根目錄執行；`--output` 必須是新檔案，避免覆寫之前結果：

```bash
env -u PYTHONPATH .venv/bin/python scripts/benchmark_sharpa_parallel.py \
  --frames 100 --warmup 20 --rounds 2 --native-threads 1 \
  --output outputs/benchmarks/sharpa_parallel_threads1_repeat.json

env -u PYTHONPATH .venv/bin/python scripts/benchmark_sharpa_parallel.py \
  --frames 100 --warmup 20 --rounds 2 --native-threads 0 \
  --output outputs/benchmarks/sharpa_parallel_defaults_repeat.json
```

本次生成資料為 [限制數值 thread=1](../outputs/benchmarks/sharpa_parallel_20260928_threads1.json) 與 [保留函式庫預設](../outputs/benchmarks/sharpa_parallel_20260928_defaults.json)。JSON 包含逐幀耗時、objective gap、角度差指標、NLopt status、啟動／關閉時間及 worker 回收結果；函式庫預設組另記錄了 RSS/PSS 快照。這些是本機生成量測資料，不是追蹤的 golden fixture。

這些受控原型結果支持 process 的計算收益；各模式獨立演進的序列品質與其他背景負載仍值得追加量測，也不代表已解決 alpha=0.3、50 ms 插值、SDK 或實機通訊造成的延遲。

## 正式流程整合：雙 process／30 ms

`run_sharpa_joint_teleop.py` 與 `run_crx_sharpa_joint_teleop.py` 現在都使用 [BimanualProcessSolver](../src/teleoperation/parallel_solver.py)，preview 與 ROS 模式一致，CLI 不變。左右 worker 各自從四份 typed config 建立該側 Retargeter、Pinocchio model/data 與 NLopt optimizer，每側 `maxtime=0.030` 秒，數值運算 thread 數設為 1。正式 worker 不再像原型先建立完整雙側 flow。

`BimanualExecutionFlow` 是唯一 lifecycle owner：在 `source.open()` 前啟動並等待兩個 worker ready，之後才取得新輸入、建立 backend 及校正。直接呼叫 `flow.step()` 的工具應先 `flow.start_solver()`，並在 finally 呼叫 `flow.close()`；未先啟動時，第一筆 step 只啟動 worker，丟棄啟動前取得的 sample。

父端維持 mapping、量測校正、平滑與發布責任。每次派工都帶新的 request ID、source frame ID，以及父端上一筆有效平滑指令作 seed；`raw` acquisition 物件不傳入 worker。左右結果必須同幀、完整、有限且符合 joint bounds 才返回；求解後仍檢查 150 ms input freshness。丟棄過期結果或追蹤恢復時，下一輪使用父端恢復後的 seed，因此 worker 內前一輪結果不會覆蓋量測／filter 狀態。

| 控制項目 | 正式值／行為 |
| --- | --- |
| NLopt 預算 | 30 ms／側，兩個 process 同時運行 |
| worker ready timeout | 30 秒，發生在輸入／backend 啟動之前 |
| 整批 worker 回覆 timeout | 250 ms；與 30 ms 求解預算、150 ms freshness 不同 |
| 命令與 ROS 發布 | 仍為目標 20 Hz 求解、100 Hz 線性插值發布 |
| filter／interpolation | arm alpha 0.5、hand alpha 0.3、50 ms horizon，維持原設定 |
| worker 失敗／錯誤結果 | 鎖定失敗並停止 session，不切回依序求解，不發布單側結果 |
| duration 到期 | 取消等待並停止 backend；已算完但尚未發布的結果仍丟棄 |
| 正常關閉 | 先停止 backend，再停止／join worker，最後關閉輸入 |
| worker 回收 | 先等 0.5 秒，必要時 terminate，再必要時 kill；每次強制停止後限時 join |
| 父程序突然死亡 | Linux `PR_SET_PDEATHSIG=SIGKILL`，並檢查安裝 signal 前的 parent PID 競態 |

正式 worker 是 daemon，但清理不只依賴 daemon：有 stop／join／terminate／kill，也有 Linux parent-death signal。非 Linux 使用父程序 sentinel 監看 thread，沒有宣稱它能中止所有卡住的 native 呼叫。worker 不持有 ROS／SDK 或感測器連線。

使用者授權後已啟動 **ROS mock** 測試：domain 189、localhost discovery、Sharpa `backend:=mock`、CRX `mock:=true input_rate_hz:=100.0`，分別測 linear 與 Ruckig stream，不開 RViz／viewer，不連硬體。

| ROS mock 模式 | 求解目標數 | 命令 topic 實測頻率 | 結果 |
| --- | --- | --- | --- |
| 純雙手 | 35 | 左右手均約 100.0 Hz，各 173 筆 publication | 通過 |
| CRX linear + 雙手 | 35 | 左右手及雙臂均約 100.0 Hz，各 171 筆 publication | 通過 |
| CRX Ruckig stream + 雙手 | 35 | 左右手及雙臂均約 100.0 Hz，各 171 筆 publication | 通過 |

三個整合情境均通過，檢查 worker 確實為 30 ms、量測與命令維度／跟隨、跨 topic 同 timestamp、pause／resume，以及 ROS feedback thread 和兩個 solver worker 正常關閉。沒有測實體 SDK／馬達延遲。

正式 worker 的 [故障與 lifecycle 測試](../tests/test_parallel_solver.py) 包含單側崩潰／卡住、初始化失敗、錯誤 ID、非法角度、時間到期取消、過期結果的 seed 恢復、輸入啟動失敗、啟動前 sample 丟棄，以及父程序 SIGKILL。Linux parent-death 測試使用 subreaper 回收測試孫程序，避免測試本身留下 zombie；有限幀真實 solver 測試另外確認 worker 先於輸入啟動，結束後正常回收。

完整 headless regression 執行時為 **354 passed、10 skipped、3 failed**。三個失敗均在 `tests/test_crx5ia_leap.py`，對應調查開始前就存在的右側 LEAP URDF `flange_to_leap` 位移修改：`(0.01, -0.03, -0.065)` → `(0.008, -0.04, -0.060)`。未改動或還原這份使用者資產；在 `/tmp` 以 Git 原始 URDF 重驗兩項失敗的 FK 檢查均通過，原始檔 SHA 也符合 manifest。未調整 expected outputs 或降低測試標準。

該次 10 項跳過為 MuJoCo 未安裝的 5 項，以及需要 opt-in ROS 環境的 5 項（CRX-only 3、Sharpa 2）。Sharpa ROS 測試已在 sourced Jazzy 環境另行執行，並再新增、通過 Ruckig mock 情境；本次沒有啟動 CRX-only gateway 或 MuJoCo。後續新增的有限幀 lifecycle 測試也納入最後的 focused regression：process、Sharpa execution／interpolation、bimanual execution 與 package import boundaries 共 **83 passed，無 skip**。Python compileall、文件連結及 `git diff --check` 亦通過。

## 下一步如何定位實機差異

以下為建議，本次沒有執行。

1. **固定比較條件。** 確認純 Sharpa 完整指令，兩次維持相同手部 driver、20/100 Hz、50 ms horizon、手勢及 viewer 設定。記錄 `commands` 每段時間的增量、`solve_ms`、`stale`、`paused`；`commands` 是 solver 目標數，不是 ROS 發布數。
2. **先比較顯示負載。** 在相同控制參數下比較 retargeting `--no-viewer`，以及 CRX `rviz:=false`。如果差異縮小，再調整顯示更新方式；這比直接更改馬達參數更能隔離因素。
3. **同時量測命令與回授。** `ros2 topic hz` 只能看訊息到達頻率，即使 100 Hz 正常，也可能一直發相同／緩慢變化的 target。要比較同一關節的 `joint_command.position` 與 `joint_states.position`，用小幅週期動作的相位差或互相關估計「ROS 命令到量測位置」的跟隨時間。這仍包含 SDK、馬達及 feedback 採樣延遲。
4. **把上游延遲分段。** 若增加量測欄位，記錄 host input receive、左右 solve 起訖、raw/filter target、`backend.execute`、ROS publish、hand callback 開始／SDK 寫入返回及 SDK 讀回時間。現有 command header 是發布時間，無法反推出 Quest frame 的年齡。跨主機須同步時鐘；Browser、host monotonic、ROS clock 不可直接相減。CRX `/crx5ia/interpolated_joint_commands` 診斷訊息使用 monotonic timestamp，也不可直接與 ROS clock header 相減。
5. **依證據調參。** 若主要是上游 filter，先在 preview／mock 單獨提高 hand alpha，觀察抖動，再單獨縮短 horizon；兩個參數目前都不是手指專用 CLI（horizon 會影響全部 joints）。若要只調 hand alpha，可用 `--config` 的 bimanual `output` 覆寫並保留完整 output 欄位，因目前採淺層 update。若 delay 發生在 `joint_command → joint_states`，再檢查 SDK callback 耗時、插值與速度係數。沒有實測前不建議直接關掉 SDK 插值或提高硬體速度。
6. **在雙 process／30 ms 下重新量測。** 現在已整合平行求解；先核對實際 solve time、目標 Hz 與輸出品質，再決定是否降低計算成本。將同一側手指與手臂求解／發布排程分離仍是更進一步的架構變更，涉及腕部目標耦合、校正和一致性，需要另行設計驗證。提高 `--command-hz` 仍不能保證計算能在更短週期內完成。

現場可使用的唯讀檢查例子（需在原本已啟動的 ROS 環境執行）：

```bash
ros2 pkg prefix dual_crx_control
ros2 pkg prefix dual_sharpa_wave
ros2 param get /crx5ia/joint_interpolation method
ros2 param get /crx5ia/joint_interpolation ruckig_target_mode
ros2 param get /sharpa/left_hand/hand_node interpolation
ros2 param get /sharpa/left_hand/hand_node speed_coeff
ros2 topic info /sharpa/left_hand/joint_command --verbose
ros2 topic hz /sharpa/left_hand/joint_command
ros2 topic hz /sharpa/right_hand/joint_command
ros2 topic hz /crx5ia/joint_targets
```

`topic hz` 各自執行、量完停止；必要時對左右 `joint_states` 做同樣量測。若為多機環境，還要核對 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、discovery 設定及時鐘同步。

## 原始碼索引

| 問題 | 主要來源 |
| --- | --- |
| 兩支腳本差異、30 ms 預算、CLI 預設 | [sharpa_teleop.py](../src/retargeting_apps/sharpa_teleop.py)：`build_flow`、`main` |
| 左右求解派工與結果匯合 | [bimanual.py](../src/teleoperation/bimanual.py)：`BimanualRetargetingPipeline.step` |
| 排程、丟棄過期結果、恢復、統計 | [bimanual_execution.py](../src/teleoperation/bimanual_execution.py)：`step`、`run` |
| alpha 濾波公式 | [output.py](../src/teleoperation/output.py)：`QposOutputFilter.apply` |
| 兩份平滑設定 | [crx5ia_sharpa_wave.yaml](../configs/bimanual/crx5ia_sharpa_wave.yaml)、[sharpa_wave.yaml](../configs/bimanual/sharpa_wave.yaml) |
| profile 與目標差異 | [合併左側 profile](../configs/retargeting_profiles/vector_wrist_joint_crx5ia_sharpa_wave_left.yaml)、[純手左側 profile](../configs/retargeting_profiles/vector_wrist_joint_sharpa_wave_left.yaml)；右側有對應檔案 |
| topic、QoS、timer、回授檢查 | [sharpa_joint.py](../src/retargeting_ros/sharpa_joint.py)：`SharpaJointBackend` |
| 插值的起點與終點時間 | [joint_interpolation.py](../src/retargeting_ros/joint_interpolation.py)：`set_target`、`sample` |
| Quest 最新 frame 與 freshness | [receiver.py](../src/teleoperation/inputs/quest3/receiver.py)、[online.py](../src/teleoperation/inputs/quest3/online.py) |
| viewer 在主迴圈的工作 | [bimanual visualizer](../src/retargeting_apps/visualization/execution/bimanual.py)：`update` |
| Sharpa launch／實機設定 | [dual_sharpa.launch.py](../../ws_fanuc/src/dual_sharpa_wave_ros2/launch/dual_sharpa.launch.py)、[hardware YAML](../../ws_fanuc/src/dual_sharpa_wave_ros2/config/dual_sharpa_hardware.yaml) |
| 收到命令即寫 SDK、timer 讀回授 | [hand_node.py](../../ws_fanuc/src/dual_sharpa_wave_ros2/dual_sharpa_wave/hand_node.py)、[sharpa_sdk_hand.py](../../ws_fanuc/src/dual_sharpa_wave_ros2/dual_sharpa_wave/sharpa_sdk_hand.py) |
| CRX launch 實際預設 | [dual_arm.launch.py](../../ws_fanuc/src/dual_crx_control/launch/dual_arm.launch.py)：`generate_launch_description` |
| CRX 插值與 controller 頻率 | [interpolation/node.py](../../ws_fanuc/src/dual_crx_control/src/dual_crx_control/interpolation/node.py)、[controllers YAML](../../ws_fanuc/src/dual_crx_control/config/dual_arm_controllers.yaml) |
| FANUC port／transport | [crx5ia Xacro](../../ws_fanuc/src/fanuc_driver/fanuc_hardware_interface/robot/crx5ia.urdf.xacro)、[stream.cpp](../../ws_fanuc/src/fanuc_driver/fanuc_libs/stream_motion/src/stream.cpp)、[rmi.cpp](../../ws_fanuc/src/fanuc_driver/fanuc_libs/rmi/src/rmi.cpp) |

外部 workspace 連結按目前 `~/retargeting_crx` 與 `~/ws_fanuc` 並列的目錄結構建立。
