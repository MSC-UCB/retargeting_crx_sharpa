# 雙手拇指與無名指捏合：停止手勢實作提案

適用於 `scripts/run_crx_sharpa_joint_teleop.py` 的 Quest 雙手輸入，包含 preview 與 ROS backend。

**確定採用的操作：左右手各自將拇指與無名指指尖捏合，兩手同時維持 2 秒。** 不是兩隻手互相接觸，也不是拇指與食指捏合。本方案取代先前的雙手 V 建議。

**狀態：本文是待實作規格。現在的程式尚未提供此功能或下列新 CLI 選項。** 本次只更新文件，未修改控制程式或啟動硬體。

## 1. 使用者操作與停止語意

1. 正常操作時，手留在 Quest 可追蹤的位置。
2. 雙手各自做出拇指與無名指捏合。第一次辨識到完整候選時，立即鎖定停止新目標，不再讓機器人跟隨該影格。
3. 保持捏合 2 秒。程式顯示確認進度，完成後以正常原因退出並清理資源。
4. 候選成立後若放開、缺手或資料逾時，停止鎖定仍有效；以「手勢確認中斷」結束會話，不恢復跟隨。
5. 重新控制需要重新執行程式、取得新 feedback 並重新校準。不自動回 home、不自動張開機器手、不自動重啟 SDK。

**2 秒只確認退出意圖，不是等待 2 秒才停止跟隨。** 這也代表完整候選即使只出現短時間，仍可能中斷會話；代價是偏向誤停而不是意外恢復。因此候選的幾何判斷必須先用一般操作資料驗證誤觸率。

停止新目標、程式退出與機器人實際停穩是不同事件。此手勢是正常結束遙操作的便利功能，不能取代硬體急停。做手勢的早期過渡動作，在尚未辨識成候選前仍可能被模仿；若必須保持實際握持不變，腳踏停止較適合。

## 2. 第一版啟用方式

建議第一版明確選用，保留既有指令行為。以下為**實作完成後才可使用的提案指令**：

```bash
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py \
  --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50 \
  --stop-gesture dual-thumb-ring-pinch --stop-gesture-hold-s 2.0
```

| 擬新增選項 | 提案值／行為 |
| --- | --- |
| `--stop-gesture` | `none` 或 `dual-thumb-ring-pinch`；第一版預設 `none`，明確啟用後才攔截手勢。 |
| `--stop-gesture-hold-s` | 預設 `2.0`，必須有限且大於零；本次選定的使用方式為 2 秒。 |

只在這個 CRX＋Sharpa／Quest 入口先啟用，支援 `--no-viewer`。不默默替其他 flow、hand-only 或 synthetic demo 開啟；選用不支援的輸入組合時應在啟動前報錯。純 preview 不建立 ROS 連線。

第一版 ROS 驗證以使用者目前實測良好的 **CRX `method=ruckig`、`ruckig_target_mode=waypoint`** 為前提。啟用時應透過標準 ROS 參數讀取確認該 node 的實際模式；未知或未驗證模式應在發布前拒絕啟用，不能從 retargeting 的 `linear` 輸出推測下游模式。這是待增加的啟用檢查，不是目前已有的檢查，也不會自動改 launch 參數。

## 3. 單手幾何判斷

沿用 `SensorHandSample.keypoints_wrist`，在 mapping／`human_hand_scale` 之前使用 Quest 解碼後的 21 個腕部局部關鍵點。不使用 optimizer 結果辨識，也不新增模型或依賴。

| 部位 | 索引 |
| --- | --- |
| 拇指指尖 | `4` |
| 食指指根、PIP、DIP、指尖 | `5, 6, 7, 8` |
| 中指指根、PIP、DIP、指尖 | `9, 10, 11, 12` |
| 無名指指根、PIP、DIP、指尖 | `13, 14, 15, 16` |
| 小指指根、PIP、DIP、指尖 | `17, 18, 19, 20` |

令 `p[i]` 為第 i 點，定義：

```text
w      = norm(p[5] - p[17])          # 掌寬基準
r_ring = norm(p[4] - p[16]) / w     # 拇指與無名指距離
r_i    = norm(p[4] - p[i]) / w      # i = 8, 12, 20
```

建議以下**尚待實測校準的初始門檻**，集中放在 detector 的設定中，不散落在 flow／backend：

| 項目 | 初始值 | 用途 |
| --- | ---: | --- |
| `pinch_enter_ratio` | `0.20` | 單手從未捏合進入捏合，要求 `r_ring <= 0.20`。 |
| `pinch_exit_ratio` | `0.30` | 已進入後，`r_ring >= 0.30` 才視為放開；中間區域保留狀態。 |
| `other_tip_margin_ratio` | `0.08` | 要求其他三指的 `r_i >= r_ring + 0.08`，避免把所有指尖擠在一起誤當指定捏合。 |
| `fist_pip_angle_deg` | `110` | 若食指、中指、小指的 PIP 內角全部小於此值，排除為握拳候選。 |
| `max_sample_age_s` | `0.15` | 沿用目前 Quest freshness 範圍。 |
| `max_gap_s` | `0.15` | 兩次有效新影格間隔上限；不得靠舊影格完成確認。 |
| `hold_s` | `2.0` | 兩手連續符合條件的確認時間。 |

例如掌寬 7 cm，進入與離開距離約為 1.4 cm／2.1 cm；這是追蹤骨架的距離門檻，不是保證實際指腹已接觸。確認階段仍須持續滿足防誤觸條件。

PIP 內角以「指根－PIP－DIP」三點計算，伸直時接近 180°。不要求其他手指全部伸直；上述條件只是排除整體握拳及難以區分的抓握。若實際捏合造成相鄰手指遮擋，需根據錄製調整門檻，不能直接取消所有防誤觸條件。

先驗證 `(21, 3)` 形狀、有限座標、非退化掌寬與骨段，再計算距離、角度；角度內積裁切至 `[-1, 1]`。掌寬／骨架尺度可與初始化的有效樣本中位數比對，排除突變；容許比例需由離線資料決定，不把追蹤錯誤當成超高信心捏合。目前資料不提供可直接使用的、已校準的手勢信心分數。

## 4. 新影格與 2 秒的判定

左右手必須來自同一個同步影格，兩手資料都有效且通過上節幾何判斷，才是完整候選。兩隻手可以先後開始捏合；計時從**第一次同時符合**的影格開始。

- 使用 `raw.received_monotonic_ns` 的接收時間，並以本機 monotonic 時鐘檢查 age；不使用 wall clock 或機器人 message stamp 計算手勢持續時間。
- `source_index` 必須增加。重複影格不增加進度，但持續沒有新影格時，仍要透過 `poll(now)` 判定是否逾時。
- 新影格的接收時間必須增加，且相鄰間隔不得超過 `max_gap_s`。
- 確認完成必須由一張新的有效雙手捏合影格觸發：`last_received - first_received >= 2.0 s`。不能只靠 timer 醒來就宣告成功。
- 20 Hz 下通常約有 40 次取樣間隔，但不用「數 40 個迴圈」代替實際時間。
- 候選後出現放開、防誤觸條件失敗、缺手、NaN、影格倒退、過期或長空窗，立即視為確認中斷；仍保留停止鎖定並走退出清理。

## 5. 狀態機與停止優先順序

`BimanualExecutionFlow` 仍是唯一生命週期管理者；手勢模組只管理辨識與確認時間，不建立新的 runtime/session controller。

| 狀態 | 行為 | 下一步 |
| --- | --- | --- |
| `RUNNING` | 每次新影格先檢查手勢，再允許正常 IK。 | 完整候選 → `STOP_LATCHED`。 |
| `STOP_LATCHED` | 阻止新 IK／移動目標，請求 backend 停止，取消 worker 等待；繼續讀 Quest 確認手勢。 | 滿 2 秒 → `EXIT_CONFIRMED`；中斷 → `EXIT_UNCONFIRMED`。 |
| `EXIT_CONFIRMED` | 記錄正常手勢退出；完成清理。 | 結束程序。 |
| `EXIT_UNCONFIRMED` | 記錄確認中斷／逾時；完成清理。 | 結束程序，不回到 `RUNNING`。 |

backend／worker 故障、Ctrl+C 或原本的 duration 到期，不必等待 2 秒，直接走相對應的退出原因。停止流程要冪等，多個原因同時發生時不得重複發送 hold。

不能把手勢鎖定直接等同 `tracking_paused`：現在 tracking pause 在約 0.3 秒的新有效追蹤、且 backend 允許時可能自動重新校準恢復。停止鎖定的優先順序必須高於 tracking recovery，而且鎖定後不再呼叫 `resume_tracking()`。

**主流程順序示意，不是現在已有的 API：**

```text
收到 sample／沒有新資料時定期 poll
    ↓
GestureStopDetector.update(sample, now) 或 poll(now)
    ↓
第一次完整候選？
    flow 鎖定禁止新移動目標
    backend 原子地取消 publication timer、pending target 與舊 generation
    backend 執行經驗證的停止／hold 策略
    pair_solver.cancel()
    該 sample 不進 IK
    ↓
已鎖定？只更新確認狀態；不檢查一般 tracking recovery、不再求解
    ↓
否則才進入既有 step / solver / filter
    ↓
接受 solver 結果前、呼叫 backend.execute 前，再檢查停止鎖定
```

`SharpaJointBackend` 現有的 lock、`_output_generation` 與 timer cancel 可供沿用。停止邊界定義為 backend 在鎖內設好狀態並撤銷 generation；已在這個邊界之前發布的訊息無法收回，後續舊 timer 計算結果必須丟棄。不能只停止 20 Hz 的 IK、卻讓 100 Hz publisher 繼續送舊目標。

## 6. 如何接到目前的程式

| 檔案／模組 | 擬修改內容 |
| --- | --- |
| 新增 `src/teleoperation/stop_gesture.py` | `GestureStopDetector`：純 NumPy 幾何、遲滯、frame freshness、`update()`／`poll()` 與辨識結果；不 import ROS、不呼叫 backend。 |
| `src/teleoperation/bimanual_execution.py` | 可選 detector、獨立的 stop latch／reason、IK 前攔截、confirmation 分支、發布前複查、停止與清理整合。 |
| `src/retargeting_apps/sharpa_teleop.py` | 解析選項、限制支援的輸入組合、注入 detector、顯示進度與退出原因；保持入口腳本薄層。 |
| `src/retargeting_ros/sharpa_joint.py` | 停止狀態與發布 timer 同步；一次性停止策略、下游模式啟用檢查、明確記錄停止請求與失敗。 |
| `src/teleoperation/parallel_solver.py` | 優先沿用 `cancel()`／`close()`；不改雙 process、30 ms 預算。 |
| 新增 focused tests | 幾何、確認時間、停止鎖定、publisher race、worker 結果取消、正常退出與錯誤清理。 |

幾個容易漏掉的整合細節：

- detector 應在 backend 初始化／新校準之前就能處理輸入；若先做出退出手勢，不需為了退出而建立硬體連線。同步的 backend startup 若正在阻塞，第一版仍無法即時處理新影格，須列入延遲量測。
- `run()` 現在會先做 `assert_tracking()`。進入手勢確認分支後，應跳過一般 tracking／恢復邏輯，避免停止後的 backend 狀態把確認流程當作可恢復斷線。
- `pair_solver.cancel()` 會讓等待回覆丟出 `RuntimeError`。已知的使用者停止應辨識為取消，不誤報 solver crash，也不嘗試再次使用已取消的 solver。
- `_duration_expired` 目前只處理定時退出；不把手勢硬塞成 duration 到期。共用底層停止與清理行為，但保留不同原因和手勢確認狀態。
- 結束順序：先封鎖並停止輸出，再清理 worker、Quest session、ROS context／thread 與 viewer；使用 `try/finally` 保證某一元件失敗不跳過其餘清理。候選成立後要保留 Quest 到確認結束，不能立刻關掉輸入。
- 第一版在主流程判斷，因此正在等待 solver 時無法立即辨識新手勢。30 ms 是 optimizer 預算，不是手勢反應上限；要記錄最壞觀測延遲。日後若需要接收端早期偵測，只能送停止事件，不另建一個會直接發布命令的執行緒。

## 7. 下游停止策略與第一版界線

第一版應明確承諾「**鎖定新命令並結束會話**」，不能只從程序退出就宣稱「機器人已安全停穩」。

目前 `SharpaJointBackend.request_stop()`／`close()` 會取消 interpolation，並在 feedback 可用時送一次 measured-position hold。可在 **waypoint 模式**下將此路徑作為第一版待驗證基礎；需要確認 idempotency，且所有鎖內狀態與 timer race 測試通過。它仍是位置目標，實際減速與到達該目標由下游處理，不能承諾瞬間停住。

**不把同一路徑直接宣告支援現有 stream。** 先前錄製的 off-cycle measured hold 曾被差分成高速終端狀態，造成停止後額外動作，詳見 [分析報告](docs/vertical-demo-recording-analysis-20260928.md)。支援 stream 前，必須先完成其停止合約與回歸測試，不能只換成 `KeyboardInterrupt` 或 `sys.exit()`。

Sharpa SDK driver 的 command timeout 目前預設為 0.5 秒，實際以 launch 設定為準；逾時會停止並關閉 SDK session，之後不會因下一筆目標自動恢復。因此這 2 秒確認期間預期可以出現 SDK session 停止，不應保持傳送舊移動目標來躲過 timeout，也不應重新連線。繼續讀取 Quest 完成意圖確認不依賴 SDK session 保持 active。

這裡沒有增加自動鬆手或回位動作；但不能因此保證停止後抓握力不變。若需要「實際停穩」確認，應另外對新鮮 feedback 計算速度並驗證各 driver 狀態，設定獨立逾時；該觀測不能延後停止請求。feedback 失聯時回報無法確認，照樣清理，不顯示停止成功。

未來需要在所有 CRX 插值模式下提供統一受控停止時，可能要修改 dual_crx／Sharpa driver 的停止介面；那是額外的介面工作，不混同於本次手勢幾何判斷。

## 8. 提示、退出結果與測試標準

提示不依賴 viewer 或視窗焦點。主控台必須有清楚原因；有 viewer 時顯示相同狀態與 0–2 秒進度。不要只在電腦畫面提供操作員在 Quest 中看不到的唯一提示；可再接既有 Quest 頁面提示或可聽提示，但不因此新增第一版必需依賴。

建議退出結果：確認手勢退出 `0`、backend／worker 故障 `1`、已停止但手勢確認中斷 `2`；Ctrl+C 保留明確的中斷原因，是否統一成 `130` 另與現有 CLI 慣例整合。清理失敗優先回報錯誤，不能用正常確認蓋過故障。

離線必要案例：

- 左右手捏合、不同掌寬、不同腕部旋轉、指尖距離門檻與遲滯。
- 普通抓握、握拳、拇指食指 pinch、單手無名指 pinch，不觸發完整候選。
- 1.99 秒未確認，2.0 秒後的新有效影格才確認；兩手先後開始以重疊時間計算。
- 重複 frame 不累積，NaN／缺手／過期／倒退／大於 150 ms 空窗中斷確認。
- 候選影格不進 IK；確認中斷後也不自動 resume／重新校準。
- 停止事件與 worker 回覆／100 Hz timer 同時到達時，撤銷邊界後沒有舊移動目標；單次 hold 不重複發送。
- preview、`--no-viewer`、startup 前手勢、duration 到期、Ctrl+C 與停止清理錯誤。

接著用隔離 ROS mock、下游 waypoint 驗證完整 56 軸通道，量測：候選成立、停止邊界、最後移動指令、意圖確認、feedback 停穩與程式退出時間；確認所有 worker／執行緒關閉。mock 通過後才安排獨立硬體驗證，特別量測停止後額外偏移與機器手姿態。

**實作驗收重點：雙手拇指＋無名指持續 2 秒能退出；候選成立後不再跟隨、放開不會恢復，且退出過程不觸發額外未預期動作。** 幾何門檻、下游停止效果與誤觸率都尚待驗證，本文不代表功能已啟用。
