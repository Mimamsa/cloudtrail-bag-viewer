[![English](https://img.shields.io/badge/Language-English-lightgrey)](README.md)
[![繁體中文](https://img.shields.io/badge/Language-%E7%B9%81%E9%AB%94%E4%B8%AD%E6%96%87-blue)](README.zh-TW.md)

# CloudTrail bag viewer

不依賴 ROS 的 C++ ROS1 bag 點雲檢視器，支援體素降取樣與點雲累計、里程計軌跡追蹤、反射強度上色，以及 PLY 匯出。

以 C++17 直接解析 **ROS1 bag v2.0** 與 `sensor_msgs/PointCloud2`，使用 Qt5 / OpenGL 顯示點雲，預設累計到目前幀並以 **5 公分體素**降取樣。預設 topic 為 `/cloud_registered`。不需要安裝或啟動 ROS，也不使用 rosbag、roscpp、PCL 或 RViz。

![累計點雲、光達位置與軌跡](docs/odometry_preview.png)

## 安裝環境

### 作業系統與必要元件

目前已在 **Ubuntu 22.04.5 LTS** 上完成編譯與功能驗證。以下使用 Ubuntu／Debian 的 `apt` 套件名稱；其他版本尚未逐一驗證。編譯器需支援 **C++17**，目前建置使用 **Qt5**。

| 套件 | 用途 | 本專案實測元件版本 |
| --- | --- | --- |
| `build-essential` | 提供 C++ 編譯器、Make 等基本編譯工具 | GCC / G++ 11.4.0 |
| `cmake` | 設定專案、產生編譯規則與執行 CTest | CMake 3.22.1 |
| `qtbase5-dev` | Qt5 Widgets、Gui 與相關開發檔案 | Qt 5.15.3 |
| `libgl1-mesa-dev` | OpenGL 標頭與連結用開發函式庫 | 視窗要求 OpenGL 2.1 compatibility context |
| `libbz2-dev` | 解壓縮使用 `bz2` 的 bag chunk | BZip2 1.0.8 |
| `liblz4-dev` | 解壓縮使用 `lz4` 的 bag chunk | LZ4 1.9.3 |

**不需要安裝 ROS、PCL 或 RViz，也不需要執行任何 ROS `setup.bash`。** 即使測試 bag 沒有壓縮，編譯時仍須安裝 BZip2 與 LZ4 開發套件，因為程式包含這兩種解壓功能。

### 安裝必要套件

在終端機執行以下指令；`sudo` 可能要求輸入系統帳號密碼：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  qtbase5-dev \
  libgl1-mesa-dev \
  libbz2-dev \
  liblz4-dev
```

Doxygen 僅用來產生程式碼文件，屬於選用工具，**不是編譯或執行點雲程式的必要套件**。如需文件功能，另行執行 `sudo apt install doxygen`，使用方式見後面的「Doxygen 程式碼文件」。

### 桌面視覺化與無視窗執行

- **3D 視窗、截圖及播放量測**：需要可用的 Linux 桌面顯示環境（X11／Wayland），以及能建立 OpenGL 2.1 compatibility context 的顯示驅動。請從桌面環境的終端機啟動。
- **無視窗檢查與匯出**：`--info`、`--validate`、`--export`、`--export-all`、`--export-all-raw` 可在沒有桌面的伺服器或 SSH 終端機執行，不會建立 OpenGL 視窗。編譯與執行仍需要上述 Qt、OpenGL 等函式庫，因為目前使用同一個可執行檔。
- **輸入資料**：支援已正常建立索引的 ROS1 bag v2.0 `.bag`；不支援 ROS2 `.db3` 或 MCAP。範例 `.bag` 已被 `.gitignore` 排除，取得 GitHub 程式碼後需自行準備資料。可將檔案放在專案根目錄，或在指令中指定其完整路徑。

## 編譯與啟動

### 1. 進入專案根目錄

下列指令請在包含 `CMakeLists.txt` 的資料夾執行。若專案放在 `~/codebases/`，例如：

```bash
cd ~/codebases/cloudtrail-bag-viewer
```

若下載至其他位置，請將路徑換成實際的專案資料夾。

### 2. 設定與編譯

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

第一行檢查依賴並將編譯檔案產生到 `build/`；第二行使用兩個平行工作編譯。成功後會產生 `build/bag_cloud_viewer`。建議使用 `Release`，因為點雲解碼、累計與播放效能會受編譯最佳化影響。

編譯資料夾 `build/` 已被 `.gitignore` 排除，不需上傳至 GitHub。

### 3. 執行回歸測試

```bash
ctest --test-dir build --output-on-failure
```

測試會自行建立小型訊息與 bag fixture，**不需要準備大型 bag，也不需要桌面環境**。成功時會顯示 `100% tests passed`。

### 4. 啟動程式

以下以放在專案根目錄的 `2026-09-15-00-36-37.bag` 為例。從第一幀開始顯示：

```bash
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag
```

直接建立並顯示整包資料的累計地圖：

```bash
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --all
```

在無桌面環境先檢查讀檔是否正常：

```bash
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --info
```

預設點雲 topic 為 `/cloud_registered`，型別須為 `sensor_msgs/PointCloud2`。若名稱不同，可加上 `--topic /your_cloud_topic`；其他匯出與視覺化參數見後面的「命令列用法」。

### 常見安裝問題

| 情況 | 處理方式 |
| --- | --- |
| CMake 找不到 `Qt5Config.cmake` | 安裝 `qtbase5-dev`，再重新執行 CMake 設定指令；目前專案使用 Qt5。 |
| 找不到 OpenGL、BZip2 或 LZ4 | 確認已安裝 `libgl1-mesa-dev`、`libbz2-dev`、`liblz4-dev`，需要的是開發套件。 |
| SSH／伺服器上無法連線顯示器或建立 OpenGL context | 改用上述無視窗參數；需要視覺化時，請在具備可用桌面與顯示驅動的環境執行。 |
| 搬移專案後，CMake 提示快取中的來源／編譯路徑不一致 | 舊 `build/` 含有絕對路徑。使用新的編譯資料夾，例如以 `-B build-new` 設定、`cmake --build build-new -j2` 編譯，之後執行 `./build-new/bag_cloud_viewer`。 |

一般啟動時從第一幀開始；播放時逐步累加。累計範圍始終為第 0 幀到滑桿所選幀，跳回較早幀時會重建，重複選取同一幀不會重複累加。

起始視角與 `Fit view` 都以 map 的 **+Z 向上、+Y 向左、朝 +X 前方看**；相機位於點雲中心的 −X 側。旋轉時維持 Z 軸朝上，俯仰限制在 ±89°，避免翻轉。

視窗操作：

| 操作 | 功能 |
| --- | --- |
| 滑鼠左鍵拖曳 | 旋轉視角 |
| 滑鼠右鍵拖曳 | 平移視角 |
| 滾輪 | 縮放 |
| Play / Pause | 依實際經過時間與所選倍率推進 bag 時間／暫停 |
| Previous / Next | 前／後一幀；累計模式停用 Previous，切回單幀模式恢復 |
| 下方滑桿 | 放開滑桿後跳到指定幀，累計到該幀 |
| LiDAR pose | 顯示目前光達位置（洋紅色）與朝向座標軸 |
| Trajectory | 顯示截至目前幀的 Odometry 軌跡（金色） |
| Accumulate (5 cm) | 切換累計點雲／原始單幀 |
| Load all frames | 一次載入整包資料並顯示累計地圖 |
| Export all frames... | 匯出整包資料，與目前滑桿位置無關 |
| 匯出模式選單 | 5 cm voxel centers：完整體素地圖；All original XYZ + intensity：全部原始有效 XYZ 與強度 |
| Fit view | 讓目前點雲完整入鏡 |
| Fit every frame | 每次換幀重新調整視角 |
| Height color / Single color / Intensity (grayscale) | 依 Z 高度著色／單色／反射強度灰階 |
| 0.25x、0.5x、1x、2x、4x、8x、16x | 播放速度 |

XYZ 軸以紅、綠、藍顯示，位於資料原點。累計點雲預設最多顯示 100 萬個體素中心，使用均勻 reservoir sampling 控制顯示記憶體；`--max-points` 可調整。原始單幀超過上限時採等間距抽樣。播放中暫時最多繪製 20 萬點，暫停後恢復 `--max-points` 指定的上限。**顯示上限不影響完整體素地圖與匯出點數**。

累計採 5 × 5 × 5 公分格網，以每格中心代表該格點雲（不是格內點的平均值），並保留該格第一個點的反射強度。每個 8 × 8 × 8 體素區塊使用 512 個佔用位元，強度另外只為已佔用格緊密配置，不為未佔用格預留 512 個 float。體素地圖仍會隨探索空間增長，並非固定容量快取。

## 播放速度與效能

播放使用單調實際時間計算目標 bag 時間：`目標 = 播放起點 + 實際經過秒數 × 倍率`。不再於每一幀處理結束後額外等待完整幀間隔，避免處理成本累積成速度誤差。速度選單與排程共用同一組倍率。

高倍速時畫面最多每秒更新約 30 次，直接顯示當下已到期的最後一幀；**累計模式仍依序讀入所有中間幀**，只合併畫面更新，不略過地圖資料。每次播放累計最多處理約 50 ms 後讓出事件迴圈，避免長時間落後時整個 UI 持續卡住。播放暫停、拖曳定位後會重新錨定；切換倍率時目標時間保持連續。

狀態列的 `speed 實際倍率 / 選定倍率` 顯示本次播放／切換速度後的平均實際速度，`lag` 是相對目標 bag 時間的落後秒數。若硬體讀檔或累計處理能力不足，實際速度仍可能達不到設定值，此時可直接由倍率與 lag 觀察。

可用同一份 bag 自動播放約 5 秒，印出量測並退出（需要圖形環境）：

```bash
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --playback-rate 16 --benchmark-seconds 5
```

本機以預設顯示設定量測：

| 測試 | 實際倍率 |
| --- | --- |
| 修正前：選 16x、從第零幀開始 | 4.83x |
| 修正後：選 16x、從第零幀開始 | 15.90x |
| 修正後：選 8x、從第零幀開始 | 7.96x |
| 修正後：第 2000 幀起、強度上色、選 16x | 15.90x |

上述為約 5 秒的視窗播放量測，會受系統負載影響；量測列出實際 wall time、推進的 bag 時間、起訖幀、繪圖次數、累計幀數與原始點數。

## 光達位置與軌跡

預設直接解析 `/Odometry` 的 `nav_msgs/Odometry`，不依賴 ROS。洋紅色標記代表目前位置，旁邊標示 `child_frame_id`；局部紅／綠／藍軸代表光達的 X／Y／Z 朝向，紅色箭頭朝 +X。金色線顯示截至目前對應 Odometry 的移動軌跡。這些標記會覆蓋在點雲上，讓密集點雲中的位置仍可辨識。

使用點雲與 Odometry 的 **Header.stamp** 配對，不使用 bag 記錄時間。一般取最接近的樣本，同距離時選較早者；時間差超過 0.2 秒，或 Odometry 的 `header.frame_id` 與點雲座標系不同時，不顯示該幀位置，並在狀態列說明。軌跡也不會跨不同 parent／child frame 連線。不做 TF 轉換或額外外參補償，位置依 Odometry 的 `child_frame_id` 顯示。

播放、滑桿跳轉、單幀及累計模式都會同步更新位置與軌跡；狀態列列出位置 XYZ（公尺）與配對時間差。點雲 PLY 匯出內容為點雲 XYZ 與 intensity，不包含視窗的軌跡與座標軸。

可選參數：

```bash
# 使用其他名稱的 Odometry topic
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --odom-topic /Odometry

# 不讀取位置與軌跡
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --no-odometry
```

缺少 Odometry topic 時仍可查看點雲。Odometry 只保留位置、朝向、時間與座標系，沒有保留 covariance 或 twist；載入時會正規化 quaternion，並拒絕無效位置與旋轉資料。

## 反射強度上色與儲存

上色選單新增 **Intensity (grayscale)**。也可從命令列指定初始模式：

```bash
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --all --color intensity
```

![依反射強度顯示的累計點雲](docs/intensity_preview.png)

- 讀取 PointCloud2 中名為 `intensity` 的純量欄位，支援八種 PointField 數值型別與大小端序；在記憶體及輸出統一轉為 float32。此測試 bag 的欄位為 FLOAT32、offset 32。
- 單幀模式使用該點原始強度；5 公分累計模式保留每格**第一個點**的強度，不取平均或最大值。同一個體素的顯示樣本與完整匯出使用相同代表強度。
- 以目前顯示點的第 1／99 百分位數設定對比範圍（播放中最多每 0.2 秒從至多 65,536 個等距樣本估計；暫停時重新使用完整顯示資料），再以對數灰階顯示：強度低較暗、強度高較亮。超出範圍的顏色會飽和，畫面列出使用範圍；這只影響上色，**不會正規化、裁切或改寫匯出值**。
- 沒有強度欄位或強度為 NaN／Inf 時，仍保留有效 XYZ，強度以 NaN 儲存並顯示為中間灰階。若整份顯示資料皆無有效強度，畫面會提示無強度可用。

PLY 可定義額外的純量屬性，因此不需改用 PCD。所有新匯出的單幀 ASCII、整包原始 binary 與體素 binary PLY 都包含：

```text
property float x
property float y
property float z
property float intensity
```

binary 模式每點為 16 bytes，依序儲存 little-endian 的四個 float32。讀檔軟體需要保留／顯示自訂純量屬性才能看到 `intensity`；若匯入後只有幾何，應先檢查軟體是否忽略額外欄位。格式依據：[美國國會圖書館的 PLY 格式說明](https://www.loc.gov/preservation/digital/formats/fdd/fdd000501.shtml)。

## 命令列用法

幀編號從 **0** 開始。`--info`、`--validate`、`--export`、`--export-all` 和 `--export-all-raw` 不會啟動圖形視窗，可在無桌面的環境執行。

```bash
# 檢查指定幀的大小、時間與座標系
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --info --frame 100

# 完整解碼與驗證該 topic 的所有幀
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --validate

# 匯出單幀為 ASCII PLY，包含有效點的 XYZ 與 intensity
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --frame 100 --export frame100.ply

# 整包累計，降取樣為 5 公分體素，匯出 binary PLY
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --export-all all_clouds_5cm.ply

# 整包原始有效 XYZ 與 intensity，不降取樣，逐幀串流寫入 binary PLY
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --export-all-raw all_clouds_raw.ply

# 保留原本的單幀顯示模式
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --single-frame

# 指定 topic 與顯示點數上限
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --topic /cloud_registered --max-points 200000

# 擷取 3D 畫面後自動退出（需要可用的桌面與 OpenGL）
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --frame 100 --screenshot preview100.png
```

匯出整包資料時，兩種模式都輸出 XYZ 與 intensity，不包含 RGB、逐點時間等其他欄位。原始模式保留跨幀重複點；體素模式每個被佔用的格子輸出一點。新匯出功能先寫入同目錄暫存檔，完成後才更換目標檔案；原始模式支援取消並清除暫存檔。視窗中的載入進度也可取消，保留已完成幀的累計結果。

## 實作方式與支援範圍

- `src/bag.hpp`、`src/bag.cpp`：獨立的 bag 與 PointCloud2 解析器，不依賴 Qt；可重用 `bag::Reader`、`bag::Cloud`、`bag::writePly`。
- `src/playback.hpp`：單調時間播放錨點、連續倍率切換與到期幀查詢。
- `src/intensity.hpp`：顯示用有限強度統計、P1/P99 對比範圍與正規化。
- `src/odometry.hpp`、`src/odometry.cpp`：依訊息時間配對 Odometry，保留位置與朝向軌跡。
- `src/accumulation.hpp`、`src/accumulation.cpp`：分塊體素地圖、累計控制與 binary PLY 串流匯出。
- `src/main.cpp`：Qt5 / OpenGL 視窗與命令列介面。
- 啟動時讀取 bag 的 record headers、connection metadata 與 index，不把整個 2.3 GB 檔案載入記憶體。讀取指定幀時才載入／解壓對應 chunk，快取一個 chunk。
- 支援 `none`、`bz2`、標準 ROS LZ4 frame 壓縮；索引以 bag 記錄時間排序。
- XYZ 依 PointField 名稱、offset 與 datatype 解碼，支援 PointField 的八種數值型別、大小端、`point_step` 與 `row_step` padding；過濾 NaN / Inf。不假設 XYZ 緊密排列或每點只有 12 bytes。
- 累計與整包匯出要求所有訊息的 frame ID 一致；此測試資料為 `map`。不套用 TF，因此不同 frame ID 會回報錯誤。高度色與強度灰階均為顯示用顏色；intensity 會保留，未讀取 RGB。
- 需要已建立索引且正常結束寫入的 ROS1 bag v2.0；不支援 ROS2 `.db3` / MCAP 或損壞 bag 的修復。單一 chunk / record 的配置上限為 512 MiB。
- 播放依 bag 記錄時間及實際經過時間排程，處理延遲時合併畫面更新；累計仍處理全部幀。`stamp` 是訊息內的 Header 時間，可能與 bag 記錄時間不同。

## 驗證結果

測試檔：`2026-09-15-00-36-37.bag`，topic：`/cloud_registered`。

| 項目 | 結果 |
| --- | --- |
| 訊息型別 | `sensor_msgs/PointCloud2` |
| 完整解碼幀數 | 5,317 |
| bag 記錄時間跨度 | 約 531.571 秒 |
| 有效 XYZ 點總數（跨所有幀，未去重） | 50,264,043 |
| 第一幀尺寸 | 7,796 × 1 |
| 第一幀 point_step | 48 bytes |
| 第一幀 intensity | FLOAT32，offset 32，範圍 7～11,472 |
| 第一幀座標系 | `map` |
| Odometry 訊息數 | 5,317 |
| Odometry parent → child | `map` → `sensor_frame` |
| 成功配對的點雲幀數 | 5,317 / 5,317 |
| 最大 Header.stamp 配對時間差 | 0 秒 |
| 5 公分降取樣後完整體素數 | 24,971,805 |
| 體素 PLY 大小 | 399,549,039 bytes（約 381 MiB，含 intensity） |
| 全部原始 XYZ PLY 大小 | 804,224,847 bytes（約 767 MiB，含 intensity） |
| 累計＋體素匯出尖峰 RSS | 約 283 MiB（保留 intensity，預設 100 萬個顯示點） |
| 原始 XYZ 串流匯出尖峰 RSS | 約 18 MiB |

已實際啟動 OpenGL 視窗、載入全部累計點雲並擷取上方預覽。記憶體數值由本機 `/usr/bin/time` 量測命令列匯出，不包含 Qt 視窗與顯示卡的額外開銷。已在目錄下產生 `all_clouds_5cm.ply`。解析測試執行方式：

```bash
ctest --test-dir build --output-on-failure
```

測試涵蓋大小端、不同欄位順序、row padding、NaN 過濾、截斷訊息、bag 索引、跨 chunk 隨機存取，以及 none / bz2 / LZ4 壓縮；另涵蓋正負體素／區塊邊界、累計前後跳轉、取消後續讀、不重複累加、顯示點數上限不影響完整匯出、binary PLY 內容與原始匯出取消時保留既有檔案；強度大小端與各型別解碼、缺值與不合法欄位、體素強度對應及 PLY 儲存、強度顯示範圍；播放絕對時間排程、停頓後追趕、不規則／重複時間、倍率切換及重新錨定；Odometry 解碼、四元數正規化、無效訊息拒絕、跨 chunk 的 Odometry 索引讀取、最近時間配對、時間容差與座標系檢查。

格式參考：[ROS 官方 Odometry 訊息定義](https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg)、[ROS bag v2.0 文件](https://ftp.osuosl.org/.1/ros/ros_wiki_mirror/Bags%282f%29Format%282f%292%282e%290.html)、[ROS 官方 PointCloud2 解析實作](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py)。

## Doxygen 程式碼文件

`src/` 與 `tests/` 中全部 C++ 原始碼與標頭檔皆包含繁體中文與英文對照的 Doxygen 註解，涵蓋：

- 模組、類別、資料欄位、函式參數、回傳值與例外條件。
- ROS1 bag record／chunk 索引、PointCloud2 欄位配置、端序與邊界檢查。
- 體素區塊位元編碼、負座標分格、顯示 reservoir sampling 與完整匯出的差異。
- 檔案快取、非擁有指標與參考的生命週期、取消與暫存檔提交流程。
- Odometry 時間配對、Z-up 相機、OpenGL overlay、播放與 UI 回呼。
- 合成 bag 與訊息 fixture 的格式，以及各回歸測試驗證的行為。

在專案根目錄產生 HTML 文件：

```bash
# 若尚未安裝 Doxygen
sudo apt install doxygen

doxygen Doxyfile
```

文件入口為 `build/doxygen/html/index.html`，包含公開介面、私有解析細節、測試與可瀏覽的原始碼。不需要安裝 Graphviz，文件產生也不需要 bag 檔案或圖形環境。

## License

本專案採用 [MIT License](LICENSE) 授權。
