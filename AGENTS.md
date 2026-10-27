# CloudTrail: agent development guidelines / 開發指引

This file applies to the repository root and all subdirectories. Follow more specific nested `AGENTS.md` files when present. Explicit user instructions take precedence over these project conventions.

本文件適用於專案根目錄與所有子目錄。若子目錄有更具體的 `AGENTS.md`，請一併遵循；使用者明確要求優先於本文件的專案慣例。

## Project / 專案定位

CloudTrail is a ROS-free C++17 viewer for ROS1 bag v2.0 point clouds. The executable is `bag_cloud_viewer`; the repository name is `cloudtrail-bag-viewer`.

CloudTrail 是不依賴 ROS 的 C++17 點雲檢視器。核心能力包含 PointCloud2 解碼、5 公分體素累計、Odometry 軌跡、反射強度上色、PLY 匯出與倍速播放。

- Keep parsing, accumulation, and timestamp matching independent of ROS, PCL, RViz, and Qt. Qt5/OpenGL belong in the viewer layer.
  解析、累計與時間配對保持獨立；Qt5/OpenGL 用於視窗層。
- Preserve the current behaviors below unless the requested task intentionally changes them. Update the implementation, meaningful tests, and both READMEs together when behavior changes.
  以下為目前行為；若任務要求變更，請同步調整實作、必要測試與兩版 README。

## Code map / 程式架構

| Path | Responsibility / 職責 |
| --- | --- |
| `src/bag.hpp`, `src/bag.cpp` | Bag indices, chunk decompression, PointCloud2/Odometry decoding, ASCII PLY / 索引、解壓、訊息解析與單幀匯出 |
| `src/accumulation.hpp`, `src/accumulation.cpp` | Blocked voxel storage, display sampling, accumulation and binary PLY / 分塊體素、取樣、累計與串流匯出 |
| `src/odometry.hpp`, `src/odometry.cpp` | Pose tracks and nearest-message-time matching / 位姿軌跡與最近時間配對 |
| `src/playback.hpp` | Monotonic playback timeline and due-frame lookup / 單調播放時間軸與到期幀查詢 |
| `src/intensity.hpp` | Display-only intensity statistics and normalization / 顯示用強度統計 |
| `src/main.cpp` | CLI, Qt widgets, OpenGL rendering and UI callbacks / 命令列、視窗、繪圖與互動 |
| `tests/reader_tests.cpp` | Headless synthetic fixtures and regression tests / 無視窗合成資料與回歸測試 |
| `README.md`, `README.zh-TW.md` | English and Traditional Chinese user documentation / 英文與繁體中文使用說明 |
| `Doxyfile` | Bilingual source documentation configuration / 雙語程式碼文件設定 |

## Build and validation / 編譯與驗證

Run from the directory containing `CMakeLists.txt`. Dependencies and the verified Ubuntu environment are documented in both READMEs. Required libraries are Qt5 Widgets/Gui, OpenGL, BZip2, and LZ4.

請在含有 `CMakeLists.txt` 的目錄執行。依賴套件與已驗證環境以兩版 README 為準。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

- Build after C++ changes. For functional changes, run the relevant existing tests; extend regression coverage for parsing, storage, synchronization, or scheduling changes. Do not add tests that merely repeat trivial implementation details or require a large private bag.
  修改 C++ 後確認編譯；功能變更執行相關既有測試。解析、儲存、同步或排程應有有意義的回歸驗證；小型文字或文件變更不需新增測試。
- Existing tests generate small bag/message fixtures and need neither ROS, a GUI, nor the sample bag. Keep them effective in Release builds; use the existing `check()` pattern rather than disabled `assert` checks.
  測試自行建立 fixture，且應在 Release 模式仍能檢查失敗條件。
- Use a fresh build directory after moving the repository if CMake reports cached absolute paths. Do not edit generated makefiles or cache entries to work around relocation.
  搬移專案造成快取路徑不符時重新設定編譯目錄，不手改產生的檔案。
- For comment/documentation changes, check links and run `doxygen Doxyfile` when source documentation or its configuration changes. Resolve documentation warnings.
  註解與文件變更應確認連結；涉及 Doxygen 時確認文件產生且無警告。

## Data and timing contracts / 資料與時間約定

### Bag decoding / Bag 解析

- Support indexed ROS1 bag v2.0 with `none`, `bz2`, and standard ROS LZ4 frame compression. Do not claim ROS2 `.db3`, MCAP, or damaged-bag recovery without implementing and verifying it.
  支援範圍須與實作、測試一致，不將未支援格式標為可用。
- Decode fields using names, datatypes, offsets, byte order, `point_step`, and `row_step`; never assume tightly packed XYZ. Preserve bounds checks, exact-read checks, and the current 512 MiB per-record/chunk allocation guard.
  保留欄位配置、端序、padding 與資料長度檢查，勿直接假設固定點格式。
- Filter nonfinite XYZ. Missing or invalid intensity is NaN and must not remove an otherwise valid point. Intensity is stored as float32.
  XYZ 非有限值會被移除；強度缺值以 NaN 表示，保留有效 XYZ。
- Retain lazy chunk loading and a single decompressed-chunk cache. `Reader` is not thread-safe: its stream position and cache are shared. Account for ownership and lifetimes before adding asynchronous work.
  不一次載入整包 bag；非同步設計需處理讀取器共享狀態與生命週期。

### Accumulation and export / 累計與匯出

- The default voxel edge is 0.05 m. Use floor-based indices for negative coordinates. Represent each occupied cell by its center and the first received point's intensity, not an average or maximum.
  5 公分格網的負座標分格、中心點與第一筆強度語意須一致。
- The full voxel map and bounded display reservoir are different data sets. Display limits must never limit complete exports. Preserve compact occupied-cell intensity storage rather than allocating a full float array for every sparse block.
  顯示取樣不等於完整地圖；避免為每個稀疏區塊配置完整強度陣列。
- Accumulation covers every frame from zero through the selected frame. Repeated seeks must not double-count; backward seeks rebuild. Cancellation and time-budget yields retain only completed-frame progress.
  累計範圍、重複定位、後退重建與取消後續讀應維持正確。
- Export XYZ plus `intensity`. Binary PLY uses four little-endian float32 values per vertex, totaling 16 bytes. Display normalization must never change exported intensity. Original export keeps cross-frame duplicates; voxel export writes one representative per occupied cell.
  原始匯出保留重複點；體素匯出逐格輸出，兩者均保留強度欄位。
- Whole-bag binary exports stream through a temporary file beside the destination and commit only on completion. Cancellation/failure must preserve an existing destination. Never overwrite the input bag with an export.
  串流匯出、暫存檔提交與來源檔保護不可因功能修改而失效。

### Odometry and playback / 里程計與播放

- Defaults are `/cloud_registered` (`sensor_msgs/PointCloud2`) and `/Odometry` (`nav_msgs/Odometry`). Pose matching uses `Header.stamp`; playback uses bag recording time. Never interchange the two.
  位姿配對的訊息時間與播放的 bag 記錄時間須明確區分。
- Match the nearest pose with an earlier-sample tie break, the same parent frame, and a 0.2 s tolerance. Normalize valid quaternions. Do not silently apply TF or sensor extrinsics or connect trajectories across different parent/child frames.
  保留時間容差、座標系檢查與四元數驗證；外參或 TF 須明確實作。
- Playback must follow an absolute monotonic clock, not sleep a complete frame interval after processing. Keep rate changes continuous and re-anchor after pause/resume or manual seeks.
  不將處理成本持續加進逐幀等待，避免倍速名義提高但實際速度不變。
- Coalesce late display updates while still accumulating every intermediate frame. Current playback limits are approximately 30 display updates/s, 200,000 drawn points, and 50 ms accumulation batches; pausing restores the configured display limit.
  高倍速可合併畫面，但不略過地圖資料，並維持 UI 可回應。
- The supported rates are 0.25x, 0.5x, 1x, 2x, 4x, 8x, and 16x. Keep selector labels and timing multipliers sourced from the same definitions. Report actual rate and lag rather than promising a rate unsupported by measurements.
  倍率選單與排程使用同一份定義，效能結論須有實測依據。

## Viewer behavior / 視窗行為

- Preserve the initial/Fit view orientation: map +Z up, +Y left, looking along +X. Clamp pitch to avoid flipping.
  起始視角與重設視角皆保持既定 map 軸向。
- Disable Previous in accumulated mode and enable it in single-frame mode, including programmatic switches such as loading/exporting the whole map. Keep mode-control synchronization centralized.
  不只處理 checkbox 的手動切換，也要涵蓋封鎖 signals 的程式內部切換。
- Keep height, single-color, and intensity coloring available. Intensity contrast uses display-only P1/P99 statistics and logarithmic grayscale; skip unnecessary statistics in other color modes. Preserve playback sampling/throttling.
  強度上色不改資料；高度上色不應額外計算整份地圖的強度百分位數。
- GUI visualization, screenshots, and benchmarks require a working desktop/OpenGL context. Headless inspection/export must finish before creating `QApplication`.
  保持無視窗功能可在沒有桌面的環境執行。

## Documentation and style / 文件與風格

- Write C++17 consistent with nearby code. Prefer explicit ownership, RAII, checked binary reads, and useful error messages. Keep Qt out of the reader library. Format only touched code when needed; avoid unrelated repository-wide reformatting.
  修改範圍集中於任務，沿用既有縮排與架構。
- Maintain Traditional Chinese and English source comments. Preserve Chinese explanations and add English counterparts. Existing Doxygen blocks use `@par English`; member comments use `EN:`. Document units, timestamps, ownership, cancellation, and exceptions where relevant. Keep Doxygen tags such as `@param` and `@return` unique rather than duplicating them for translation.
  原始碼註解維持中英對照；翻譯應包含契約與演算法，不只函式名稱。
- Keep `README.md` in English and `README.zh-TW.md` in Traditional Chinese. Synchronize features, CLI examples, dependencies, and measured results. Preserve reciprocal language badges and valid relative links. Keep both pages in Doxygen input.
  使用者可從兩份 README 開頭的 badge 相互切換。
- Preserve the MIT `LICENSE` and its copyright notice.
  保留 MIT 授權文件與版權標示。

## Local data and verification / 本機資料與驗證

- Never commit bag recordings (`*.bag`, `*.bag.active`), generated PLY files, build directories/backups, or generated Doxygen HTML. Preserve the existing `.gitignore` rules.
  大型資料、匯出結果與編譯產物不加入 Git。
- The local sample bag may be absent in a fresh clone. Use synthetic fixtures for regression tests; use the real bag only when available and useful for integration/performance validation.
  不讓一般測試依賴未隨 repository 提交的資料。
- For a relevant GUI performance change, measure elapsed wall time and bag time advanced with the existing benchmark:
  修改播放效能時可使用既有量測功能，並確認所有中間幀仍被累計。

```bash
./build/bag_cloud_viewer 2026-09-15-00-36-37.bag --playback-rate 16 --benchmark-seconds 5
```

Report what changed, what was actually checked, and any remaining limitation. Do not report unrun tests or unmeasured performance as verified.

完成後說明變更、實際執行的驗證及限制；未執行的檢查不可宣稱通過。
