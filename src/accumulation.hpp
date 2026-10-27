/**
 * @file accumulation.hpp
 * @brief 5 公分體素地圖、逐幀累計與 binary PLY 匯出的公開介面。
 *
 * 完整地圖採分塊佔用位元與緊密強度陣列儲存，顯示點採固定上限的均勻 reservoir
 * sample。 完整體素與顯示取樣是兩份用途不同的資料；匯出不能只遍歷顯示點。
 *
 * @par English
 * Public interfaces for 5 cm voxel maps, incremental accumulation, and binary PLY export. The
 * complete map uses blocked occupancy bits and packed intensities; display uses a bounded
 * uniform reservoir sample. The complete voxel map and display sample serve different
 * purposes: export must not iterate only the display sample.
 */
#pragma once
#include "bag.hpp"
#include <array>
#include <functional>
#include <random>
#include <unordered_map>

namespace bag {
// Return false to cancel after the last completed frame.
// 中文：回傳 false 會在最近完成的幀之後取消。
/**
 * @brief 累計或原始匯出完成一幀後的進度／取消回呼。
 *
 * completed 是已完成幀數，total 是本次目標幀數，兩者均不是百分比。
 * 回傳 true 繼續；false 在最近完成的幀邊界取消。回呼於呼叫執行緒同步執行。
 *
 * @par English
 * Progress and cancellation callback after a frame is accumulated or exported. completed is
 * the number of completed frames and total is the target frame count, not a percentage. Return
 * true to continue or false to cancel at a completed-frame boundary. The callback runs
 * synchronously on the caller's thread.
 */
using Progress = std::function<bool(size_t completed, size_t total)>;
/**
 * @class bag::VoxelGrid
 * @brief 以 8×8×8 體素區塊壓縮儲存的稀疏佔用地圖。
 *
 * 每格記錄佔用狀態與第一筆點的 intensity，以格子中心代表 XYZ，不計算重心。
 * 區塊需要 512
 * bits、已佔用格的緊密強度陣列及雜湊中繼資料；記憶體隨探索區域增長。
 * 所有輸入必須位於相同 frame；此類別不做 TF 轉換。
 *
 * @par English
 * A sparse occupancy map using 8-by-8-by-8 voxel blocks. Each cell stores occupancy and the
 * first input point's intensity; its center represents XYZ rather than a centroid. A block
 * needs 512 occupancy bits, packed intensities for occupied cells, and hash metadata. Memory
 * grows with explored space. All inputs must use one frame; no TF conversion is performed.
 */
/**
 * @var uint64_t bag::VoxelGrid::inputCount
 * @brief 包含同格及跨幀重複點的原始 XYZ 累計數。
 *
 * @par English
 * Total original XYZ points added, including duplicates within cells and across frames.
 */
class VoxelGrid {
public:
  /**
   * @brief 建立空白體素地圖與固定種子的顯示取樣器。
   *
   * @param size 體素邊長，單位沿用 XYZ；公尺資料的 0.05 即 5 公分。
   * @param sampleLimit 顯示點數上限；0 代表不保留顯示點，但完整地圖仍建立。
   * @throws std::runtime_error 邊長不是有限正數。
   *
   * @par English
   * Create an empty voxel map with a deterministically seeded display sampler. Parameters: size
   * is the cell width in the same units as XYZ, so 0.05 means 5 cm for meter coordinates;
   * sampleLimit bounds display points, with zero disabling the display sample while retaining
   * the full map. Throws std::runtime_error unless size is finite and positive.
   */
  explicit VoxelGrid(double size = 0.05, size_t sampleLimit = 1000000);
  /**
   * @brief 將一幀 XYZ 點加入佔用格網。
   *
   * 每格索引為 floor(coordinate /
   * voxelSize)，因此負座標也有一致的半開格子邊界。 已佔用的格子不重複計入
   * size()，但原始點仍計入 inputPoints()。 先驗證 frame
   * 與全部座標，再變更佔用資料；不保留原始點。
   *
   * @param cloud 同一座標系、所有 XYZ 皆為有限值的點雲。
   * @throws std::runtime_error 座標系不同或格子索引超出 int32_t 範圍。
   *
   * @par English
   * Add one frame of XYZ points to the occupancy grid. Each index is floor(coordinate /
   * voxelSize), giving consistent half-open cells for negative coordinates. An occupied cell is
   * counted once by size(), while all input points count toward inputPoints(). Validate the
   * frame and all coordinates before modifying occupancy; original points are not retained.
   * Parameter cloud must share the map frame and have finite XYZ. Throws std::runtime_error for
   * mismatched frames or voxel indices outside int32_t.
   */
  void add(const Cloud &cloud);
  /**
   * @brief 清空佔用資訊、統計與 frame，並重設取樣亂數種子。
   *
   * 容器可能保留配置容量供下一次重建使用，不保證 RSS 立即降為零。
   *
   * @par English
   * Clear occupancy, statistics, and the frame, and reset the sampler's random seed. Containers
   * may retain allocated capacity for a later rebuild; RSS is not guaranteed to drop immediately
   * to zero.
   */
  void clear();
  // Bounded uniform display sample; use forEachPoint() to enumerate the full
  // map.
  // 中文：顯示點是有界的均勻取樣；請使用 forEachPoint() 遍歷完整 地圖。
  /**
   * @brief 取得有限數量的均勻顯示取樣。
   *
   * @return 內部取樣陣列的唯讀參考；不是完整體素集合。
   * @warning add() 或 clear() 可能使元素指標、迭代器失效。
   *
   * @par English
   * Get the bounded uniform display sample. Returns a const reference to the internal sample,
   * not all occupied voxels. Warning: add() and clear() may invalidate element pointers and
   * iterators.
   */
  const std::vector<Point> &points() const { return samples; }
  /**
   * @brief 取得完整地圖中已佔用的體素數。
   *
   * @return 完整體素數，不受顯示上限影響。
   *
   * @par English
   * Get the number of occupied voxels in the complete map. The returned count is independent of
   * the display limit.
   */
  uint64_t size() const { return voxelCount; }
  /**
   * @brief 逐格重建完整地圖的體素中心並呼叫 visitor。
   *
   * 遍歷所有佔用格，完全不受 sampleLimit 影響；雜湊遍歷次序不具空間排序意義。
   * 不一次建立完整點陣列，適合分批匯出。
   *
   * @param visitor 每個體素呼叫一次的同步回呼；不可在回呼中修改此地圖。
   *
   * @par English
   * Reconstruct each occupied voxel center and call visitor. Iterate the complete map regardless
   * of sampleLimit; hash iteration is not spatially sorted. No complete point array is
   * materialized, making this suitable for batched export. Parameter visitor is a synchronous
   * callback called once per cell and must not modify the map.
   */
  void forEachPoint(const std::function<void(Point)> &visitor) const;
  /**
   * @brief 取得首次加入點雲的座標系名稱。
   *
   * @return frame ID；尚未加入點雲或剛 clear() 時為空字串。
   *
   * @par English
   * Get the coordinate-frame name established by the first input cloud. Returns an empty string
   * before any input or after clear().
   */
  const std::string &frame() const { return frameId; }
  /**
   * @brief 取得累計加入的原始點數，包含跨幀或同格的重複點。
   *
   * @return 所有成功加入 Cloud::points.size() 的總和。
   *
   * @par English
   * Get the total number of original input points, including repeated points within a voxel or
   * across frames. Returns the sum of Cloud::points.size() for successfully added clouds.
   */
  uint64_t inputPoints() const { return inputCount; }
  /**
   * @brief 取得體素邊長。
   *
   * @return 建構時指定的邊長。
   *
   * @par English
   * Get the voxel width supplied to the constructor.
   */
  double voxelSize() const { return leafSize; }

private:
  /**
   * @struct bag::VoxelGrid::Key
   * @brief 三個軸向的區塊索引；每個區塊容納八格。
   *
   * @par English
   * Three signed block indices, with eight voxels per block along each axis.
   */
  /**
   * @var int32_t bag::VoxelGrid::Key::x
   * @brief X 軸區塊索引；一個區塊為八格。
   *
   * @par English
   * X block index; each block spans eight cells.
   */
  /**
   * @var int32_t bag::VoxelGrid::Key::y
   * @brief Y 軸區塊索引；一個區塊為八格。
   *
   * @par English
   * Y block index; each block spans eight cells.
   */
  struct Key {
    int32_t x, y, z; ///< 區塊座標；不是點座標或格內索引。
    ///< EN: Block coordinates, not point coordinates or indices within a block.
    /**
     * @brief 比較兩個區塊的三軸索引。
     *
     * @param b 另一個區塊 key。
     * @return 三軸皆相同才為 true。
     *
     * @par English
     * Compare all three block coordinates. Parameter b is the other key. Returns true only if all
     * coordinates match.
     */
    bool operator==(const Key &b) const {
      return x == b.x && y == b.y && z == b.z;
    }
  };
  /**
   * @brief 將區塊索引混合成 unordered_map 使用的雜湊值。
   *
   * @par English
   * Hash block coordinates for use in unordered_map.
   */
  struct Hash {
    /**
     * @brief 計算三軸混合雜湊，分散正負座標及相鄰區塊。
     *
     * @param k 區塊索引。
     * @return 平台 size_t 寬度的雜湊值。
     *
     * @par English
     * Mix all three coordinates to distribute adjacent and negative blocks. Parameter k is the
     * block key. Returns a hash value with the platform's size_t width.
     */
    size_t operator()(const Key &k) const;
  };
  double leafSize;                         ///< 一個體素的邊長。
  ///< EN: Width of one voxel.
  size_t sampleLimit;                      ///< 顯示取樣點數上限。
  ///< EN: Maximum number of display sample points.
  uint64_t inputCount = 0, voxelCount = 0; ///< 累計原始點數與不同佔用體素數。
  ///< EN: Total input points and distinct occupied voxels.
  bool hasFrame = false; ///< 是否已由首次加入的點雲設定 frame。
  ///< EN: Whether an input cloud has established the frame.
  std::string frameId;   ///< 整份累計地圖共用的座標系。
  ///< EN: Shared coordinate frame of the accumulated map.
  // 8x8x8 voxels per block: 512 occupancy bits instead of a hash node per
  // point.
  // 中文：每個區塊含 8x8x8 格：使用 512 個佔用位元，避免為每個 體素配置一個雜湊節點。
  /**
   * @brief 區塊的佔用位元與依 local index 排序的緊密強度陣列。
   *
   * intensities 不為未佔用格配置元素，避免稀疏區塊配置完整 512 個 float。
   * 尚未遇到有效強度時陣列可為空；建立陣列時補齊既有格子的 NaN。
   *
   * @par English
   * A block's occupancy bits and compact intensities ordered by local index. Do not allocate
   * intensity elements for unoccupied cells or a full 512-float array for sparse blocks. The
   * vector may remain empty until a finite intensity arrives; initialize earlier occupied cells
   * to NaN when allocating it.
   */
  struct Block {
    std::array<uint64_t, 8> bits{}; ///< 512 格的佔用狀態。
    ///< EN: Occupancy bits for 512 cells.
    std::vector<float>
        intensities; ///< 已佔用格的第一筆 intensity，或缺值 NaN。
        ///< EN: First intensity for each occupied cell, or NaN for a missing value.
  };
  std::unordered_map<Key, Block, Hash>
      blocks; ///< 區塊到 8 個 64-bit 佔用字組的對照表。
      ///< EN: Block lookup containing occupancy words and packed intensities.
  std::vector<Point> samples; ///< 最多 sampleLimit 個體素中心的顯示樣本。
  ///< EN: Display sample containing at most sampleLimit voxel centers.
  std::mt19937_64 random{
      42}; ///< 固定種子使相同輸入順序的 reservoir sampling 可重現。
      ///< EN: A fixed seed makes reservoir sampling reproducible for identical input order.
};
/**
 * @brief 維護從第零幀累計到指定幀的狀態。
 *
 * 已完成幀範圍始終為 [0, loadedFrames())。往前跳會補讀所有中間幀，
 * 往後退則清空重建，避免重播同一幀造成累計統計錯誤。
 *
 * @par English
 * Maintain accumulation from frame zero through a selected frame. Completed frames always form
 * the range [0, loadedFrames()). Forward seeks read every intervening frame; backward seeks
 * clear and rebuild the map to avoid double accumulation and incorrect statistics.
 */
class Accumulation {
public:
  /**
   * @brief 繫結讀取器並建立空體素地圖。
   *
   * @param reader 必須比此物件活得更久的 bag 讀取器；不移轉所有權。
   * @param size 體素邊長，預設 0.05 公尺。
   * @param sampleLimit 顯示點數上限，與完整匯出點數無關。
   *
   * @par English
   * Bind a reader and create an empty voxel map. Parameters: reader is a non-owning reference
   * and must outlive this object; size is the voxel width, defaulting to 0.05 meters;
   * sampleLimit caps display points, not complete export.
   */
  explicit Accumulation(Reader &reader, double size = 0.05,
                        size_t sampleLimit = 1000000)
      : grid(size, sampleLimit), reader(reader) {}
  // Contains exactly frames [0, loadedFrames()). Seeking backwards rebuilds.
  // 中文：內容恰好涵蓋 [0, loadedFrames()) 的幀；向後定位會重建。
  /**
   * @brief 將累計結果推進或重建到 target（包含該幀）。
   *
   * 取消檢查在一整幀成功加入後進行；取消後仍可由 loadedFrames()
   * 確認已完成位置， 再次 seek() 可繼續。同一 target 重複呼叫不會重複加點。
   *
   * @param target Reader::frames 的零起始目標幀。
   * @param progress 可省略的進度與取消回呼。
   * @return true 表示完成；false 表示回呼要求停止。
   * @throws std::runtime_error 目標超界、讀檔失敗或點雲不適合合併。
   *
   * @par English
   * Advance or rebuild accumulation through target, inclusive. Cancellation is checked only
   * after a complete frame is added. After cancellation, loadedFrames() identifies the last
   * completed position; seek() can resume. Repeating the same target does not add it twice.
   * Parameters: target is a zero-based Reader::frames index; progress is an optional callback.
   * Returns true on completion or false when canceled. Throws std::runtime_error for an invalid
   * index, read failure, or incompatible cloud.
   */
  bool seek(size_t target, const Progress &progress = {});
  /**
   * @brief 取得已完整加入地圖的幀數。
   *
   * @return 最後完成的幀索引加一；空地圖狀態為零。
   *
   * @par English
   * Get the number of fully accumulated frames. Returns the last completed frame index plus one,
   * or zero when empty.
   */
  size_t loadedFrames() const { return loaded; }
  VoxelGrid grid; ///< 目前完成幀範圍的累計地圖與顯示樣本。
  ///< EN: Map and display sample covering the completed frame range.

private:
  Reader &reader; ///< 非擁有參考；與其他操作共用讀檔快取。
  ///< EN: Non-owning reader reference sharing its chunk cache with other operations.
  size_t loaded = 0; ///< 已完成幀數，也就是下一個待加入的幀索引。
  ///< EN: Completed-frame count, also the next frame index to accumulate.
};
// Binary little-endian XYZ PLY. Writes to a temporary file, then renames on
// success.
// 中文：Little-endian binary PLY，包含 XYZ 與 intensity；先寫暫存檔，完成後才 重新命名提交。
/**
 * @brief 以固定大小批次匯出完整體素中心與第一筆強度為 binary PLY。
 *
 * 每批至多 4096 點；不經過顯示取樣，且使用暫存檔成功提交後才替換目標。
 *
 * @param path 目標檔案路徑，父目錄須存在。
 * @param grid 要匯出的完整佔用地圖。
 * @throws std::runtime_error 無法開啟或寫入檔案。
 * @throws std::filesystem::filesystem_error 最終重新命名失敗。
 *
 * @par English
 * Export all voxel centers and their first intensities as binary PLY in bounded batches. Each
 * batch holds at most 4096 points. Bypass the display sample and replace the destination only
 * after a successful temporary-file commit. Parameters: path is the destination and its parent
 * must exist; grid is the complete map. Throws std::runtime_error for file errors or
 * std::filesystem::filesystem_error if final rename fails.
 */
void writeVoxelPly(const std::string &path, const VoxelGrid &grid);
/**
 * @brief 將指定 XYZ 與 intensity 輸出成 little-endian binary PLY。
 *
 * @param path 目標 PLY 路徑。
 * @param points 全部要輸出的 XYZ，呼叫端應確保資料有效。
 * @throws std::runtime_error 建檔、寫入或關檔失敗。
 * @throws std::filesystem::filesystem_error 暫存檔無法重新命名。
 *
 * @par English
 * Export the given XYZ and intensity array as little-endian binary PLY. Parameters: path is
 * the destination; points contains every point to write and should contain valid data. Throws
 * std::runtime_error on create, write, or close failure, or std::filesystem::filesystem_error
 * on rename failure.
 */
void writeBinaryPly(const std::string &path, const std::vector<Point> &points);
// Streams all valid original XYZ points, retaining only one decoded
// frame/chunk.
// 中文：串流輸出所有原始有效 XYZ 與強度，每次僅保留一個解碼 幀與一個 chunk。
/**
 * @brief 串流匯出整包所有原始有效 XYZ 與 intensity，不做體素降取樣。
 *
 * 每次只保留一幀解碼結果並共用讀取器的單一 chunk 快取。跨幀重複點照常寫出。
 * 所有訊息 frame 必須一致；取消或例外時刪除暫存檔，既有目標不會被提交替換。
 *
 * @param reader 提供全部點雲幀的讀取器，不移轉所有權。
 * @param path binary PLY 目標路徑。
 * @param progress 每完成一幀呼叫的可選進度／取消回呼。
 * @return true 表示已提交完整檔案；false 表示取消。
 * @throws std::runtime_error 讀寫失敗或 frame 不一致。
 * @throws std::filesystem::filesystem_error 最終重新命名失敗。
 *
 * @par English
 * Stream all original valid XYZ points and intensities without voxel downsampling. Retain one
 * decoded frame and the reader's single cached chunk at a time; preserve duplicates across
 * frames. All frames must match. Cancellation and exceptions delete the temporary file without
 * committing a replacement over an existing destination. Parameters: reader provides all cloud
 * frames without transferring ownership; path is the binary PLY destination; progress is
 * optional. Returns true after commit or false on cancellation. Throws std::runtime_error for
 * read/write errors or frame mismatch, or std::filesystem::filesystem_error if rename fails.
 */
bool exportRawPly(Reader &reader, const std::string &path,
                  const Progress &progress = {});
} // namespace bag
// 中文：bag 命名空間。
