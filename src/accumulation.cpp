/**
 * @file accumulation.cpp
 * @brief 分塊體素、顯示取樣與具暫存檔提交的 PLY 寫入實作。
 *
 * 體素地圖儲存佔用位元與每格第一筆強度，不為每個原始點建立物件；匯出以批次緩衝控制暫存記憶體。
 * 暫存檔以 POSIX mkstemp 建立於目的檔同一目錄，完成前不替換原目的檔。
 *
 * @par English
 * Blocked voxels, display sampling, and PLY writing with temporary-file commits. The map
 * stores occupancy bits and the first intensity in each voxel instead of retaining every input
 * point. Export uses bounded batches. POSIX mkstemp creates a temporary file beside the
 * destination, which is replaced only after successful completion.
 */
#include "accumulation.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace bag {
namespace {
/**
 * @brief 將實數座標轉成有號體素索引。
 *
 * 使用 floor 而非向零截斷，確保負座標邊界與正座標一致。
 *
 * @param value 一個軸向的座標。
 * @param size 有限正體素邊長。
 * @return floor(value / size)。
 * @throws std::runtime_error 商非有限或超過 int32_t。
 *
 * @par English
 * Convert a real coordinate to a signed voxel index. Use floor rather than truncation toward
 * zero so negative coordinates follow the same cell boundaries. Parameters: value is one
 * coordinate; size is a finite positive cell width. Returns floor(value / size). Throws
 * std::runtime_error for a nonfinite quotient or an index outside int32_t.
 */
int32_t coordinate(float value, double size) {
  double v = std::floor(double(value) / size);
  if (!std::isfinite(v) || v < std::numeric_limits<int32_t>::min() ||
      v > std::numeric_limits<int32_t>::max())
    throw std::runtime_error("Coordinate outside supported voxel index range");
  return int32_t(v);
}
/**
 * @brief 以兩次乘法與位移混合 64-bit 輸入。
 *
 * @param v 待分散的位元表示。
 * @return 混合後的雜湊值，不用於密碼學。
 *
 * @par English
 * Mix a 64-bit value using two multiplications and shifts. Parameter v is the input bit
 * pattern. Returns a mixed hash value; this is not a cryptographic hash.
 */
uint64_t mix(uint64_t v) {
  v = (v ^ (v >> 30)) * 0xbf58476d1ce4e5b9ULL;
  v = (v ^ (v >> 27)) * 0x94d049bb133111ebULL;
  return v ^ (v >> 31);
}
/**
 * @brief 以 RAII 清理暫存檔的 little-endian binary XYZI PLY 寫入器。
 *
 * 每個頂點依序包含四個 float：x、y、z、intensity。
 * 先預留 20 字元的 vertex count 欄位，所有點寫完後回填總數，避免事先掃描資料。
 * 只有 finish() 成功才提交檔案；未完成時解構函式移除暫存檔。
 *
 * @par English
 * An RAII writer for little-endian binary XYZI PLY files. Each vertex contains four floats in
 * order: x, y, z, intensity. Reserve a 20-character vertex-count field and fill it after
 * writing all points, avoiding a preliminary scan. Only finish() commits the destination;
 * destruction removes an uncommitted temporary file.
 */
class PlyWriter {
public:
  /**
   * @brief 建立唯一的同目錄暫存檔並寫入 PLY header。
   *
   * @param path 完成後要替換的目的路徑。
   * @throws std::runtime_error 暫存檔無法建立或開啟。
   *
   * @par English
   * Create a unique temporary file beside the destination and write its PLY header. Parameter
   * path is the final destination. Throws std::runtime_error if the temporary file cannot be
   * created or opened.
   */
  explicit PlyWriter(const std::string &path) : destination(path) {
    // mkstemp reserves a unique sibling so final rename stays on the same
    // filesystem.
    // 中文：mkstemp 在目的檔旁保留唯一的暫存檔，讓最後的重新命名留在同一個 檔案系統中。
    std::string pattern = path + ".tmp.XXXXXX";
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back(0);
    int fd = mkstemp(name.data());
    if (fd < 0)
      throw std::runtime_error("Cannot create output: " + path);
    close(fd);
    temporary = name.data();
    file.open(temporary, std::ios::binary);
    if (!file) {
      std::filesystem::remove(temporary);
      throw std::runtime_error("Cannot open temporary PLY");
    }
    file << "ply\nformat binary_little_endian 1.0\nelement vertex ";
    countPosition = file.tellp();
    file << std::setw(20) << 0
         << "\nproperty float x\nproperty float y\nproperty float "
            "z\nproperty float intensity\nend_header\n";
  }
  /**
   * @brief 關閉串流並以不拋出 filesystem 例外的方式嘗試清除未提交暫存檔。
   *
   * @par English
   * Close the stream and attempt to remove an uncommitted temporary file without throwing
   * filesystem exceptions.
   */
  ~PlyWriter() {
    file.close();
    if (!temporary.empty()) {
      std::error_code ec;
      std::filesystem::remove(temporary, ec);
    }
  }
  /**
   * @brief 將一批 float XYZ 與 intensity 轉成 little-endian bytes 寫入。
   *
   * 使用 4096 點的固定 staging buffer，無需將所有輸出轉成第二份完整 byte
   * array。
   *
   * @param points 本批要輸出的點，呼叫期間不得被修改。
   * @throws std::runtime_error 寫入串流失敗，例如磁碟空間不足。
   *
   * @par English
   * Encode a batch of float XYZ and intensity values as little-endian bytes. A fixed staging
   * buffer holds 4096 points, avoiding a second byte array for the entire output. Parameter
   * points is the current batch and must not change during the call. Throws std::runtime_error
   * on write failure, for example when the disk is full.
   */
  void append(const std::vector<Point> &points) {
    // Bounded staging buffer avoids per-coordinate file writes and host-endian
    // assumptions.
    // 中文：有界的批次緩衝區避免逐座標寫檔，也避免依賴主機的 位元組順序。
    std::array<char, 16 * 4096> data{};
    size_t used = 0;
    for (const auto &p : points) {
      for (float v : {p.x, p.y, p.z, p.intensity}) {
        uint32_t bits;
        std::memcpy(&bits, &v, 4);
        for (int i = 0; i < 4; ++i)
          data[used++] = char(bits >> (8 * i));
      }
      if (used == data.size()) {
        file.write(data.data(), used);
        used = 0;
      }
    }
    if (used)
      file.write(data.data(), used);
    if (!file)
      throw std::runtime_error("PLY write failed (check available disk space)");
    count += points.size();
  }
  /**
   * @brief 回填 vertex count、確認串流完成，再重新命名提交。
   *
   * 重新命名位於同一檔案系統；提交後清除 temporary，使解構時不移除目的檔。
   *
   * @throws std::runtime_error 回填、flush 或 close 失敗。
   * @throws std::filesystem::filesystem_error rename 失敗。
   *
   * @par English
   * Fill the vertex count, verify the stream, and rename the temporary file to commit. Both
   * paths are on the same filesystem. Clear temporary after committing so destruction does not
   * remove the destination. Throws std::runtime_error if patching, flushing, or closing fails,
   * or std::filesystem::filesystem_error if rename fails.
   */
  void finish() {
    file.seekp(countPosition);
    file << std::setw(20) << count;
    file.flush();
    if (!file)
      throw std::runtime_error("Cannot finalize PLY");
    file.close();
    if (file.fail())
      throw std::runtime_error("Cannot close PLY");
    std::filesystem::rename(temporary, destination);
    temporary.clear();
  }

private:
  /** @brief 提交時要替換的目的 PLY 路徑。
   *
   * @par English
   * The destination PLY path to replace when committing.
   */
  std::string destination, temporary; ///< 目的路徑與尚未提交的唯一暫存路徑。
  ///< EN: Final destination path and the unique temporary path awaiting commit.
  std::ofstream file;                 ///< 暫存檔的 binary 輸出串流。
  ///< EN: Binary output stream for the temporary file.
  std::streampos countPosition; ///< 20 字元 vertex count 欄位的起點。
  ///< EN: Offset of the 20-character vertex-count field.
  uint64_t count = 0;           ///< 所有 append() 已寫出的頂點總數。
  ///< EN: Total vertices written by all append() calls.
};
} // namespace
// 中文：匿名命名空間。
/**
 * @brief 驗證體素邊長並設定顯示樣本容量上限。
 *
 *
 * @par English
 * Validate the voxel width and configure the display sample limit.
 */
VoxelGrid::VoxelGrid(double size, size_t sampleLimit)
    : leafSize(size), sampleLimit(sampleLimit) {
  if (!std::isfinite(size) || size <= 0)
    throw std::runtime_error("Voxel size must be finite and positive");
}
/**
 * @brief 混合三軸區塊索引，避免相近座標形成相近雜湊值。
 *
 *
 * @par English
 * Mix the three block coordinates so nearby coordinates do not produce clustered hash values.
 */
size_t VoxelGrid::Hash::operator()(const Key &k) const {
  return mix(uint32_t(k.x)) ^
         mix(uint64_t(uint32_t(k.y)) + 0x9e3779b97f4a7c15ULL) ^
         mix(uint64_t(uint32_t(k.z)) + 0x3c6ef372fe94f82aULL);
}
/**
 * @brief 新增佔用格並更新有界顯示樣本。
 *
 * 先將格子分成區塊座標與 0..7 的局部座標，再令 local=x+8*y+64*z。
 * local/64 選擇字組、local%64 選擇 bit；同格重複點不影響 reservoir
 * 或第一筆強度。 強度以 set-bit 的 rank 緊密排列，插入時只搬移同一個小區塊的
 * float。
 *
 *
 * @par English
 * Add occupied cells and update the bounded display sample. Split a voxel into block
 * coordinates and local coordinates in the range 0..7, then encode local = x + 8*y + 64*z.
 * local/64 selects the word and local%64 selects the bit. Duplicate points in one cell do not
 * change its first intensity or reservoir membership. Intensities are packed by set-bit rank;
 * insertion moves floats only within a small block.
 */
void VoxelGrid::add(const Cloud &cloud) {
  if (hasFrame && frameId != cloud.frame)
    throw std::runtime_error("Cannot merge different frame IDs: " + frameId +
                             " / " + cloud.frame);
  for (auto p : cloud.points) {
    coordinate(p.x, leafSize);
    coordinate(p.y, leafSize);
    coordinate(p.z, leafSize);
  }
  if (!hasFrame) {
    frameId = cloud.frame;
    hasFrame = true;
  }
  /**
   * @brief 對格子索引做向負無限大除以八，取得區塊索引。
   *
   * 先轉成 int64_t 再取負數，避免 INT32_MIN 的溢位。
   *
   * v 為有號格子索引。
   * 結果為包含該格子的區塊座標。
   *
   * @par English
   * Divide a signed voxel index by eight with rounding toward negative infinity to obtain its
   * block index. Convert to int64_t before negation to avoid overflow for INT32_MIN. Input v is
   * a signed voxel index; the result identifies the containing block.
   */
  auto blockIndex = [](int32_t v) {
    return int32_t(v >= 0 ? int64_t(v) / 8 : -((-int64_t(v) + 7) / 8));
  };
  for (auto p : cloud.points) {
    int32_t x = coordinate(p.x, leafSize), y = coordinate(p.y, leafSize),
            z = coordinate(p.z, leafSize);
    Key key{blockIndex(x), blockIndex(y), blockIndex(z)};
    unsigned local = unsigned(int64_t(x) - int64_t(key.x) * 8) +
                     8 * unsigned(int64_t(y) - int64_t(key.y) * 8) +
                     64 * unsigned(int64_t(z) - int64_t(key.z) * 8);
    auto &block = blocks[key];
    auto &bits = block.bits;
    uint64_t mask = uint64_t(1) << (local % 64);
    if (bits[local / 64] & mask)
      continue;
    float intensity = std::isfinite(p.intensity)
                          ? p.intensity
                          : std::numeric_limits<float>::quiet_NaN();
    if (!block.intensities.empty() || std::isfinite(intensity)) {
      size_t rank = 0, total = 0;
      for (unsigned word = 0; word < 8; ++word) {
        auto count = size_t(__builtin_popcountll(bits[word]));
        total += count;
        if (word < local / 64)
          rank += count;
      }
      rank += size_t(__builtin_popcountll(bits[local / 64] & (mask - 1)));
      if (block.intensities.empty())
        block.intensities.resize(total,
                                 std::numeric_limits<float>::quiet_NaN());
      block.intensities.insert(block.intensities.begin() + rank, intensity);
    }
    bits[local / 64] |= mask;
    ++voxelCount;
    Point center{float((double(x) + .5) * leafSize),
                 float((double(y) + .5) * leafSize),
                 float((double(z) + .5) * leafSize), intensity};
    // Uniform reservoir sample: display memory is bounded independently of map
    // size.
    // 中文：均勻的 reservoir 取樣讓顯示記憶體保持有界，不隨完整地圖 大小增加。
    if (samples.size() < sampleLimit)
      samples.push_back(center);
    else if (sampleLimit) {
      auto i =
          std::uniform_int_distribution<uint64_t>(0, voxelCount - 1)(random);
      if (i < sampleLimit)
        samples[size_t(i)] = center;
    }
  }
  inputCount += cloud.points.size();
}
/**
 * @brief 遍歷非零佔用 bit 並還原體素中心。
 *
 * ctzll 僅在 bits 非零時呼叫；bits &= bits-1 每次移除最低的一個 set bit。
 * 使用 int64_t 計算區塊原點，再加上半格，避免整數乘八溢位。
 *
 *
 * @par English
 * Enumerate occupied bits and reconstruct voxel centers. Call ctzll only for nonzero bits;
 * bits &= bits-1 removes the lowest set bit each iteration. Compute block origins in int64_t
 * before adding a half-cell offset to avoid integer overflow when multiplying by eight.
 */
void VoxelGrid::forEachPoint(const std::function<void(Point)> &visitor) const {
  for (const auto &block : blocks) {
    size_t rank = 0;
    for (unsigned word = 0; word < 8; ++word) {
      uint64_t bits = block.second.bits[word];
      while (bits) {
        unsigned local = word * 64 + unsigned(__builtin_ctzll(bits));
        bits &= bits - 1;
        Point p{float((int64_t(block.first.x) * 8 + int(local % 8) + .5) *
                      leafSize),
                float((int64_t(block.first.y) * 8 + int((local / 8) % 8) + .5) *
                      leafSize),
                float((int64_t(block.first.z) * 8 + int(local / 64) + .5) *
                      leafSize)};
        if (!block.second.intensities.empty())
          p.intensity = block.second.intensities[rank];
        ++rank;
        visitor(p);
      }
    }
  }
}
/**
 * @brief 清除地圖與統計，並重置固定亂數序列以支援可重現重建。
 *
 * @par English
 * Clear the map and counters, and reset the fixed random sequence for reproducible rebuilds.
 */
void VoxelGrid::clear() {
  blocks.clear();
  samples.clear();
  frameId.clear();
  hasFrame = false;
  inputCount = 0;
  voxelCount = 0;
  random.seed(42);
}
/**
 * @brief 按幀邊界完成累計並處理退回／取消。
 *
 * loaded 只在整幀成功加入後增加；回呼觀察到的永遠是完整幀狀態。
 *
 *
 * @par English
 * Accumulate at frame boundaries, including backward seeks and cancellation. Increase loaded
 * only after a complete frame has been added, so callbacks always observe a fully processed
 * frame boundary.
 */
bool Accumulation::seek(size_t target, const Progress &progress) {
  if (target >= reader.frames.size())
    throw std::runtime_error("Frame index out of range");
  if (target + 1 < loaded) {
    grid.clear();
    loaded = 0;
  }
  while (loaded <= target) {
    grid.add(reader.read(loaded));
    ++loaded;
    if (progress && !progress(loaded, target + 1))
      return false;
  }
  return true;
}
/**
 * @brief 將完整佔用格網分批送入 PLY 寫入器。
 *
 *
 * @par English
 * Send the complete occupied voxel map to the PLY writer in bounded batches.
 */
void writeVoxelPly(const std::string &path, const VoxelGrid &grid) {
  PlyWriter writer(path);
  std::vector<Point> batch;
  batch.reserve(4096);
  grid.forEachPoint([&](Point p) {
    batch.push_back(p);
    if (batch.size() == 4096) {
      writer.append(batch);
      batch.clear();
    }
  });
  writer.append(batch);
  writer.finish();
}
/**
 * @brief 將現有 XYZ 陣列透過暫存檔輸出並提交。
 *
 *
 * @par English
 * Write and commit an existing XYZ array, including intensity, through a temporary file.
 */
void writeBinaryPly(const std::string &path, const std::vector<Point> &points) {
  PlyWriter writer(path);
  writer.append(points);
  writer.finish();
}
/**
 * @brief 逐幀解碼並寫出原始有效 XYZ，避免保留整包資料。
 *
 *
 * @par English
 * Decode and write original valid XYZ points and their intensities one frame at a time,
 * without retaining the whole bag.
 */
bool exportRawPly(Reader &reader, const std::string &path,
                  const Progress &progress) {
  PlyWriter writer(path);
  std::string frame;
  bool first = true;
  for (size_t i = 0; i < reader.frames.size(); ++i) {
    auto cloud = reader.read(i);
    if (!first && frame != cloud.frame)
      throw std::runtime_error("Cannot export mixed frame IDs without TF");
    frame = cloud.frame;
    first = false;
    writer.append(cloud.points);
    if (progress && !progress(i + 1, reader.frames.size()))
      return false;
  }
  writer.finish();
  return true;
}
} // namespace bag
// 中文：bag 命名空間。
