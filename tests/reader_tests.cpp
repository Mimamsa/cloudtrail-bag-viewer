/**
 * @file reader_tests.cpp
 * @brief 不依賴 ROS 的解析、體素、同步及匯出回歸測試。
 *
 * 測試自行建立小型序列化訊息與兩個 chunk 的 bag，涵蓋 none、bz2、LZ4。
 * 測試產物寫在 CTest 工作目錄，通過後移除；不需要大型實際 bag 或圖形環境。
 * 所有判斷使用 check()，不受 Release 模式關閉 assert 的影響。
 *
 * @par English
 * ROS-independent regression tests for parsing, voxels, synchronization, and export. Build
 * small serialized messages and two-chunk bags covering none, bz2, and LZ4. Write fixtures in
 * the CTest working directory and remove them after success; neither a large bag nor a GUI is
 * needed. All assertions use check(), remaining active in Release builds where assert may be
 * disabled.
 */
#include "accumulation.hpp"
#include "bag.hpp"
#include "intensity.hpp"
#include "odometry.hpp"
#include "playback.hpp"
#include <algorithm>
#include <bzlib.h>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <lz4frame.h>
#include <stdexcept>
#include <tuple>
using bag::Bytes;
/**
 * @brief 附加一個 little-endian uint32_t，供測試資料建構使用。
 *
 * @param b 接收 bytes 的陣列。
 * @param v 要序列化的值。
 *
 * @par English
 * Append a little-endian uint32_t to construct test data. Parameters: b receives the bytes; v
 * is the value to serialize.
 */
void u32(Bytes &b, uint32_t v) {
  for (int i = 0; i < 4; ++i)
    b.push_back(v >> (8 * i));
}
/**
 * @brief 附加 ROS 長度前綴字串。
 *
 * @param b 接收 bytes 的陣列。
 * @param s 不含結尾 NUL 的字串內容。
 *
 * @par English
 * Append a ROS length-prefixed string. Parameters: b receives the bytes; s supplies content
 * without a terminating NUL.
 */
void str(Bytes &b, const std::string &s) {
  u32(b, s.size());
  b.insert(b.end(), s.begin(), s.end());
}
/**
 * @brief 建立具有固定位置與非單位四元數的合法 Odometry 訊息。
 *
 * 位置固定 (1,2,3)，四元數 (0,0,2,2) 對應正規化後繞 Z 旋轉 90 度。
 * 補齊 covariance 與 twist，使截斷測試能驗證完整訊息長度。
 *
 * @param sec Header.stamp 的整秒部分，奈秒固定 500000000。
 * @return 完整序列化 Odometry payload。
 *
 * @par English
 * Build a valid Odometry message with fixed position and a deliberately unnormalized
 * quaternion. Translation is (1,2,3); quaternion (0,0,2,2) normalizes to a 90-degree Z
 * rotation. Include covariance and twist so truncation tests validate the full message length.
 * Parameter sec is the whole-second Header timestamp, with nanoseconds fixed at 500000000.
 * Returns the complete serialized payload.
 */
Bytes odometryMessage(uint32_t sec = 123) {
  Bytes b;
  u32(b, 1);
  u32(b, sec);
  u32(b, 500000000);
  str(b, "map");
  str(b, "sensor_frame");
  // Position and a deliberately unnormalised 90 degree rotation about Z.
  // 中文：位置資料與刻意未正規化的繞 Z 軸 90 度旋轉四元數。
  for (double v : {1., 2., 3., 0., 0., 2., 2.}) {
    uint64_t bits;
    std::memcpy(&bits, &v, 8);
    for (int i = 0; i < 8; ++i)
      b.push_back(bits >> (8 * i));
  }
  b.resize(b.size() + (36 + 6 + 36) * 8, 0);
  return b;
}
/**
 * @brief 建立具欄位重排、行尾 padding 與 NaN 的 PointCloud2 fixture。
 *
 * 使用兩行一列，XYZ 排列為 z、x、y；可選 intensity 緊接 XYZ，另保留四 bytes
 * 行尾 padding。 第一點為 (1,2,3)，第二點 Y 為
 * NaN，用於驗證過濾與組織尺寸仍保留。
 *
 * @param big 點資料是否採 big-endian；ROS 訊息外層仍為 little-endian。
 * @param intensityType 測試用 intensity datatype，涵蓋八種 PointField 型別。
 * @param withIntensity 是否包含 intensity 欄位。
 * @param intensityValue 要寫入的強度；整數型別會截斷小數。
 * @return 完整 PointCloud2 序列化 payload。
 *
 * @par English
 * Build a PointCloud2 fixture with reordered fields, row padding, and NaN. Use two rows and
 * one column with XYZ fields ordered z,x,y, followed by optional intensity and four bytes of
 * row padding. The first point is (1,2,3); the second has NaN Y to test filtering while
 * retaining organized dimensions. Parameters: big controls point-data byte order, while
 * message framing stays little-endian; intensityType covers all eight datatypes; withIntensity
 * includes or omits the field; intensityValue is truncated for integer types. Returns the
 * complete serialized payload.
 */
Bytes message(bool big, uint8_t intensityType = 7, bool withIntensity = true,
              float intensityValue = 42.5f) {
  Bytes b;
  u32(b, 1);
  u32(b, 123);
  u32(b, 500000000);
  str(b, "map");
  u32(b, 2);
  u32(b, 1);
  u32(b, withIntensity ? 4 : 3);
  for (auto name : {"z", "x", "y"}) {
    str(b, name);
    u32(b, name[0] == 'x' ? 4 : name[0] == 'y' ? 8 : 0);
    b.push_back(7);
    u32(b, 1);
  }
  const size_t widths[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
  size_t intensityBytes = withIntensity ? widths[intensityType] : 0;
  if (withIntensity) {
    str(b, "intensity");
    u32(b, 12);
    b.push_back(intensityType);
    u32(b, 1);
  }
  b.push_back(big);
  u32(b, 12 + intensityBytes);
  u32(b, 16 + intensityBytes);
  u32(b, 2 * (16 + intensityBytes));
  for (int row = 0; row < 2; ++row) {
    for (float v : {3.f, 1.f, row == 0 ? 2.f : std::nanf("")}) {
      uint32_t bits;
      std::memcpy(&bits, &v, 4);
      for (int i = 0; i < 4; ++i)
        b.push_back(bits >> (8 * (big ? 3 - i : i)));
    }
    if (withIntensity) {
      uint64_t bits = 0;
      if (intensityType == 7) {
        uint32_t v;
        std::memcpy(&v, &intensityValue, 4);
        bits = v;
      } else if (intensityType == 8) {
        double v = intensityValue;
        std::memcpy(&bits, &v, 8);
      } else
        bits = uint64_t(int64_t(intensityValue));
      for (size_t i = 0; i < intensityBytes; ++i)
        b.push_back(bits >> (8 * (big ? intensityBytes - 1 - i : i)));
    }
    u32(b, 0); // Four bytes of row padding, distinct from intensity.
    // 中文：四個行尾 padding 位元組，與 intensity 欄位分開。
  }
  b.push_back(0);
  return b;
}
/**
 * @brief 以 Release 模式仍有效的條件判斷回報測試失敗。
 *
 * @param v 期待成立的條件。
 * @throws std::runtime_error 條件不成立。
 *
 * @par English
 * Check a test condition even when Release builds disable assert. Parameter v is expected to
 * be true. Throws std::runtime_error when it is false.
 */
void check(bool v) {
  if (!v)
    throw std::runtime_error("Test failed");
}
/**
 * @brief 產生四位元組的 little-endian header value。
 *
 * @param n 要編碼的無號整數。
 * @return 可直接放入 bag header 的 bytes。
 *
 * @par English
 * Create a four-byte little-endian header value. Parameter n is the unsigned integer. Returns
 * raw bytes suitable for a bag header.
 */
Bytes integer(uint32_t n) {
  Bytes b;
  u32(b, n);
  return b;
}
/**
 * @brief 將一般文字轉為不含長度前綴的 header value。
 *
 * @param s 原始文字。
 * @return 字元內容的 byte array。
 *
 * @par English
 * Convert text to a header value without a length prefix. Parameter s is the source text.
 * Returns its character bytes.
 */
Bytes text(const std::string &s) { return Bytes(s.begin(), s.end()); }
/**
 * @brief 建立一組 bag 長度前綴 key=value 欄位。
 *
 * @param fields 欄位名稱與二進位 value 的列表。
 * @return 不含最外層 header length 的 bag header bytes。
 *
 * @par English
 * Build length-prefixed key=value fields for a bag header. Parameter fields lists names and
 * binary values. Returns header bytes without the outer header length.
 */
Bytes header(std::initializer_list<std::pair<std::string, Bytes>> fields) {
  Bytes b;
  for (const auto &f : fields) {
    u32(b, f.first.size() + 1 + f.second.size());
    b.insert(b.end(), f.first.begin(), f.first.end());
    b.push_back('=');
    b.insert(b.end(), f.second.begin(), f.second.end());
  }
  return b;
}
/**
 * @brief 將一筆完整 bag record 附加到 fixture。
 *
 * @param b 接收 record 的 bag 或 chunk 陣列。
 * @param h 已編碼 header。
 * @param d record payload。
 *
 * @par English
 * Append a complete bag record to a fixture. Parameters: b is the receiving bag/chunk buffer;
 * h is the encoded header; d is its payload.
 */
void record(Bytes &b, const Bytes &h, const Bytes &d) {
  u32(b, h.size());
  b.insert(b.end(), h.begin(), h.end());
  u32(b, d.size());
  b.insert(b.end(), d.begin(), d.end());
}
/**
 * @brief 獨立解析 binary PLY 並逐座標比對預期輸出。
 *
 * 檢查格式、vertex count、float 位元組端序及結尾無額外資料。
 * 不依賴生產程式的 PLY reader，以免讀寫兩邊相同錯誤互相掩蓋。
 *
 * @param path 要檢查的輸出檔案。
 * @param expected 依檔案次序排列的預期 XYZ。
 * @throws std::runtime_error 任一格式或內容檢查失敗。
 *
 * @par English
 * Independently parse binary PLY and compare every coordinate and intensity with expected
 * output. Check format, vertex count, float byte order, and absence of trailing data. Avoid
 * reusing a production reader so matching read/write bugs cannot hide each other. Parameters:
 * path is the output under test; expected is ordered by file position. Throws
 * std::runtime_error on any mismatch.
 */
void checkBinaryPly(const std::filesystem::path &path,
                    const std::vector<bag::Point> &expected) {
  std::ifstream f(path, std::ios::binary);
  check(bool(f));
  std::string line;
  bool binary = false, hasIntensity = false;
  uint64_t count = UINT64_MAX;
  while (std::getline(f, line) && line != "end_header") {
    if (line == "format binary_little_endian 1.0")
      binary = true;
    if (line == "property float intensity")
      hasIntensity = true;
    if (line.rfind("element vertex ", 0) == 0)
      count = std::stoull(line.substr(15));
  }
  check(binary && hasIntensity && count == expected.size() &&
        line == "end_header");
  for (auto point : expected)
    for (float value : {point.x, point.y, point.z, point.intensity}) {
      uint8_t bytes[4];
      f.read(reinterpret_cast<char *>(bytes), 4);
      check(bool(f));
      uint32_t bits = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
                      (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
      float decoded;
      std::memcpy(&decoded, &bits, 4);
      check(decoded == value || (std::isnan(decoded) && std::isnan(value)));
    }
  check(f.peek() == std::char_traits<char>::eof());
}
/**
 * @brief 驗證體素佔用、中心、邊界、顯示上限及完整匯出。
 *
 * @test 同格重複點只佔一格；正負座標及跨區塊的三軸位元定位可還原正確中心。
 * @test 即使 sampleLimit 為零，完整地圖與完整 PLY 仍保留所有格子。
 * @test 不同 frame 被拒絕，clear() 後可加入新 frame。
 *
 * @par English
 * Test voxel occupancy, centers, boundaries, display limits, and complete export. Verify
 * duplicate points occupy one cell, and signed coordinates and all three block-local axes
 * reconstruct correctly. Verify full maps/PLY retain all cells even when sampleLimit is zero.
 * Reject mismatched frames, and permit a new frame after clear().
 */
void checkVoxels() {
  // 中文：固定色帶不受資料範圍影響；檢查邊界、中點與超出範圍的截斷。
  // EN: Fixed colors are data-independent; verify endpoints, midpoint, and out-of-range clamping.
  check(bag::heightColorScale.value(-30) == 0);
  check(bag::heightColorScale.value(10) == 1);
  check(bag::heightColorScale.value(-10) == .5f);
  check(bag::heightColorScale.value(-100) == 0);
  check(bag::heightColorScale.value(100) == 1);
  check(bag::heightColorScale.value(0) == .75f);
  // EN: Full-map bounds survive display sampling and reset on backward rebuilds.
  // 中文：完整地圖範圍不受顯示取樣影響，清除重建時重設。
  bag::HeightScale heights;
  check(heights.value(0) == .5f);
  heights.add(-2); heights.add(6);
  check(heights.value(-2) == 0 && heights.value(6) == 1);
  check(heights.value(0) == .25f && heights.value(100) == 1);
  bag::HeightScale flat;
  flat.add(3); flat.add(std::numeric_limits<double>::quiet_NaN());
  check(flat.value(3) == .5f);
  bag::VoxelGrid bounded(.05, 1);
  bag::Cloud tall;
  tall.frame = "map";
  tall.points = {{0, 0, -2}, {0, 0, 6}, {0, 0, 1}};
  bounded.add(tall);
  check(bounded.points().size() == 1);
  check(std::abs(bounded.heightRange().minimum + 1.975) < 1e-6);
  check(std::abs(bounded.heightRange().maximum - 6.025) < 1e-6);
  bounded.clear();
  check(!bounded.heightRange().valid);
  tall.points = {{0, 0, 3}};
  bounded.add(tall);
  check(bounded.heightRange().value(3.025) == .5f);
  bag::VoxelGrid grid;
  bag::Cloud c;
  c.frame = "map";
  c.points = {{.001f, 0, 0}, {.011f, 0, 0}, {.021f, 0, 0}};
  grid.add(c);
  c.points = {{.041f, 0, 0}};
  grid.add(c);
  check(grid.points().size() == 1 && grid.inputPoints() == 4);
  check(std::abs(grid.points()[0].x - .025f) < 1e-7f);
  c.points = {{-.001f, 0, 0}, {.051f, 0, 0}};
  grid.add(c);
  check(grid.points().size() == 3 && grid.inputPoints() == 6);
  c.frame = "other";
  bool rejected = false;
  try {
    grid.add(c);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected && grid.inputPoints() == 6);
  auto path = std::filesystem::current_path() / "voxel_test.ply";
  bag::writeBinaryPly(path.string(), grid.points());
  checkBinaryPly(path, grid.points());
  std::filesystem::remove(path);
  // The full exported map must be independent of the display sample limit.
  // 中文：完整匯出地圖不得受顯示取樣上限影響。
  bag::VoxelGrid small(.05, 1);
  c.frame = "map";
  c.points = {{-.401f, 0, 0}, {-.001f, 0, 0}, {.001f, 0, 0}, {.401f, 0, 0}};
  small.add(c);
  check(small.size() == 4 && small.points().size() == 1);
  std::vector<bag::Point> complete;
  small.forEachPoint([&](bag::Point p) { complete.push_back(p); });
  check(complete.size() == 4);
  for (auto p : complete)
    check(p.y == .025f && p.z == .025f);
  bag::writeVoxelPly(path.string(), small);
  checkBinaryPly(path, complete);
  std::filesystem::remove(path);
  bag::VoxelGrid dense(.05, 0);
  bag::Cloud block;
  block.frame = "map";
  for (int x = -8; x < 8; ++x)
    for (int y = 0; y < 8; ++y)
      for (int z = 0; z < 8; ++z)
        block.points.push_back({float((x + .5) * .05), float((y + .5) * .05),
                                float((z + .5) * .05)});
  dense.add(block);
  check(dense.size() == 1024 && dense.points().empty());
  std::vector<bag::Point> reconstructed;
  dense.forEachPoint([&](bag::Point p) { reconstructed.push_back(p); });
  auto order = [](auto a, auto b) {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
  };
  std::sort(reconstructed.begin(), reconstructed.end(), order);
  std::sort(block.points.begin(), block.points.end(), order);
  check(reconstructed.size() == block.points.size());
  for (size_t i = 0; i < reconstructed.size(); ++i)
    check(reconstructed[i].x == block.points[i].x &&
          reconstructed[i].y == block.points[i].y &&
          reconstructed[i].z == block.points[i].z);
  bag::writeVoxelPly(path.string(), dense);
  reconstructed.clear();
  dense.forEachPoint([&](bag::Point p) { reconstructed.push_back(p); });
  checkBinaryPly(path, reconstructed);
  std::filesystem::remove(path);
  grid.clear();
  c.frame = "new_map";
  grid.add(c);
  check(grid.inputPoints() == 4 && grid.frame() == "new_map");
}
/**
 * @brief 驗證位姿解碼與最近時間查詢的邊界行為。
 *
 * @test 四元數被正規化，零四元數與截斷訊息被拒絕。
 * @test 未排序輸入會排序，同距離選較早者，錯誤
 * frame、過大時間差與空軌跡不匹配。
 *
 * @par English
 * Test pose decoding and nearest-time query boundaries. Normalize quaternions and reject zero
 * quaternions or truncated messages. Sort unsorted input, choose earlier samples on ties, and
 * reject incorrect frames, excessive time differences, or empty tracks.
 */
void checkOdometry() {
  auto b = odometryMessage();
  auto p = bag::decodeOdometry(b);
  check(p.stamp == 123.5 && p.frame == "map" && p.childFrame == "sensor_frame");
  check(p.position[0] == 1 && p.position[1] == 2 && p.position[2] == 3);
  check(std::abs(p.orientation[2] - std::sqrt(.5)) < 1e-12 &&
        p.orientation[2] == p.orientation[3]);
  b.pop_back();
  bool rejected = false;
  try {
    bag::decodeOdometry(b);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  b = odometryMessage();
  size_t poseOffset = 16 + 3 + 4 + 12; // Header frame and child string.
  // 中文：Header 的 frame 與 child frame 字串。
  for (size_t i = poseOffset + 24; i < poseOffset + 56; ++i)
    b[i] = 0;
  rejected = false;
  try {
    bag::decodeOdometry(b);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  auto later = p;
  later.stamp = 124.5;
  bag::OdometryTrack track(std::vector<bag::Pose>{later, p});
  check(track.match(123.5, "map") == 0);
  check(track.match(124.5, "map") == 1);
  check(track.match(124., "map", .5) ==
        0); // Equal-distance tie chooses earlier pose.
        // 中文：時間距離相同時選擇較早的位姿。
  check(!track.match(124., "map"));
  check(!track.match(123.5, "odom"));
  check(!track.match(0, "map"));
  check(!track.match(200, "map"));
  check(track.match(123.4, "map") == 0);
  check(track.match(124.6, "map") == 1);
  bag::OdometryTrack empty(std::vector<bag::Pose>{});
  check(!empty.match(123.5, "map"));
}
/**
 * @brief 驗證絕對時間排程、延遲追趕及連續倍率切換，避免回歸為逐幀相對等待。
 *
 * @test 16x 在實際 0.5 秒後到第 80 幀，模擬處理停頓後直接跳到應到期的幀。
 * @test 暫停後重新錨定、拖曳後重新錨定，以及不規則／重複時間的邊界處理。
 *
 * @par English
 * Test absolute playback timing, delayed catch-up, and continuous rate changes to prevent a
 * return to relative per-frame waiting. At 16x, half a wall second reaches frame 80; simulated
 * processing stalls jump to the frame already due. Test re-anchoring after pause or seek, plus
 * irregular and duplicate timestamps.
 */
void checkPlayback() {
  std::vector<bag::Frame> frames;
  for (size_t i = 0; i <= 1000; ++i)
    frames.push_back({0, 0, 1000 + i * .1});
  bag::PlaybackClock clock;
  clock.reset(5, 1000, 16);
  check(clock.dueFrame(frames, 5) == 0 && clock.dueFrame(frames, 5.5) == 80);
  check(clock.dueFrame(frames, 7) ==
        320); // A blocked decoder cannot accumulate extra per-frame sleeps.
        // 中文：解碼器阻塞不應累加額外的逐幀等待時間。
  clock.reset(0, 1000, 1);
  check(clock.targetTime(2) == 1002);
  clock.setRate(2, 16);
  check(clock.targetTime(2) == 1002 && clock.targetTime(2.5) == 1010);
  check(clock.dueFrame(frames, 2.5) == 100 && clock.rate() == 16);
  clock.reset(100, frames[100].time, 8);
  check(clock.dueFrame(frames, 100) == 100 &&
        clock.dueFrame(frames, 100.5) == 140);
  clock.reset(0, 1000, .25);
  check(clock.dueFrame(frames, 2) == 5);
  clock.reset(0, 999, 1);
  check(clock.dueFrame(frames, 0) == 0 &&
        clock.dueFrame(frames, 10000) == 1000);
  std::vector<bag::Frame> irregular = {
      {0, 0, 10}, {0, 0, 10.1}, {0, 0, 10.1}, {0, 0, 10.7}};
  clock.reset(0, 10.1, 1);
  check(clock.dueFrame(irregular, 0) == 2 &&
        clock.dueFrame(irregular, .59) == 2 &&
        clock.dueFrame(irregular, .61) == 3);
  // 循環不繼承上輪逾期時間，並保留變更後倍率；亦涵蓋單幀時間軸。
  // EN: Loops discard previous lateness, preserve changed rates, and support one-frame timelines.
  clock.reset(0, 1000, 16);
  for (int cycle = 1; cycle <= 3; ++cycle) {
    const double wall = cycle * 20.;
    check(clock.dueFrame(frames, wall) == frames.size() - 1);
    clock.restart(wall, frames.front().time);
    check(clock.dueFrame(frames, wall) == 0);
    check(clock.dueFrame(frames, wall + .5) == 80 && clock.rate() == 16);
  }
  clock.setRate(61, 8);
  clock.restart(70, frames.front().time);
  check(clock.dueFrame(frames, 70.5) == 40 && clock.rate() == 8);
  std::vector<bag::Frame> single{{0, 0, 1000}};
  clock.restart(80, single.front().time);
  check(clock.dueFrame(single, 80) == 0 && clock.dueFrame(single, 81) == 0);
  bool rejected = false;
  try {
    clock.reset(0, 0, 0);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  rejected = false;
  try {
    clock.dueFrame({}, 0);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  bag::IntensityScale scale;
  std::vector<bag::Point> points(1000, bag::Point{0, 0, 0, 8});
  scale.update(points, 50);
  check(scale.finiteCount == 50 && scale.low == 8 && scale.high == 8);
}
/**
 * @brief 驗證強度解碼、體素強度順序、顯示正規化與 PLY 保留。
 *
 * @test 八種強度型別、大小端、缺值、非有限強度、不合法欄位偏移。
 * @test 同格保留第一點強度；區塊內亂序插入的強度仍與座標對應。
 * @test P1/P99 上色範圍、常數範圍及彩色轉換不更動原始強度。
 *
 * @par English
 * Test intensity decoding, voxel intensity ordering, display normalization, and PLY retention.
 * Cover all eight datatypes, both byte orders, missing/nonfinite values, and invalid offsets.
 * Retain the first intensity in a voxel and preserve correspondence after out-of-order block
 * inserts. Verify P1/P99 ranges and constant ranges, and ensure color mapping leaves source
 * values unchanged.
 */
void checkIntensity() {
  // 中文：驗證調色盤端點、截斷、連續性與缺值，避免回到灰階或產生無效 RGB。
  // EN: Check palette endpoints, clamping, continuity, and missing values for valid colored RGB.
  const auto lowColor = bag::intensityColor(0);
  const auto highColor = bag::intensityColor(1);
  check(lowColor[2] > lowColor[0] && highColor[0] > highColor[2]);
  check(bag::intensityColor(-1) == lowColor && bag::intensityColor(2) == highColor);
  const auto missingColor = bag::intensityColor(std::nanf(""));
  check(missingColor[0] == .4f && missingColor[1] == .4f && missingColor[2] == .4f);
  auto previous = lowColor;
  for (int i = 1; i <= 1000; ++i) {
    const auto rgb = bag::intensityColor(i / 1000.f);
    for (size_t c = 0; c < rgb.size(); ++c) {
      check(std::isfinite(rgb[c]) && rgb[c] >= 0 && rgb[c] <= 1);
      check(std::abs(rgb[c] - previous[c]) < .15f);
    }
    previous = rgb;
  }
  for (bool big : {false, true})
    for (uint8_t type = 1; type <= 8; ++type) {
      auto c = bag::decode(message(big, type));
      check(c.hasIntensity && c.points.size() == 1);
      check(c.points[0].intensity == (type >= 7 ? 42.5f : 42.f));
    }
  auto missing = bag::decode(message(false, 7, false));
  check(!missing.hasIntensity && std::isnan(missing.points[0].intensity));
  for (float value : {std::nanf(""), std::numeric_limits<float>::infinity()}) {
    auto c = bag::decode(message(false, 7, true, value));
    check(c.points.size() == 1 && std::isnan(c.points[0].intensity));
  }
  auto corrupt = message(false);
  Bytes name;
  str(name, "intensity");
  auto it =
      std::search(corrupt.begin(), corrupt.end(), name.begin(), name.end());
  check(it != corrupt.end());
  size_t offset = size_t(it - corrupt.begin()) + name.size();
  for (size_t i = 0; i < 4; ++i)
    corrupt[offset + i] = 255;
  bool rejected = false;
  try {
    bag::decode(corrupt);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  corrupt = message(false);
  corrupt[offset + 5] =
      2; // count starts after four offset bytes and one datatype byte.
      // 中文：count 位於四個 offset 位元組與一個 datatype 位元組之後。
  rejected = false;
  try {
    bag::decode(corrupt);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  bag::Cloud cloud;
  cloud.frame = "map";
  cloud.points = {{.151f, 0, 0, 15},
                  {.001f, 0, 0, 1},
                  {.051f, 0, 0, 5},
                  {.351f, 0, 0, 35},
                  {.011f, 0, 0, 100}};
  bag::VoxelGrid grid(.05, 1);
  grid.add(cloud);
  check(grid.size() == 4 && grid.points().size() == 1);
  std::vector<bag::Point> full;
  grid.forEachPoint([&](auto p) { full.push_back(p); });
  std::sort(full.begin(), full.end(), [](auto a, auto b) { return a.x < b.x; });
  check(full[0].intensity == 1 && full[1].intensity == 5 &&
        full[2].intensity == 15 && full[3].intensity == 35);
  cloud.points = {{.061f, 0, 0, 999}};
  grid.add(
      cloud); // Later returns cannot overwrite the representative intensity.
      // 中文：後續回波不得覆寫已選定的代表強度。
  std::vector<bag::Point> exported;
  grid.forEachPoint([&](auto p) { exported.push_back(p); });
  auto path = std::filesystem::current_path() / "intensity_test.ply";
  bag::writeVoxelPly(path.string(), grid);
  checkBinaryPly(path, exported);
  std::filesystem::remove(path);
  bag::VoxelGrid lazy;
  cloud.points = {{.001f, 0, 0}};
  lazy.add(cloud);
  cloud.points = {{.051f, 0, 0, 8}};
  lazy.add(cloud);
  lazy.forEachPoint([&](auto p) {
    if (p.x < .05f)
      check(std::isnan(p.intensity));
    else
      check(p.intensity == 8);
  });
  bag::IntensityScale scale;
  std::vector<bag::Point> points;
  for (int i = 0; i <= 100; ++i)
    points.push_back({0, 0, 0, float(i)});
  points.push_back({0, 0, 0});
  scale.update(points);
  check(scale.finiteCount == 101 && scale.low == 1 && scale.high == 99 &&
        scale.minimum == 0 && scale.maximum == 100);
  check(scale.value(0) == 0 && scale.value(100) == 1 &&
        scale.value(50) == .5f && points[100].intensity == 100);
  scale.update({{0, 0, 0, -3}, {0, 0, 0, -3}});
  check(scale.value(-3) == .5f);
  scale.update({{0, 0, 0}});
  check(scale.finiteCount == 0 && scale.value(10) == .3f);
}
/**
 * @brief 建立指定壓縮的雙 chunk bag，進行端到端回歸驗證。
 *
 * @test 點雲／Odometry 索引、非零 chunk offset、跨 chunk 隨機存取及 Header
 * 時間配對。
 * @test 累計取消後續讀、後退重建及同幀不重複累加。
 * @test 原始 PLY 取消保留既有目的檔，成功則輸出全部 XYZ。
 *
 * @param compression none、bz2 或 lz4。
 *
 * @par English
 * Create a two-chunk bag with the selected compression for end-to-end regression tests. Check
 * cloud/odometry indices, nonzero chunk offsets, cross-chunk random access, and Header-based
 * time matching. Test accumulation cancellation/resume, backward rebuilding, and duplicate
 * prevention. Canceled raw export must preserve an existing destination; success writes all
 * XYZ and intensity. Parameter compression is none, bz2, or lz4.
 */
void checkBag(const std::string &compression) {
  Bytes b = text("#ROSBAG V2.0\n");
  record(b,
         header({{"op", {3}},
                 {"index_pos", Bytes(8)},
                 {"conn_count", integer(2)},
                 {"chunk_count", integer(2)}}),
         {});
  for (uint32_t i = 0; i < 2; ++i) {
    Bytes chunk;
    // Connection records may precede messages within the chunk: the index
    // offset is not zero.
    // 中文：connection record 可出現在 chunk 的訊息之前，因此索引 位移不是零。
    record(chunk,
           header({{"op", {7}},
                   {"conn", integer(42)},
                   {"topic", text("/cloud_registered")}}),
           header({{"type", text("sensor_msgs/PointCloud2")}}));
    auto offset = chunk.size();
    Bytes time = integer(100 + i);
    u32(time, 0);
    record(chunk, header({{"op", {2}}, {"conn", integer(42)}, {"time", time}}),
           message(i == 1));
    auto odometryOffset = chunk.size();
    record(chunk, header({{"op", {2}}, {"conn", integer(43)}, {"time", time}}),
           odometryMessage(123 + i));
    Bytes compressed = chunk;
    if (compression == "bz2") {
      compressed.resize(chunk.size() * 2 + 600);
      unsigned int n = compressed.size();
      check(
          BZ2_bzBuffToBuffCompress(reinterpret_cast<char *>(compressed.data()),
                                   &n, reinterpret_cast<char *>(chunk.data()),
                                   chunk.size(), 1, 0, 0) == BZ_OK);
      compressed.resize(n);
    } else if (compression == "lz4") {
      compressed.resize(LZ4F_compressFrameBound(chunk.size(), nullptr));
      auto n = LZ4F_compressFrame(compressed.data(), compressed.size(),
                                  chunk.data(), chunk.size(), nullptr);
      check(!LZ4F_isError(n));
      compressed.resize(n);
    }
    record(b,
           header({{"op", {5}},
                   {"compression", text(compression)},
                   {"size", integer(chunk.size())}}),
           compressed);
    Bytes index = time;
    u32(index, offset);
    record(b,
           header({{"op", {4}},
                   {"ver", integer(1)},
                   {"conn", integer(42)},
                   {"count", integer(1)}}),
           index);
    Bytes odometryIndex = time;
    u32(odometryIndex, odometryOffset);
    record(b,
           header({{"op", {4}},
                   {"ver", integer(1)},
                   {"conn", integer(43)},
                   {"count", integer(1)}}),
           odometryIndex);
  }
  record(
      b,
      header(
          {{"op", {7}}, {"conn", integer(43)}, {"topic", text("/Odometry")}}),
      header({{"type", text("nav_msgs/Odometry")}}));
  record(b,
         header({{"op", {7}},
                 {"conn", integer(42)},
                 {"topic", text("/cloud_registered")}}),
         header({{"type", text("sensor_msgs/PointCloud2")}}));
  auto path =
      std::filesystem::current_path() / ("fixture_" + compression + ".bag");
  {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<char *>(b.data()), b.size());
    check(bool(f));
  }
  {
    bag::Reader reader(path.string());
    check(reader.frames.size() == 2);
    check(reader.odometryFrames.size() == 2);
    bag::OdometryTrack track(reader);
    check(track.poses().size() == 2 && track.poses()[0].stamp == 123.5 &&
          track.poses()[1].stamp == 124.5);
    for (auto i : {1, 0, 1}) {
      check(reader.readOdometry(i).stamp == 123.5 + i);
      check(reader.read(i).points.size() == 1);
    }
    check(track.match(reader.read(0).stamp, "map") ==
          0); // Header time differs from bag time 100.
          // 中文：Header 時間與 bag 記錄時間 100 不同。
    bag::Reader withoutOdometry(path.string(), "/cloud_registered", "");
    check(withoutOdometry.odometryFrames.empty());
    bag::Accumulation accumulation(reader);
    check(!accumulation.seek(1, [](size_t, size_t) { return false; }));
    check(accumulation.loadedFrames() == 1 &&
          accumulation.grid.inputPoints() == 1);
    check(accumulation.seek(1));
    check(accumulation.grid.inputPoints() == 2 &&
          accumulation.grid.points().size() == 1);
    accumulation.seek(1);
    check(accumulation.grid.inputPoints() == 2);
    accumulation.seek(0);
    check(accumulation.grid.inputPoints() == 1 &&
          accumulation.loadedFrames() == 1);
    accumulation.seek(1);
    check(accumulation.grid.inputPoints() == 2);
    auto exported =
        std::filesystem::current_path() / ("raw_" + compression + ".ply");
    {
      std::ofstream existing(exported);
      existing << "preserve me";
    }
    check(!bag::exportRawPly(reader, exported.string(),
                             [](size_t, size_t) { return false; }));
    {
      std::ifstream existing(exported);
      std::string line;
      std::getline(existing, line);
      check(line == "preserve me");
    }
    check(bag::exportRawPly(reader, exported.string()));
    checkBinaryPly(exported, {{1, 2, 3, 42.5f}, {1, 2, 3, 42.5f}});
    std::filesystem::remove(exported);

    for (auto i : {1, 0, 1}) {
      auto cloud = reader.read(i);
      check(cloud.points.size() == 1 && cloud.points[0].y == 2);
    }
    bool rejected = false;
    try {
      reader.read(2);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected);
  }
  bool rejected = false;
  try {
    bag::Reader wrong(path.string(), "/missing");
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
  std::filesystem::remove(path);
}
/**
 * @brief 依序執行所有無視窗回歸測試並轉換失敗為非零結束碼。
 *
 * @return 所有測試通過為 0；任一例外為 1，並在 stderr 列出錯誤。
 *
 * @par English
 * Run all headless regression tests and convert failures to a nonzero exit status. Returns
 * zero when all tests pass, or one on any exception with the error written to stderr.
 */
int main() {
  try {
    for (bool big : {false, true}) {
      auto b = message(big);
      auto c = bag::decode(b);
      check(c.width == 1 && c.height == 2 && c.frame == "map" &&
            c.stamp == 123.5);
      check(c.points.size() == 1 && c.points[0].x == 1 && c.points[0].y == 2 &&
            c.points[0].z == 3);
      b.pop_back();
      bool rejected = false;
      try {
        bag::decode(b);
      } catch (const std::exception &) {
        rejected = true;
      }
      check(rejected);
    }
    checkPlayback();
    checkIntensity();
    checkOdometry();
    checkVoxels();
    for (auto compression : {"none", "bz2", "lz4"})
      checkBag(compression);
    std::cout << "Odometry decoding/synchronization, voxel occupancy, "
                 "accumulation seeks/cancellation, binary export, "
                 "chunk indexing, random access, none/bz2/lz4 compression, "
                 "endian, field order, row padding, NaN filtering and "
                 "truncation checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
