/**
 * @file bag.cpp
 * @brief ROS1 bag record、PointCloud2 與 Odometry 的二進位解析實作。
 *
 * bag 的長度與 header 整數採 little-endian；PointCloud2 資料端序由訊息指定。
 * 所有位元組讀取均先檢查可用長度，並限制單一配置以避免損壞長度造成巨大配置。
 *
 * @par English
 * Binary parsing of ROS1 bag records, PointCloud2, and Odometry. Bag lengths and header
 * integers use little-endian encoding; PointCloud2 data uses the byte order specified in the
 * message. Byte reads are checked and individual allocations are bounded to reject malformed
 * oversized lengths.
 */
#include "bag.hpp"
#include <algorithm>
#include <array>
#include <bzlib.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <lz4frame.h>
#include <stdexcept>
namespace bag {
namespace {
constexpr uint32_t maxBlock =
    512u * 1024 * 1024; ///< 單一 record 或解壓 chunk 的配置上限。
    ///< EN: Maximum allocation for one record or decompressed chunk.
/**
 * @brief 集中檢查解析前置條件並拋出可讀錯誤。
 *
 * @param ok 條件是否成立。
 * @param s 失敗時的錯誤說明。
 * @throws std::runtime_error ok 為 false。
 *
 * @par English
 * Check a parsing precondition and throw a readable error. Parameters: ok states whether the
 * condition holds; s describes a failure. Throws std::runtime_error when ok is false.
 */
void require(bool ok, const std::string &s) {
  if (!ok)
    throw std::runtime_error(s);
}
/**
 * @brief 不擁有資料、具有邊界檢查的記憶體讀取游標。
 *
 * take() 推進指標並扣除剩餘長度。被參考的位元組緩衝區必須比游標活得更久。
 *
 * @par English
 * A non-owning, bounds-checked memory cursor. take() advances the pointer and subtracts the
 * consumed length. The referenced byte buffer must outlive the cursor.
 */
struct Cursor {
  const uint8_t *p; ///< 下一個待讀位元組，非擁有指標。
  ///< EN: Non-owning pointer to the next unread byte.
  size_t n;         ///< 由 p 起算的剩餘可讀長度。
  ///< EN: Bytes remaining from p onward.
  /**
   * @brief 繫結完整位元組緩衝區，不進行複製。
   *
   * @param b 必須在游標使用期間持續有效的資料。
   *
   * @par English
   * Reference a complete byte buffer without copying. Parameter b must remain valid while the
   * cursor is used.
   */
  Cursor(const Bytes &b) : p(b.data()), n(b.size()) {}
  /**
   * @brief 繫結一段有明確長度的記憶體。
   *
   * @param v 資料起點。
   * @param s 可讀取的位元組數。
   *
   * @par English
   * Reference a memory region with an explicit length. Parameters: v is its start; s is the
   * number of readable bytes.
   */
  Cursor(const uint8_t *v, size_t s) : p(v), n(s) {}
  /**
   * @brief 檢查並消耗指定長度的資料。
   *
   * @param s 欲讀取的位元組數。
   * @return 原先游標位置，可讀取至少 s bytes。
   * @throws std::runtime_error 剩餘資料不足。
   *
   * @par English
   * Validate and consume a requested byte count. Parameter s is the number of bytes. Returns the
   * previous pointer, from which at least s bytes may be read. Throws std::runtime_error if
   * insufficient data remains.
   */
  const uint8_t *take(size_t s) {
    require(s <= n, "Truncated record/message");
    auto v = p;
    p += s;
    n -= s;
    return v;
  }
  /**
   * @brief 讀取一個無號位元組並前進。
   *
   * @return 讀取值。
   * @throws std::runtime_error 已到資料尾端。
   *
   * @par English
   * Read an unsigned byte and advance. Returns its value; throws std::runtime_error at the end
   * of the buffer.
   */
  uint8_t u8() { return *take(1); }
  /**
   * @brief 逐位元組組合 little-endian uint32_t。
   *
   * 不依賴主機端序，亦不要求輸入指標對齊。
   *
   * @return 解碼的 32-bit 無號整數。
   * @throws std::runtime_error 剩餘長度不足四個位元組。
   *
   * @par English
   * Assemble a little-endian uint32_t one byte at a time, without depending on host byte order
   * or input alignment. Returns the decoded integer. Throws std::runtime_error if fewer than
   * four bytes remain.
   */
  uint32_t u32() {
    auto v = take(4);
    return uint32_t(v[0]) | (uint32_t(v[1]) << 8) | (uint32_t(v[2]) << 16) |
           (uint32_t(v[3]) << 24);
  }
  /**
   * @brief 讀取 ROS 的 uint32 長度前綴字串。
   *
   * 相同長度前綴形式也用於 bag header 的 key=value 欄位。
   *
   * @return 依宣告長度複製的字串，可包含二進位位元組。
   * @throws std::runtime_error 長度前綴或字串本體不完整。
   *
   * @par English
   * Read a ROS string prefixed by a uint32 length. Bag header key=value fields use the same
   * length-prefix representation. Returns a copy of the declared byte range, which may contain
   * binary bytes. Throws std::runtime_error if the length or content is truncated.
   */
  std::string str() {
    auto s = u32();
    auto v = take(s);
    return {reinterpret_cast<const char *>(v), s};
  }
};
/**
 * @brief bag record header 的鍵值表；值可能是二進位整數，並非一般文字。
 *
 * @par English
 * Bag record header lookup; values may be binary integers rather than ordinary text.
 */
using Header = std::map<std::string, Bytes>;
/**
 * @brief 解析連續的長度前綴 key=value 欄位。
 *
 * 只以第一個等號分隔鍵值，避免二進位值中的等號被誤解。重複鍵由後者覆蓋。
 *
 * @param b 不含外層 header length 的 header 區塊。
 * @return 名稱到原始 value bytes 的對照表。
 * @throws std::runtime_error 欄位截斷或缺少等號。
 *
 * @par English
 * Parse length-prefixed key=value fields. Split at the first equals sign only so equals signs
 * inside binary values remain intact. Later duplicate keys overwrite earlier ones. Parameter b
 * is the header payload without the outer header length. Returns a map from names to raw value
 * bytes. Throws std::runtime_error for truncated fields or a missing equals sign.
 */
Header header(const Bytes &b) {
  Cursor c(b);
  Header h;
  while (c.n) {
    auto s = c.str();
    auto i = s.find('=');
    require(i != std::string::npos, "Invalid header field");
    h[s.substr(0, i)] = Bytes(s.begin() + i + 1, s.end());
  }
  return h;
}
/**
 * @brief 查找必要 header 欄位。
 *
 * @param h 已解析的 header。
 * @param k 必須存在的欄位名稱。
 * @return h 中 value 的唯讀參考。
 * @throws std::runtime_error 欄位不存在。
 *
 * @par English
 * Find a required header field. Parameters: h is the parsed header; k is the required key.
 * Returns a const reference to its value in h. Throws std::runtime_error if the key is absent.
 */
const Bytes &field(const Header &h, const std::string &k) {
  auto i = h.find(k);
  require(i != h.end(), "Missing header: " + k);
  return i->second;
}
/**
 * @brief 取得恰好四位元組的 header 整數。
 *
 * @param h 來源 header。
 * @param k 整數欄位名稱。
 * @return little-endian 無號整數。
 * @throws std::runtime_error 欄位不存在或長度不為四。
 *
 * @par English
 * Read a header integer with exactly four bytes. Parameters: h is the header; k is the field
 * name. Returns a little-endian unsigned integer. Throws std::runtime_error if the field is
 * missing or its width is not four.
 */
uint32_t num(const Header &h, const std::string &k) {
  Cursor c(field(h, k));
  require(c.n == 4, "Invalid integer header");
  return c.u32();
}
/**
 * @brief 將 header value 視為文字複製。
 *
 * @param h 來源 header。
 * @param k 文字欄位名稱。
 * @return topic、訊息型別或壓縮名稱等文字。
 * @throws std::runtime_error 欄位不存在。
 *
 * @par English
 * Copy a header value as text. Parameters: h is the header; k is the text field name. Returns
 * a topic name, message type, compression name, or other string. Throws std::runtime_error if
 * the field is absent.
 */
std::string str(const Header &h, const std::string &k) {
  auto &b = field(h, k);
  return {b.begin(), b.end()};
}
/**
 * @brief 讀取並驗證一位元組的 bag record opcode。
 *
 * @param h 來源 header。
 * @return record 類型代碼。
 * @throws std::runtime_error op 缺漏或長度錯誤。
 *
 * @par English
 * Read and validate the one-byte bag record opcode. Parameter h is the header. Returns the
 * record type. Throws std::runtime_error if op is missing or has the wrong size.
 */
uint8_t op(const Header &h) {
  auto &b = field(h, "op");
  require(b.size() == 1, "Invalid opcode");
  return b[0];
}
/**
 * @brief 從檔案完整讀取指定長度。
 *
 * @param f 已開啟的二進位輸入串流。
 * @param p 至少具有 n bytes 空間的輸出記憶體。
 * @param n 要讀取的位元組數，允許零。
 * @throws std::runtime_error 串流失敗或資料不足。
 *
 * @par English
 * Read exactly the requested number of bytes from a file. Parameters: f is an open binary
 * stream; p provides at least n writable bytes; n is the byte count and may be zero. Throws
 * std::runtime_error on stream failure or truncation.
 */
void readExact(std::ifstream &f, void *p, size_t n) {
  if (n)
    f.read(static_cast<char *>(p), n);
  require(bool(f), "Truncated bag file");
}
/**
 * @brief 從檔案讀取 little-endian 長度或整數欄位。
 *
 * @param f 已開啟的二進位串流。
 * @return 32-bit 無號整數。
 * @throws std::runtime_error 檔案截斷。
 *
 * @par English
 * Read a little-endian length or integer from a file. Parameter f is an open binary stream.
 * Returns a uint32_t. Throws std::runtime_error if the file is truncated.
 */
uint32_t read32(std::ifstream &f) {
  std::array<uint8_t, 4> b{};
  readExact(f, b.data(), 4);
  return uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) |
         (uint32_t(b[3]) << 24);
}
/**
 * @brief 配置受上限限制的緩衝區並讀入檔案資料。
 *
 * @param f 來源串流。
 * @param n 欲配置與讀取的長度。
 * @return 擁有資料的位元組陣列。
 * @throws std::runtime_error 長度超過 512 MiB 或檔案截斷。
 *
 * @par English
 * Allocate a bounded byte buffer and read it from the stream. Parameters: f is the source; n
 * is the allocation and read length. Returns an owning byte array. Throws std::runtime_error
 * for lengths above 512 MiB or truncated files.
 */
Bytes readBytes(std::ifstream &f, uint32_t n) {
  require(n <= maxBlock, "Record exceeds 512 MiB limit");
  Bytes b(n);
  readExact(f, b.data(), n);
  return b;
}
/**
 * @brief 查詢 PointField datatype 的儲存寬度。
 *
 * 依序支援 INT8、UINT8、INT16、UINT16、INT32、UINT32、FLOAT32、FLOAT64。
 *
 * @param t ROS PointField datatype，合法範圍 1 至 8。
 * @return 型別寬度，單位 bytes。
 * @throws std::runtime_error 不支援的 datatype。
 *
 * @par English
 * Get the storage width of a PointField datatype. Supports INT8, UINT8, INT16, UINT16, INT32,
 * UINT32, FLOAT32, and FLOAT64. Parameter t is the ROS datatype code, from 1 through 8.
 * Returns its size in bytes. Throws std::runtime_error for unsupported codes.
 */
size_t typeSize(uint8_t t) {
  static const size_t sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
  require(t > 0 && t < 9, "Invalid PointField datatype");
  return sizes[t];
}
/**
 * @brief 解碼指定端序與 PointField 型別的單一數值。
 *
 * 先逐位元組組合位元表示，再用 memcpy 還原浮點值，避免對齊及 strict aliasing
 * 問題。
 *
 * @param p 至少 typeSize(t) bytes 的資料。
 * @param t PointField datatype。
 * @param big true 代表來源為 big-endian；false 為 little-endian。
 * @return 轉為 double 的數值，不在此處過濾 NaN／Inf。
 *
 * @par English
 * Decode a scalar with the specified PointField type and byte order. Assemble its bits byte by
 * byte and restore floats with memcpy to avoid alignment and strict-aliasing problems.
 * Parameters: p supplies at least typeSize(t) bytes; t is the datatype; big selects big-endian
 * when true and little-endian otherwise. Returns a double; NaN and Inf are not filtered here.
 */
double scalar(const uint8_t *p, uint8_t t, bool big) {
  size_t n = typeSize(t);
  uint64_t bits = 0;
  for (size_t i = 0; i < n; ++i)
    bits |= uint64_t(p[i]) << (8 * (big ? n - 1 - i : i));
  switch (t) {
  case 1:
    return int8_t(bits);
  case 2:
    return uint8_t(bits);
  case 3:
    return int16_t(bits);
  case 4:
    return uint16_t(bits);
  case 5:
    return int32_t(bits);
  case 6:
    return uint32_t(bits);
  case 7: {
    uint32_t v = bits;
    float f;
    std::memcpy(&f, &v, 4);
    return f;
  }
  default: {
    double d;
    std::memcpy(&d, &bits, 8);
    return d;
  }
  }
}
} // namespace
// 中文：匿名命名空間。
/**
 * @brief 依欄位描述與二維 strides 解碼有效 XYZ。
 *
 * 先解析 Header、PointField 與 data 長度，再驗證 XYZ 不超出 point_step。
 * 逐行以 row_step 定位，不能假設 row_step 等於 width * point_step。
 *
 *
 * @par English
 * Decode finite XYZ using field descriptors and two-dimensional strides. Parse the Header,
 * PointFields, and data length before validating that coordinates fit within point_step.
 * Locate each row through row_step rather than assuming row_step equals width * point_step.
 */
Cloud decode(const Bytes &data) {
  Cursor c(data);
  Cloud out;
  c.u32();
  auto sec = c.u32(), ns = c.u32();
  out.stamp = sec + ns * 1e-9;
  out.frame = c.str();
  out.height = c.u32();
  out.width = c.u32();
  /**
   * @brief 單一 PointField 解碼所需的最小欄位資訊。
   *
   * @par English
   * Minimal metadata needed to decode one PointField.
   */
  struct F {
    uint32_t offset; ///< 欄位相對於單一點起點的位移。
    ///< EN: Field offset relative to the beginning of a point.
    uint8_t type;    ///< ROS PointField datatype。
    ///< 中文：ROS PointField 的資料型別代碼。
  };
  std::map<std::string, F> fields;
  auto count = c.u32();
  require(count <= 1024, "Too many PointFields");
  for (uint32_t i = 0; i < count; ++i) {
    auto name = c.str();
    auto offset = c.u32();
    auto type = c.u8();
    auto n = c.u32();
    require(n > 0, "Empty PointField");
    if (name == "intensity")
      require(n == 1, "Intensity field must be scalar");
    typeSize(type);
    fields[name] = {offset, type};
  }
  bool big = c.u8() != 0;
  out.pointStep = c.u32();
  auto row = c.u32(), len = c.u32();
  auto points = c.take(len);
  c.u8();
  require(c.n == 0, "Trailing PointCloud2 bytes");
  require(out.pointStep > 0 && uint64_t(out.width) * out.pointStep <= row,
          "Invalid point/row step");
  require(uint64_t(row) * out.height == len, "Invalid PointCloud2 data size");
  std::array<F, 3> xyz{};
  size_t a = 0;
  for (auto name : {"x", "y", "z"}) {
    auto it = fields.find(name);
    require(it != fields.end(), std::string("Missing coordinate: ") + name);
    auto f = it->second;
    require(uint64_t(f.offset) + typeSize(f.type) <= out.pointStep,
            "Field outside point_step");
    xyz[a++] = f;
  }
  auto intensity = fields.find("intensity");
  out.hasIntensity = intensity != fields.end();
  if (out.hasIntensity)
    require(uint64_t(intensity->second.offset) +
                    typeSize(intensity->second.type) <=
                out.pointStep,
            "Intensity field outside point_step");
  out.points.reserve(uint64_t(out.width) * out.height);
  for (uint32_t y = 0; y < out.height; ++y)
    for (uint32_t x = 0; x < out.width; ++x) {
      auto p = points + size_t(y) * row + size_t(x) * out.pointStep;
      Point v{float(scalar(p + xyz[0].offset, xyz[0].type, big)),
              float(scalar(p + xyz[1].offset, xyz[1].type, big)),
              float(scalar(p + xyz[2].offset, xyz[2].type, big))};
      if (out.hasIntensity) {
        auto f = intensity->second;
        float value = float(scalar(p + f.offset, f.type, big));
        if (std::isfinite(value))
          v.intensity = value;
      }
      if (std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z))
        out.points.push_back(v);
    }
  return out;
}
/**
 * @brief 擷取位姿並檢查完整 Odometry 訊息長度。
 *
 *
 * @par English
 * Extract a pose and validate the complete Odometry message length.
 */
Pose decodeOdometry(const Bytes &data) {
  Cursor c(data);
  Pose pose;
  c.u32();
  auto sec = c.u32(), ns = c.u32();
  require(ns < 1000000000, "Invalid odometry nanoseconds");
  pose.stamp = sec + ns * 1e-9;
  pose.frame = c.str();
  pose.childFrame = c.str();
  for (auto &v : pose.position) {
    v = scalar(c.take(8), 8, false);
    require(std::isfinite(v), "Nonfinite odometry position");
  }
  double norm = 0;
  for (auto &v : pose.orientation) {
    v = scalar(c.take(8), 8, false);
    require(std::isfinite(v), "Nonfinite odometry orientation");
    norm += v * v;
  }
  require(std::isfinite(norm) && norm > 1e-20, "Invalid odometry quaternion");
  for (auto &v : pose.orientation)
    v /= std::sqrt(norm);
  c.take((36 + 6 + 36) * 8); // pose covariance, twist, twist covariance
  // 中文：位姿共變異數、速度及速度共變異數。
  require(c.n == 0, "Trailing Odometry bytes");
  return pose;
}
/**
 * @brief 掃描頂層 records，建立延遲載入 chunk 與兩種訊息的索引。
 *
 * 0x05 記錄 chunk 資料位置，0x04 提供該 chunk 的訊息偏移，0x07 提供 connection
 * 型別。 先收集索引、最後依 connection 過濾，因此支援 connection metadata
 * 放在檔案尾端。
 *
 *
 * @par English
 * Scan top-level records to build lazy chunk metadata and both message indices. Opcode 0x05
 * locates chunk data, 0x04 supplies message offsets in that chunk, and 0x07 supplies
 * connection types. Collect indices first and filter by connection later, supporting metadata
 * placed at the end of the file.
 */
Reader::Reader(const std::string &path, const std::string &topic,
               const std::string &odometryTopic)
    : file(path, std::ios::binary) {
  require(bool(file), "Cannot open bag: " + path);
  char magic[13];
  readExact(file, magic, 13);
  require(std::string(magic, 13) == "#ROSBAG V2.0\n",
          "Only ROS1 bag v2.0 is supported");
  file.seekg(0, std::ios::end);
  auto end = uint64_t(file.tellg());
  file.seekg(13);
  /**
   * @brief 掃描階段尚未依 topic 過濾的索引項目。
   *
   * @par English
   * An index entry not yet filtered by topic during the scan.
   */
  struct Entry {
    uint32_t conn; ///< 用於稍後比對 topic 與型別的 connection ID。
    ///< EN: Connection ID used later to match the topic and type.
    Frame frame;   ///< 對應訊息的 chunk、offset 與 bag 時間。
    ///< EN: The message's chunk, offset, and bag recording time.
  };
  // Scan record headers and message indexes; skip chunk payloads until read().
  // 中文：掃描 record headers 與訊息索引，直到 read() 才載入 chunk payload。
  std::vector<Entry> entries;
  uint64_t chunk = UINT64_MAX;
  while (uint64_t(file.tellg()) < end) {
    auto pos = uint64_t(file.tellg());
    auto h = header(readBytes(file, read32(file)));
    auto length = read32(file);
    auto dataPos = uint64_t(file.tellg());
    require(dataPos <= end && length <= end - dataPos,
            "Record extends beyond file");
    auto code = op(h);
    if (code == 5) {
      chunk = pos;
      chunks[pos] = {dataPos, length, num(h, "size"), str(h, "compression")};
    } else if (code == 7) {
      auto d = header(readBytes(file, length));
      connections[num(h, "conn")] = {str(h, "topic"), str(d, "type")};
    } else if (code == 4) {
      require(chunk != UINT64_MAX, "Index without chunk");
      require(num(h, "ver") == 1, "Unsupported index version");
      auto n = num(h, "count"), id = num(h, "conn");
      require(uint64_t(n) * 12 == length, "Invalid index length");
      auto b = readBytes(file, length);
      Cursor c(b);
      for (uint32_t i = 0; i < n; ++i) {
        auto sec = c.u32(), ns = c.u32(), offset = c.u32();
        entries.push_back({id, {chunk, offset, sec + ns * 1e-9}});
      }
    }
    file.seekg(dataPos + length);
    require(bool(file), "Invalid record seek");
  }
  for (const auto &e : entries) {
    auto it = connections.find(e.conn);
    if (it != connections.end() && it->second.first == topic) {
      require(it->second.second == "sensor_msgs/PointCloud2",
              "Topic is not sensor_msgs/PointCloud2");
      frames.push_back(e.frame);
    }
    if (!odometryTopic.empty() && it != connections.end() &&
        it->second.first == odometryTopic) {
      require(it->second.second == "nav_msgs/Odometry",
              "Odometry topic is not nav_msgs/Odometry");
      odometryFrames.push_back(e.frame);
    }
  }
  require(!frames.empty(), "No indexed PointCloud2 frames for " + topic +
                               " (bag may require reindexing)");
  std::stable_sort(frames.begin(), frames.end(),
                   [](auto &a, auto &b) { return a.time < b.time; });
}
/**
 * @brief 將點雲索引對應的 payload 交給 PointCloud2 解碼器。
 *
 *
 * @par English
 * Pass the indexed point cloud payload to the PointCloud2 decoder.
 */
Cloud Reader::read(size_t frame) {
  require(frame < frames.size(), "Frame index out of range");
  return decode(readMessage(frames[frame]));
}
/**
 * @brief 將 Odometry 索引對應的 payload 交給位姿解碼器。
 *
 *
 * @par English
 * Pass the indexed Odometry payload to the pose decoder.
 */
Pose Reader::readOdometry(size_t frame) {
  require(frame < odometryFrames.size(), "Odometry frame index out of range");
  return decodeOdometry(readMessage(odometryFrames[frame]));
}
/**
 * @brief 按需載入與解壓 chunk，並複製指定 message payload。
 *
 * 壓縮名稱與解壓長度必須吻合。LZ4
 * 分段解壓同時檢查輸入消耗、輸出大小與完成狀態。 回傳獨立
 * Bytes，下一次讀取不會破壞呼叫端正在解碼的資料。
 *
 *
 * @par English
 * Load and decompress a chunk on demand, then copy the requested message payload. Compression
 * names and decompressed sizes must match. LZ4 processing checks consumed input, produced
 * output, and completion. Return owning Bytes so the next read cannot invalidate data
 * currently being decoded by the caller.
 */
Bytes Reader::readMessage(const Frame &ref) {
  auto ch = chunks.at(ref.chunk);
  // Retain only one decompressed chunk, independent of the total bag size.
  // 中文：僅保留一個解壓後的 chunk，與整個 bag 大小無關。
  if (cached != ref.chunk) {
    cached = UINT64_MAX;
    file.clear();
    file.seekg(ch.data);
    auto compressed = readBytes(file, ch.length);
    require(ch.size <= maxBlock, "Chunk exceeds 512 MiB limit");
    if (ch.compression == "none") {
      require(ch.length == ch.size, "Invalid chunk size");
      buffer = std::move(compressed);
    } else if (ch.compression == "bz2") {
      buffer.resize(ch.size);
      unsigned int n = ch.size;
      auto rc = BZ2_bzBuffToBuffDecompress(
          reinterpret_cast<char *>(buffer.data()), &n,
          reinterpret_cast<char *>(compressed.data()), compressed.size(), 0, 0);
      require(rc == BZ_OK && n == ch.size, "BZ2 decompression failed");
    } else if (ch.compression == "lz4") {
      buffer.resize(ch.size);
      LZ4F_dctx *ctx = nullptr;
      require(
          !LZ4F_isError(LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION)),
          "Cannot create LZ4 context");
      size_t in = 0, out = 0, result = 1;
      while (in < compressed.size() && result) {
        size_t sn = compressed.size() - in, dn = buffer.size() - out;
        result = LZ4F_decompress(ctx, buffer.data() + out, &dn,
                                 compressed.data() + in, &sn, nullptr);
        if (LZ4F_isError(result) || (!sn && !dn))
          break;
        in += sn;
        out += dn;
      }
      LZ4F_freeDecompressionContext(ctx);
      require(result == 0 && in == compressed.size() && out == ch.size,
              "LZ4 decompression failed");
    } else {
      throw std::runtime_error("Unsupported compression: " + ch.compression);
    }
    cached = ref.chunk;
  }
  require(ref.offset < buffer.size(), "Invalid frame offset");
  Cursor c(buffer.data() + ref.offset, buffer.size() - ref.offset);
  auto hn = c.u32();
  auto hp = c.take(hn);
  auto h = header(Bytes(hp, hp + hn));
  require(op(h) == 2, "Index does not point to message");
  auto n = c.u32();
  auto p = c.take(n);
  return Bytes(p, p + n);
}
/**
 * @brief 使用九位有效數字匯出單幀 ASCII XYZ 與原始 intensity。
 *
 *
 * @par English
 * Export a single frame as ASCII XYZ and original intensity using nine significant digits.
 */
void writePly(const std::string &path, const Cloud &cloud) {
  std::ofstream f(path);
  require(bool(f), "Cannot create PLY: " + path);
  f << "ply\nformat ascii 1.0\nelement vertex " << cloud.points.size()
    << "\nproperty float x\nproperty float y\nproperty float z\nproperty float "
       "intensity\nend_header\n";
  f.precision(9);
  for (auto p : cloud.points)
    f << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << '\n';
  require(bool(f), "PLY write failed");
}
} // namespace bag
// 中文：bag 命名空間。
