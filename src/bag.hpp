/**
 * @file bag.hpp
 * @brief ROS1 bag v2.0 索引讀取與訊息解碼的公開介面。
 *
 * 所有結構皆使用標準 C++ 型別，不需要 ROS、Qt 或 PCL。
 * 點雲與位姿使用訊息 Header.stamp；Frame::time 則是 bag
 * 記錄時間，兩者不可混用。
 *
 * @par English
 * Public interfaces for ROS1 bag v2.0 indexing and message decoding. All structures use
 * standard C++ types without ROS, Qt, or PCL. Clouds and poses use message Header.stamp,
 * whereas Frame::time is bag recording time; these timestamps must not be confused.
 */
#pragma once
#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>
/**
 * @brief 獨立於 ROS 的 bag 解析、點雲累計及位姿同步工具。
 *
 * @par English
 * ROS-independent utilities for bag parsing, cloud accumulation, and pose synchronization.
 */
namespace bag {
/**
 * @brief 連續位元組緩衝區，保存原始 record、壓縮資料或序列化訊息。
 *
 * @par English
 * An owning contiguous byte buffer for raw records, compressed data, or serialized messages.
 */
using Bytes = std::vector<uint8_t>;
/**
 * @struct bag::Point
 * @brief 包含 XYZ 與可選反射強度的單精度三維點。
 *
 * 座標沿用來源訊息的座標系；此工具不執行 TF 或外參轉換。
 *
 * @par English
 * A single-precision 3D point with XYZ and optional reflectance intensity. Coordinates remain
 * in the source message's frame; no TF or extrinsic transformation is applied.
 */
/**
 * @var float bag::Point::x
 * @brief 來源 frame 中的 X 座標。
 *
 * @par English
 * X coordinate in the source frame.
 */
/**
 * @var float bag::Point::y
 * @brief 來源 frame 中的 Y 座標。
 *
 * @par English
 * Y coordinate in the source frame.
 */
struct Point {
  float x, y, z; ///< 來源座標系中的 X、Y、Z 座標。
  ///< EN: X, Y, and Z coordinates in the source frame.
  float intensity = std::numeric_limits<float>::quiet_NaN(); ///< 原始反射強度；缺少或無效時為
                                                             ///< NaN。
  ///< EN: Original reflectance intensity; missing or invalid values are represented as NaN.
};
/**
 * @struct bag::Cloud
 * @brief 已解碼並移除非有限 XYZ 的 PointCloud2。
 *
 * width、height 保留原始組織尺寸；points 是緊密排列的有效點，
 * 其數量可能小於 width * height，不再保留原始行列對應。
 *
 * @par English
 * A decoded PointCloud2 with nonfinite XYZ removed. width and height retain the original
 * organized dimensions. points is a compact array of valid points, so its size may be less
 * than width * height and it no longer preserves the original row/column correspondence.
 */
/**
 * @var uint32_t bag::Cloud::width
 * @brief 原始 PointCloud2 每行點數。
 *
 * @par English
 * Original number of points per PointCloud2 row.
 */
/**
 * @var uint32_t bag::Cloud::height
 * @brief 原始 PointCloud2 行數。
 *
 * @par English
 * Original number of PointCloud2 rows.
 */
struct Cloud {
  uint32_t width = 0, height = 0,
           pointStep = 0; ///< 原始寬、高與每點的序列化位元組跨度。
           ///< EN: Original width, height, and serialized byte stride per point.
  std::string frame;      ///< PointCloud2 Header.frame_id。
  ///< 中文：PointCloud2 的來源座標系識別字 Header.frame_id。
  double stamp = 0;       ///< PointCloud2 Header.stamp，以秒表示。
  ///< EN: PointCloud2 Header.stamp in seconds.
  bool hasIntensity = false; ///< 來源是否包含名為 intensity 的純量 PointField。
  ///< EN: Whether the source has a scalar PointField named intensity.
  std::vector<Point> points; ///< 移除 NaN／Inf 後的 XYZ 點陣列。
  ///< EN: Compact XYZ point array after filtering NaN/Inf coordinates.
};
/**
 * @brief 從 ROS1 序列化位元組解碼 PointCloud2。
 *
 * 依 PointField 名稱、offset、datatype 及資料端序讀取 XYZ，支援行尾 padding。
 * 保留 XYZ 與 intensity；缺少或非有限強度以 NaN 表示，不刪除有效 XYZ。
 * XYZ 中 NaN、Inf 或轉為 float 後非有限的點會被移除。
 *
 * @param data 完整 sensor_msgs/PointCloud2 訊息，不含 bag record header。
 * @return 保留來源時間、座標系與尺寸的有效點雲。
 * @throws std::runtime_error 訊息截斷、欄位缺漏、型別或 stride 不合法。
 *
 * @par English
 * Decode PointCloud2 from ROS1 serialized bytes. Read XYZ according to field names, offsets,
 * datatypes, and byte order, including row padding. Retain XYZ and intensity; missing or
 * nonfinite intensity becomes NaN without removing valid XYZ. Remove points whose XYZ is NaN,
 * Inf, or becomes nonfinite when converted to float. Parameter data is a complete
 * sensor_msgs/PointCloud2 payload without the bag record header. Returns valid points with
 * source time, frame, and dimensions. Throws std::runtime_error for truncation, missing
 * fields, invalid types, or invalid strides.
 */
Cloud decode(const Bytes &data);
/**
 * @struct bag::Pose
 * @brief Odometry 中 child frame 相對於 parent frame 的位姿。
 *
 * position 以 frame 為參考；orientation 使用 ROS 的 x,y,z,w 順序，
 * 由 decodeOdometry() 正規化。未儲存 covariance 與 twist。
 *
 * @par English
 * Odometry pose of a child frame relative to its parent frame. position is expressed in frame;
 * orientation follows ROS order x,y,z,w and is normalized by decodeOdometry(). Covariance and
 * twist are not retained.
 */
/**
 * @var std::string bag::Pose::frame
 * @brief 位姿的參考座標系，即 Odometry Header.frame_id。
 *
 * @par English
 * Pose reference frame, corresponding to Odometry Header.frame_id.
 */
struct Pose {
  double stamp = 0; ///< Odometry Header.stamp，以秒表示。
  ///< EN: Odometry Header.stamp in seconds.
  std::string frame,
      childFrame; ///< 位姿的 parent frame 與移動中的 child frame。
      ///< EN: Parent frame and moving child frame of the pose.
  std::array<double, 3>
      position{}; ///< XYZ 平移，單位沿用 Odometry（通常為公尺）。
      ///< EN: XYZ translation in the units used by odometry, normally meters.
  std::array<double, 4> orientation{}; ///< 旋轉四元數，順序 x、y、z、w。
  ///< EN: Rotation quaternion in x,y,z,w order.
};
/**
 * @brief 解碼 nav_msgs/Odometry 的 Header 與 Pose。
 *
 * 驗證 XYZ、四元數與奈秒欄位；跳過 pose covariance、twist、twist covariance，
 * 但仍要求訊息長度完整且無多餘尾端位元組。
 *
 * @param data 完整 ROS1 Odometry 序列化訊息。
 * @return 位置為 double、旋轉為單位四元數的位姿。
 * @throws std::runtime_error 截斷、非有限位置、無效四元數或無效時間。
 *
 * @par English
 * Decode the Header and Pose from nav_msgs/Odometry. Validate XYZ, quaternion, and
 * nanoseconds. Skip pose covariance, twist, and twist covariance, but require a complete
 * message with no trailing bytes. Parameter data is the complete ROS1 serialized payload.
 * Returns double-precision translation and a unit quaternion. Throws std::runtime_error for
 * truncation, nonfinite positions, invalid quaternions, or invalid timestamps.
 */
Pose decodeOdometry(const Bytes &data);
/**
 * @brief 單一訊息在 bag 中的索引位置。
 *
 * offset 是相對於解壓後 chunk 資料的位移，並非整個 bag 的檔案位移。
 *
 * @par English
 * Index location of one bag message. offset is relative to the decompressed chunk payload, not
 * the start of the bag file.
 */
struct Frame {
  uint64_t chunk;  ///< 外層 chunk record 起點的絕對檔案位移。
  ///< EN: Absolute file offset of the enclosing chunk record.
  uint32_t offset; ///< message record 在解壓後 chunk 中的位移。
  ///< EN: Offset of the message record within the decompressed chunk.
  double time; ///< bag 索引記錄時間，秒；不一定等於 Header.stamp。
  ///< EN: Bag index recording time in seconds; it may differ from Header.stamp.
};
/**
 * @class bag::Reader
 * @brief 具備單一 chunk 快取的 ROS1 bag v2.0 讀取器。
 *
 * 建構時掃描外層 record header 與索引，不載入所有 chunk。讀取訊息時才
 * 載入並解壓對應 chunk，支援 none、bz2 與標準 LZ4 frame。
 * 同一物件共用檔案游標與快取，不能由多個執行緒同時呼叫。
 *
 * @par English
 * A ROS1 bag v2.0 reader with a single-chunk cache. Construction scans top-level headers and
 * indices without loading every chunk. Message reads load and decompress the required chunk,
 * supporting none, bz2, and standard LZ4 frames. File position and cache are shared within the
 * object, so concurrent calls from multiple threads are not supported.
 */
/**
 * @var std::vector<Frame> bag::Reader::frames
 * @brief 按 bag 記錄時間排序的點雲訊息索引。
 *
 * @par English
 * Point cloud message indices sorted by bag recording time.
 */
class Reader {
public:
  /**
   * @brief 開啟 bag 並建立點雲與可選 Odometry 索引。
   *
   * 索引按 bag 記錄時間穩定排序。點雲 topic 必須存在且具有索引；
   * Odometry 可缺少，缺少時 odometryFrames 為空。
   *
   * @param path 輸入 bag 路徑。
   * @param topic 必須為 sensor_msgs/PointCloud2 的點雲 topic。
   * @param odometryTopic nav_msgs/Odometry topic；空字串代表不建立該索引。
   * @throws std::runtime_error
   * 檔案無法開啟、格式錯誤、訊息型別錯誤或點雲索引為空。
   *
   * @par English
   * Open a bag and index clouds and optional odometry. Indices are stably sorted by bag
   * recording time. The cloud topic must exist and have indices; odometry may be absent, leaving
   * odometryFrames empty. Parameters: path is the bag filename; topic must contain
   * sensor_msgs/PointCloud2; odometryTopic must contain nav_msgs/Odometry, or may be empty to
   * disable its indexing. Throws std::runtime_error for open failures, invalid formats, wrong
   * types, or an empty cloud index.
   */
  explicit Reader(const std::string &path,
                  const std::string &topic = "/cloud_registered",
                  const std::string &odometryTopic = "/Odometry");
  /**
   * @brief 讀取並解碼指定的點雲幀。
   *
   * @param frame frames 中的零起始索引。
   * @return 獨立持有 XYZ 資料的 Cloud，不依賴內部快取的生命週期。
   * @throws std::runtime_error 索引超界、解壓或解碼失敗。
   *
   * @par English
   * Read and decode a cloud frame. Parameter frame is a zero-based index into frames. Returns a
   * Cloud that owns its points independently of the internal cache. Throws std::runtime_error
   * for an out-of-range index, decompression failure, or decoding failure.
   */
  Cloud read(size_t frame);
  /**
   * @brief 讀取並解碼指定的 Odometry 訊息。
   *
   * @param frame odometryFrames 中的零起始索引。
   * @return 已驗證且正規化的位姿。
   * @throws std::runtime_error 索引超界、解壓或位姿解碼失敗。
   *
   * @par English
   * Read and decode an odometry message. Parameter frame is a zero-based index into
   * odometryFrames. Returns a validated, normalized pose. Throws std::runtime_error for an
   * out-of-range index, decompression failure, or invalid pose data.
   */
  Pose readOdometry(size_t frame);
  std::vector<Frame> frames,
      odometryFrames; ///< 點雲及 Odometry 索引；呼叫端應唯讀使用。
      ///< EN: Cloud and odometry indices; callers should treat them as read-only.
  std::map<uint32_t, std::pair<std::string, std::string>>
      connections; ///< connection ID 對應 (topic, ROS 訊息型別)。
      ///< EN: Connection ID mapped to a pair of topic name and ROS message type.

private:
  /**
   * @brief 依訊息索引取得原始 payload。
   *
   * 若目標不在快取中，先使舊快取失效，再讀取及解壓 chunk。
   * 驗證 offset 所在的 record 為 message data，再複製 payload。
   *
   * @param ref 指向 chunk 中 message record 的索引。
   * @return 不含 record header 的序列化訊息。
   * @throws std::runtime_error 不支援的壓縮、大小不符、截斷或無效 record。
   *
   * @par English
   * Retrieve a raw payload by message index. Invalidate the old cache before reading and
   * decompressing a different chunk. Validate that the record at the offset is message data,
   * then copy its payload. Parameter ref is the chunk/message index. Returns serialized bytes
   * without the record header. Throws std::runtime_error for unsupported compression, size
   * mismatch, truncation, or invalid records.
   */
  Bytes readMessage(const Frame &ref);
  /**
   * @struct bag::Reader::Chunk
   * @brief 供延遲讀取使用的 chunk 中繼資料。
   *
   * @par English
   * Chunk metadata used for deferred reading.
   */
  /**
   * @var uint32_t bag::Reader::Chunk::length
   * @brief 磁碟上壓縮 payload 的長度，單位 bytes。
   *
   * @par English
   * Compressed payload length on disk, in bytes.
   */
  struct Chunk {
    uint64_t data;         ///< 壓縮 payload 的絕對檔案位移。
    ///< EN: Absolute file offset of the compressed payload.
    uint32_t length, size; ///< 壓縮長度與預期解壓長度，單位 bytes。
    ///< EN: Compressed length and expected decompressed length, in bytes.
    std::string compression; ///< 壓縮名稱：none、bz2 或 lz4。
    ///< EN: Compression name: none, bz2, or lz4.
  };
  std::ifstream file;               ///< 唯一的二進位輸入串流。
  ///< EN: The reader's single binary input stream.
  std::map<uint64_t, Chunk> chunks; ///< chunk record 絕對位移到中繼資料的查表。
  ///< EN: Lookup from absolute chunk record offsets to metadata.
  uint64_t cached =
      UINT64_MAX; ///< 目前快取的 chunk key；UINT64_MAX 表示無有效快取。
      ///< EN: Cached chunk key; UINT64_MAX means no valid cache.
  Bytes buffer; ///< 最近一個 chunk 的完整解壓資料。
  ///< EN: Complete decompressed bytes of the most recently cached chunk.
};
/**
 * @brief 以 ASCII PLY 寫出單幀有效 XYZ 與 intensity。
 *
 * 直接開啟目標並覆寫，與整包 binary 匯出的暫存檔提交方式不同。
 *
 * @param path 輸出 PLY 路徑。
 * @param cloud 要匯出的點雲；寫出 points 的
 * XYZ、intensity，不寫來源尺寸或其他欄位。
 * @throws std::runtime_error 開檔或寫入失敗。
 *
 * @par English
 * Write one frame's valid XYZ and intensity as ASCII PLY. Open and overwrite the destination
 * directly, unlike the temporary-file commit used for whole-bag binary exports. Parameters:
 * path is the destination; cloud supplies points with XYZ and intensity, not source dimensions
 * or other fields. Throws std::runtime_error if opening or writing fails.
 */
void writePly(const std::string &path, const Cloud &cloud);
} // namespace bag
// 中文：bag 命名空間。
