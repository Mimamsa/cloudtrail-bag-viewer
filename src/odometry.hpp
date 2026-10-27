/**
 * @file odometry.hpp
 * @brief Odometry 位姿序列與點雲時間配對介面。
 *
 * 同步依據為訊息 Header.stamp，而非 bag 記錄時間；不插值、不套用 TF。
 *
 * @par English
 * Interfaces for odometry poses and matching to point cloud timestamps. Synchronize using
 * message Header.stamp, not bag recording time; do not interpolate or apply TF.
 */
#pragma once
#include "bag.hpp"
#include <optional>
namespace bag {
/**
 * @brief 依訊息時間排序並查詢最近位姿的唯讀軌跡。
 *
 * 載入時只保留 Pose，不保留完整 Odometry 訊息，適合每幀重複查詢。
 *
 * @par English
 * A read-only trajectory sorted by message time with nearest-pose lookup. Retain Pose objects
 * rather than complete Odometry messages for repeated per-frame queries.
 */
class OdometryTrack {
public:
  /**
   * @brief 從讀取器的 Odometry 索引解碼整條軌跡。
   *
   * @param reader 已建立點雲及 Odometry 索引的讀取器。
   * @throws std::runtime_error 任一 Odometry 訊息無法解碼。
   *
   * @par English
   * Decode the entire track from a reader's odometry indices. Parameter reader already has cloud
   * and odometry indices. Throws std::runtime_error if any odometry message cannot be decoded.
   */
  explicit OdometryTrack(Reader &reader);
  /**
   * @brief 以既有 Pose 陣列建立並穩定排序軌跡。
   *
   * @param poses 將移入物件的位姿資料；應具有有限時間與有效位姿。
   *
   * @par English
   * Build a track by moving in and stably sorting an existing Pose array. Parameter poses should
   * contain finite timestamps and valid poses.
   */
  explicit OdometryTrack(std::vector<Pose> poses);
  // Match Header.stamp, never bag recording time. Earlier sample wins ties.
  // 中文：以 Header.stamp 配對，而非 bag 記錄時間；時間距離相同時選較早的樣本。
  /**
   * @brief 以二分搜尋選取最接近點雲時間的 Odometry。
   *
   * 先選擇時間最近的樣本，同距離時取較早者，再檢查 frame 與時間差。
   * 空軌跡、非有限時間／容差、負容差、座標系不同或超出容差時不回傳位置。
   * 查詢成本為 O(log N)，不進行位置或四元數插值。
   *
   * @param stamp 點雲 Header.stamp，單位秒。
   * @param frame 要求與位姿 parent frame 一致的座標系。
   * @param maxDelta 可接受的最大絕對時間差，秒；預設 0.2。
   * @return poses() 中的索引；無可接受配對時為 std::nullopt。
   *
   * @par English
   * Find the nearest odometry sample using binary search. Choose by time first, preferring the
   * earlier sample on a tie, then check frame and tolerance. Return no match for an empty track,
   * nonfinite time/tolerance, negative tolerance, frame mismatch, or excessive time difference.
   * Query cost is O(log N); position and quaternion interpolation are not performed. Parameters:
   * stamp is the cloud Header time in seconds; frame must match the pose's parent frame;
   * maxDelta is the maximum absolute difference in seconds, default 0.2. Returns an index into
   * poses(), or std::nullopt when unacceptable.
   */
  std::optional<size_t> match(double stamp, const std::string &frame,
                              double maxDelta = .2) const;
  /**
   * @brief 取得按 Header.stamp 排序的完整軌跡。
   *
   * @return 物件生命週期內有效的唯讀陣列參考。
   *
   * @par English
   * Get the complete track sorted by Header.stamp. Returns a const reference valid for the
   * object's lifetime.
   */
  const std::vector<Pose> &poses() const { return samples; }

private:
  std::vector<Pose> samples; ///< 全部位姿，依 Header.stamp 遞增排序。
  ///< EN: All poses sorted in ascending Header.stamp order.
  /**
   * @brief 依 Header.stamp 穩定排序；相同時間維持來源次序。
   *
   * @par English
   * Stably sort by Header.stamp, preserving source order for equal timestamps.
   */
  void sort();
};
} // namespace bag
// 中文：bag 命名空間。
