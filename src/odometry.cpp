/**
 * @file odometry.cpp
 * @brief Odometry 軌跡載入、排序及最近時間查詢的實作。
 *
 * 所有配對均使用訊息時間；不根據讀檔順序假設點雲與位姿一一對應。
 *
 * @par English
 * Load, sort, and query odometry trajectories by nearest timestamp. Matching always uses
 * message time rather than assuming clouds and poses correspond one-to-one by file order.
 */
#include "odometry.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
namespace bag {
/**
 * @brief 依索引讀入所有位姿後改以 Header.stamp 排序。
 *
 *
 * @par English
 * Read all indexed poses, then sort them by Header.stamp.
 */
OdometryTrack::OdometryTrack(Reader &reader) {
  samples.reserve(reader.odometryFrames.size());
  for (size_t i = 0; i < reader.odometryFrames.size(); ++i)
    samples.push_back(reader.readOdometry(i));
  sort();
}
/**
 * @brief 接收已解碼陣列並建立排序後的軌跡。
 *
 *
 * @par English
 * Take an existing decoded array and build a timestamp-sorted track.
 */
OdometryTrack::OdometryTrack(std::vector<Pose> poses)
    : samples(std::move(poses)) {
  sort();
}
/**
 * @brief 穩定排序 Header 時間，以允許來源 bag 時間與訊息時間順序不同。
 *
 * @par English
 * Stably sort by Header time, allowing bag recording order to differ from message timestamp
 * order.
 */
void OdometryTrack::sort() {
  std::stable_sort(
      samples.begin(), samples.end(),
      [](const auto &a, const auto &b) { return a.stamp < b.stamp; });
}
/**
 * @brief 檢查 lower_bound 兩側樣本，選出最近且通過門檻的位姿。
 *
 * lower_bound 得到第一個不早於目標的樣本；與其前一個樣本比較即可得到最近者。
 * 查詢落在時間範圍外仍可使用端點，但必須通過 maxDelta。
 *
 *
 * @par English
 * Compare samples on both sides of lower_bound and choose the nearest acceptable pose.
 * lower_bound finds the first sample not earlier than the target; comparing it with its
 * predecessor is sufficient. Queries outside the track may use an endpoint only if it meets
 * maxDelta.
 */
std::optional<size_t> OdometryTrack::match(double stamp,
                                           const std::string &frame,
                                           double maxDelta) const {
  if (samples.empty() || !std::isfinite(stamp) || !std::isfinite(maxDelta) ||
      maxDelta < 0)
    return {};
  auto it =
      std::lower_bound(samples.begin(), samples.end(), stamp,
                       [](const auto &p, double t) { return p.stamp < t; });
  size_t i =
      it == samples.end() ? samples.size() - 1 : size_t(it - samples.begin());
  if (i > 0 && std::abs(samples[i - 1].stamp - stamp) <=
                   std::abs(samples[i].stamp - stamp))
    --i;
  if (samples[i].frame != frame ||
      std::abs(samples[i].stamp - stamp) > maxDelta)
    return {};
  return i;
}
} // namespace bag
// 中文：bag 命名空間。
