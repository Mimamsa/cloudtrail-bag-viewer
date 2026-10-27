/**
 * @file playback.hpp
 * @brief 以單調實際時間推進 bag 播放位置，避免逐幀處理成本造成時間漂移。
 *
 * @par English
 * Advance bag playback with monotonic wall time so per-frame processing cost does not cause
 * timing drift.
 */
#pragma once
#include "bag.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace bag {
/**
 * @brief 將單調 wall clock 映射至 bag 記錄時間的播放時間軸。
 *
 * 目標時間 = anchorBag + (wall - anchorWall) * rate。解碼或繪圖延遲不改動錨點；
 * 下一次更新直接選取已到期的最後一幀，可略過中間畫面。累計地圖仍應依序加入所有幀。
 * 暫停／拖曳後由控制層 reset()；切換倍率使用 setRate() 維持目標時間連續。
 *
 * @par English
 * A playback timeline mapping monotonic wall time to bag recording time. Target time equals
 * anchorBag + (wall - anchorWall) * rate. Decoding/drawing delays do not move the anchor; the
 * next update selects the latest due frame, skipping only intermediate display updates.
 * Accumulation must still process every frame. The controller calls reset() after pause/seek
 * and setRate() for continuous rate changes.
 */
class PlaybackClock {
public:
  /**
   * @brief 設定播放起點與倍率。
   * @param wall 單調實際時間，以秒表示，不要求 Unix epoch。
   * @param bagTime 起始幀的 bag 記錄時間，秒。
   * @param rate 正且有限的播放倍率。
   * @throws std::runtime_error 任一輸入非有限，或倍率不為正數。
   *
   * @par English
   * Set the playback origin and rate. Parameters: wall is monotonic time in seconds and need not
   * use Unix epoch; bagTime is the starting frame's bag recording time; rate must be finite and
   * positive. Throws std::runtime_error for nonfinite inputs or a nonpositive rate.
   */
  void reset(double wall, double bagTime, double rate) {
    if (!std::isfinite(wall) || !std::isfinite(bagTime) ||
        !std::isfinite(rate) || rate <= 0)
      throw std::runtime_error("Invalid playback clock anchor/rate");
    anchorWall = wall;
    anchorBag = bagTime;
    multiplier = rate;
  }
  /**
   * @brief 取得當下應到達的 bag 時間，不包含任何逐幀額外等待。
   * @param wall 與 reset() 相同時間來源的目前秒數。
   * @return 依倍率推進後的目標時間；早於錨點的查詢限制為起點。
   *
   * @par English
   * Get the bag time due now without adding any per-frame wait. Parameter wall uses the same
   * clock source as reset(). Returns the advanced target time; queries before the anchor are
   * clamped to the start.
   */
  double targetTime(double wall) const {
    return anchorBag + std::max(0., wall - anchorWall) * multiplier;
  }
  /**
   * @brief 在當下目標時間重新錨定，連續切換播放倍率。
   * @param wall 目前單調時間，秒。
   * @param rate 新的正有限倍率。
   *
   * @par English
   * Change rate continuously by re-anchoring at the current target time. Parameters: wall is
   * current monotonic time in seconds; rate is the new positive finite multiplier.
   */
  void setRate(double wall, double rate) {
    reset(wall, targetTime(wall), rate);
  }
  /** @brief 取得目前設定的播放倍率。 @return 正有限倍率。
   *
   * @par English
   * Get the configured positive finite playback multiplier.
   */
  double rate() const { return multiplier; }
  /**
   * @brief 尋找目標時間之前最後一幀，將處理延遲轉成畫面合併而非時間漂移。
   * @param frames 已按 bag 記錄時間排序且非空的訊息索引。
   * @param wall 目前單調時間，秒。
   * @return 已到期的最後一幀；目標早於第一幀時回傳零，晚於最後幀時回傳末幀。
   * @throws std::runtime_error frames 為空。
   *
   * @par English
   * Find the last frame due by target time, turning processing delays into coalesced display
   * updates rather than drift. Parameters: frames is a nonempty index sorted by bag recording
   * time; wall is current monotonic time in seconds. Returns the latest due index, clamped to
   * zero before the first frame and the last index after the final frame. Throws
   * std::runtime_error for an empty index.
   */
  size_t dueFrame(const std::vector<Frame> &frames, double wall) const {
    if (frames.empty())
      throw std::runtime_error("Playback requires nonempty frame index");
    auto end =
        std::upper_bound(frames.begin(), frames.end(), targetTime(wall),
                         [](double t, const Frame &f) { return t < f.time; });
    return end == frames.begin() ? 0 : size_t(end - frames.begin() - 1);
  }

private:
  double anchorWall = 0; ///< 單調時間錨點，秒。
  ///< EN: Monotonic time anchor in seconds.
  double anchorBag = 0;  ///< 相同錨點對應的 bag 記錄時間。
  ///< EN: Bag recording time corresponding to the anchor.
  double multiplier = 1; ///< 正有限播放倍率。
  ///< EN: Positive finite playback multiplier.
};
} // namespace bag
// 中文：bag 命名空間。
