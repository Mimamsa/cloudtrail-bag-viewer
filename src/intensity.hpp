/**
 * @file intensity.hpp
 * @brief 反射強度顯示範圍的估計與正規化；不修改原始或匯出強度。
 *
 * @par English
 * Estimate and normalize reflectance intensity for display without modifying source or
 * exported values.
 */
#pragma once
#include "bag.hpp"
#include <algorithm>
#include <array>
#include <cmath>
namespace bag {
/**
 * @brief 以有限強度的第 1／99 百分位數估計顯示對比範圍。
 *
 * 只用於畫面色彩，避免少量異常高值壓暗整張點雲。超出範圍者顯示為端點顏色，
 * Point::intensity 與 PLY 儲存值完全不變。每次更新暫用至多 N 個 float（可由
 * maxSamples 限制），並於返回時釋放。
 *
 * @par English
 * Estimate display contrast from the 1st and 99th percentiles of finite intensities. This
 * affects display color only and prevents a few unusually high values from darkening the entire
 * cloud. Values outside the range use endpoint colors; Point::intensity and PLY values
 * remain unchanged. An update temporarily stores at most N floats, optionally capped by
 * maxSamples, and releases them before returning.
 */
struct IntensityScale {
  size_t finiteCount = 0; ///< 顯示資料中具有有限強度的點數。
  ///< EN: Number of displayed data points with finite intensity.
  float minimum = 0;      ///< 原始有限強度最小值。
  ///< EN: Minimum finite source intensity.
  float maximum = 0;      ///< 原始有限強度最大值。
  ///< EN: Maximum finite source intensity.
  float low = 0;          ///< 第 1 百分位數的色彩範圍下限。
  ///< EN: Lower color-range bound at the 1st percentile.
  float high = 0;         ///< 第 99 百分位數的色彩範圍上限。
  ///< EN: Upper color-range bound at the 99th percentile.
  /**
   * @brief 由目前顯示點重新計算強度統計，不保留輸入參考。
   * @param points 單幀或累計取樣點；NaN／Inf 強度不參與統計。
   * @param maxSamples 最多檢查的等距樣本數；零表示檢查全部顯示點。
   *
   * @par English
   * Recompute intensity statistics from current display points without retaining input
   * references. Parameters: points is a single frame or accumulated display sample, excluding
   * NaN/Inf intensities from statistics; maxSamples caps evenly spaced samples, with zero
   * examining all display points.
   */
  void update(const std::vector<Point> &points, size_t maxSamples = 0) {
    std::vector<float> values;
    size_t stride =
        maxSamples
            ? std::max(size_t(1), (points.size() + maxSamples - 1) / maxSamples)
            : 1;
    values.reserve(maxSamples ? std::min(maxSamples, points.size())
                              : points.size());
    for (size_t i = 0; i < points.size(); i += stride)
      if (std::isfinite(points[i].intensity))
        values.push_back(points[i].intensity);
    finiteCount = values.size();
    minimum = maximum = low = high = 0;
    if (values.empty())
      return;
    auto bounds = std::minmax_element(values.begin(), values.end());
    minimum = *bounds.first;
    maximum = *bounds.second;
    size_t lo = (values.size() - 1) / 100,
           hi = (99 * (values.size() - 1) + 99) / 100;
    std::nth_element(values.begin(), values.begin() + lo, values.end());
    low = values[lo];
    std::nth_element(values.begin(), values.begin() + hi, values.end());
    high = values[hi];
  }
  /**
   * @brief 將強度限制到 [0,1]，僅供顏色計算。
   * @param intensity 原始強度。
   * @return 常數範圍時為 0.5；無資料或無效強度時為 0.3；其餘為線性映射。
   *
   * @par English
   * Map intensity into [0,1] for color calculation only. Parameter intensity is the original
   * value. Returns 0.5 for a constant range, 0.3 for invalid values or missing statistics, and
   * otherwise a linear mapping.
   */
  float value(float intensity) const {
    if (!finiteCount || !std::isfinite(intensity))
      return .3f;
    if (!(high > low))
      return .5f;
    return float(
        std::clamp((double(intensity) - low) / (double(high) - low), 0., 1.));
  }
};
/**
 * @brief 將線性正規化強度轉為深藍、青綠、黃、紅的連續彩色調色盤。
 * @param normalized P1/P99 範圍內的線性位置；先截斷再採對數映射。
 * @return RGB 各分量介於零與一；非有限輸入使用缺值灰色。
 * @par English
 * Map linearly normalized intensity through logarithmic contrast to a continuous dark-blue,
 * teal, yellow, red palette. Clamp normalized before mapping. Return RGB components in [0,1];
 * nonfinite input uses neutral missing-value gray. Source intensities are never modified.
 */
inline std::array<float, 3> intensityColor(float normalized) {
  if (!std::isfinite(normalized))
    return {.4f, .4f, .4f};
  const float t = std::log1p(255.f * std::clamp(normalized, 0.f, 1.f)) /
                  std::log(256.f);
  constexpr std::array<std::array<float, 3>, 4> stops{{
      {.05f, .10f, .45f}, {0.f, .75f, .65f},
      {1.f, .90f, .15f}, {.90f, .10f, .05f}}};
  const float position = t * float(stops.size() - 1);
  const size_t index = std::min(size_t(position), stops.size() - 2);
  const float fraction = position - float(index);
  std::array<float, 3> rgb{};
  for (size_t channel = 0; channel < rgb.size(); ++channel)
    rgb[channel] = stops[index][channel] * (1 - fraction) +
                   stops[index + 1][channel] * fraction;
  return rgb;
}
} // namespace bag
// 中文：bag 命名空間。
