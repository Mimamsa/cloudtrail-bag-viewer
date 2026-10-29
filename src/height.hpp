/**
 * @file height.hpp
 * @brief 不依賴相機的點雲座標 Z 範圍與高度正規化。
 * @par English
 * Camera-independent cloud-coordinate Z bounds and height normalization.
 */
#pragma once
#include <algorithm>
#include <cmath>
namespace bag {
/**
 * @brief 增量追蹤有限 Z 範圍；空集合或等高點使用色帶中點。
 * @par English
 * Incrementally track finite Z bounds; empty or flat clouds use the palette midpoint.
 */
struct HeightScale {
  double minimum = 0; ///< 最低座標 Z。 EN: Minimum cloud-coordinate Z.
  double maximum = 0; ///< 最高座標 Z。 EN: Maximum cloud-coordinate Z.
  bool valid = false; ///< 是否有有限高度。 EN: Whether a finite height has been added.
  /**
   * @brief 加入原始座標高度，不接受相機變換後的座標。
   * @param z 點雲座標系的高度；忽略非有限值。
   * @par English
   * Add a cloud-coordinate height, never a camera-transformed value. Nonfinite z is ignored.
   */
  void add(double z) {
    if (!std::isfinite(z)) return;
    if (!valid) { minimum = maximum = z; valid = true; }
    else { minimum = std::min(minimum, z); maximum = std::max(maximum, z); }
  }
  /**
   * @brief 將高度線性映射至 [0,1]；低於或高於範圍時截斷。
   * @param z 點雲座標系的 Z 值。
   * @return 正規化色帶位置；空集合、等高或非有限輸入為 0.5。
   * @par English
   * Linearly normalize cloud-coordinate z to [0,1], clamping out-of-range values.
   * Returns 0.5 for empty/flat ranges or nonfinite input.
   */
  float value(double z) const {
    if (!valid || maximum == minimum || !std::isfinite(z)) return .5f;
    return float(std::clamp((z - minimum) / (maximum - minimum), 0., 1.));
  }
};
/**
 * @brief 高度上色固定使用點雲座標 Z = -30 至 10 公尺；超出範圍使用端點顏色。
 * @par English
 * Fixed cloud-coordinate height coloring range, Z = -30 to 10 meters. Values outside this
 * range use endpoint colors. Independent of frames, accumulation, and camera transformations.
 */
inline constexpr HeightScale heightColorScale{-30, 10, true};
} // namespace bag
