// 4-connected BFS path-distance map for the frontier explorer (loaded via ctypes).
// out[y*w+x] = metres from seed through free cells, +inf if unreachable.
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

extern "C" int rosnav_path_distance(const uint8_t * free_cells, int h, int w, int sy, int sx,
                                    float resolution, float * out)
{
  if (!free_cells || !out || h <= 0 || w <= 0 || sy < 0 || sy >= h || sx < 0 || sx >= w) {
    return -1;
  }
  const float inf = std::numeric_limits<float>::infinity();
  const std::size_t n = static_cast<std::size_t>(h) * static_cast<std::size_t>(w);
  for (std::size_t i = 0; i < n; ++i) {out[i] = inf;}
  const std::size_t start = static_cast<std::size_t>(sy) * w + sx;
  if (!free_cells[start]) {return 0;}
  std::vector<int> q;
  q.reserve(n);
  q.push_back(static_cast<int>(start));
  out[start] = 0.0f;
  for (std::size_t head = 0; head < q.size(); ++head) {
    const int c = q[head];
    const int y = c / w, x = c % w;
    const float d = out[c] + resolution;
    const int nb[4] = {y > 0 ? c - w : -1, y < h - 1 ? c + w : -1, x > 0 ? c - 1 : -1,
      x < w - 1 ? c + 1 : -1};
    for (int k = 0; k < 4; ++k) {
      const int m = nb[k];
      if (m >= 0 && free_cells[m] && out[m] == inf) {
        out[m] = d;
        q.push_back(m);
      }
    }
  }
  return static_cast<int>(q.size());
}
