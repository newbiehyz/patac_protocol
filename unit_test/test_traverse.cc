#include <iostream>
#include <unordered_map>
#include <vector>

int main() {
  std::unordered_map<int, std::vector<int>> un_map;

  un_map[0] = {100, 101, 102, 103};
  un_map[1] = {204, 205, 206};
  un_map[2] = {307, 308, 309};

  for (auto outer_it = un_map.begin(); outer_it != un_map.end(); ++outer_it) {
    const int& sensorType1 = outer_it->first;
    const std::vector<int>& vec1 = outer_it->second;

    for (size_t i = 0; i < vec1.size(); ++i) {
      const int& id1 = vec1[i];

      auto inner_it = outer_it;
      for (; inner_it != un_map.end(); ++inner_it) {
        const int& sensorType2 = inner_it->first;
        const std::vector<int>& vec2 = inner_it->second;

        size_t j_start = (inner_it == outer_it) ? i : 0;
        for (size_t j = j_start; j < vec2.size(); ++j) {
          const int& id2 = vec2[j];

          // 输出不重复组合
          if (outer_it != inner_it || i != j) {
            std::cout << "((Sensor " << static_cast<int>(sensorType1) << ", "
                      << id1 << "), "
                      << "(Sensor " << static_cast<int>(sensorType2) << ", "
                      << id2 << "))\n";
          }
        }
      }
    }
  }
}