#include <iostream>
#include <unordered_map>
#include <vector>
#include <set>
int main() {
  std::unordered_map<int, std::set<int>> un_map;

  un_map[0] = {100, 101, 102, 103};
  un_map[1] = {204, 205, 206};
  un_map[2] = {307, 308, 309};

  int num = 0;
  for (auto outer_it = un_map.begin(); outer_it != un_map.end(); ++outer_it) {
    const int& sensorType1 = outer_it->first;
    const std::set<int>& set1 = outer_it->second;

    for (auto it_i = set1.begin(); it_i != set1.end(); ++it_i) {
      const int& id1 = *it_i;

      auto inner_it = outer_it;
      for (; inner_it != un_map.end(); ++inner_it) {
        const int& sensorType2 = inner_it->first;
        const std::set<int>& set2 = inner_it->second;

        auto j_start = (inner_it == outer_it) ? it_i : set2.begin();
        for (auto it_j = j_start; it_j != set2.end(); ++it_j) {
          const int& id2 = *it_j;

          // 输出不重复组合
          if (outer_it != inner_it || it_i != it_j) {
            std::cout << "(( " << static_cast<int>(sensorType1) << ", "
                      << id1 << "), "
                      << "( " << static_cast<int>(sensorType2) << ", "
                      << id2 << "))\n";
            ++num;
          }
        }
      }
    }
  }

  std::cout << "num: " << num << std::endl;
  return 0;
}