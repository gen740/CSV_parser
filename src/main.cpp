#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>

#include "circditector.hpp"
#include "csv.h"

auto main() -> int {
  io::CSVReader<3> in("../hoge.csv");
  double time, x, y;
  std::chrono::system_clock::time_point start, end;

  Circ_Detector detector;

  start = std::chrono::system_clock::now();
  while (in.read_row(time, x, y)) {
    detector.loop({time, x, y});
  }
  end = std::chrono::system_clock::now();
  std::cout << static_cast<double>(
                   std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() /
                   1000.0)
            << std::endl;
}
