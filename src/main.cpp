#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>

#include "circditector.hpp"
#include "csv.h"

auto main() -> int {
  io::CSVReader<3> in("../hoge.csv");
  double time, x, y;

  Circ_Detector detector;

  while (in.read_row(time, x, y)) {
    detector.loop({time, x, y});
  }
}
