#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <vector>

struct Data {
  double t, x, y;
  double rad();
  double r2();
};

class Circ_Detector {
 public:
  Circ_Detector() = default;

  double circ_fitting();
  bool filter(Data data);
  void loop(Data data);

 private:
  std::deque<Data> circ_pos;
  std::vector<Data> buf;

  std::mutex data;
  std::vector<Data> filtered_data;

  double rad;
  double rad_buf;
  double rad_prev;
  double rad_threshold = 0.05;
  double r2;
  double r2_buf;
  double r2_prev;
  double r2_threshold = 800'000;
  double is_one_object = false;
  int cycle = 0;
  int cycle_prev = 0;
};
