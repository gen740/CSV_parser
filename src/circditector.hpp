#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <vector>

struct Data_2D {
  double t, x, y;
};

struct Data_Pol : public Data_2D {
  double rad();
  double r2();
  double r();
};

struct Circ_Pos : Data_2D {
  double r, error;
};

class Circ_Detector {
 public:
  Circ_Detector();

  double circ_fitting();
  bool filter(Data_Pol data);
  void loop(Data_Pol data);

 private:
  std::deque<Circ_Pos> pos;
  std::deque<Data_2D> vel;
  std::deque<Data_2D> acc;

  size_t QUE_SIZE = 30;

  std::vector<Data_Pol> buf;

  std::mutex data;
  std::vector<Data_Pol> filtered_data;

  double rad;
  double rad_prev;
  double rad_threshold = 0.04;
  double r2;
  double r2_prev;
  double r2_threshold = 800'000;

  double is_one_object = false;
  int cycle = 0;
  int cycle_prev = 0;

  double error_threshold = 10'000'000;
  double r_min = 20;
  double r_max = 120;
  int prev_detected = 100;
};
