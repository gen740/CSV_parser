#include "circditector.hpp"

#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>

double Data::rad() { return std::atan2(x, y); }
double Data::r2() { return x * x + y * y; }

double Circ_Detector::circ_fitting() {
  double T30 = 0;
  double T20 = 0;
  double T12 = 0;
  double T11 = 0;
  double T03 = 0;
  double T02 = 0;
  double T21 = 0;
  double x, y, X, Y, r;
  double xbar = 0;
  double ybar = 0;
  double N = filtered_data.size();
  // std::cout << N << std::endl;
  if (N < 3) {
    return -1;
  }
  // 平均を求める。
  for (auto&& i : filtered_data) {
    xbar += i.x / N;
    ybar += i.y / N;
  }

  for (auto&& i : filtered_data) {
    X = i.x - xbar;
    Y = i.y - ybar;
    T30 += std::pow(X, 3);
    T20 += std::pow(X, 2);
    T12 += X * std::pow(Y, 2);
    T03 += std::pow(Y, 3);
    T02 += std::pow(Y, 2);
    T21 += std::pow(X, 2) * Y;
    T11 += X * Y;
  }
  x = T02 * (T30 + T12) - T11 * (T03 + T21);
  x /= 2 * (T20 * T02 - T11 * T11);
  y = -T11 * (T30 + T12) + T20 * (T03 + T21);
  y /= 2 * (T20 * T02 - T11 * T11);
  r = x * x + y * y + (T20 + T02) / N;
  std::cout << x + xbar << " " << y + ybar << std::endl;
  // " r = " << std::sqrt(r) << std::endl;
  return 0;
}

bool Circ_Detector::filter(Data data) {  //
  // 直前の位置から、フィルターをかけるようにする。
  return (data.x * data.x + data.y * data.y) < 30'000'000 && data.y > -1000  //
         && data.y < 10000 && data.x < 10000;
}

void Circ_Detector::loop(Data data) {
  static int counter = 0;  // デバッグ用
  // データの更新
  rad = data.rad();
  r2 = data.r2();
  // フィルターして、バッファーにためる
  if (filter(data)) {
    if ((rad - rad_prev) < rad_threshold  // 前のデータから極端に離れていないなら
        && (r2 - r2_prev) < r2_threshold  //
        && cycle == cycle_prev) {         // サイクルが変わってないなら
      if (!is_one_object) {
        buf = std::vector<Data>();
        buf.push_back(data);
      } else {
        buf.push_back(data);
      }
      is_one_object = true;
    } else {
      if (buf.size() > 0) {
        filtered_data = buf;
        circ_fitting();
      }
      is_one_object = false;
    }
    cycle_prev = cycle;
  }
  // 何サイクル目かを更新
  if (rad - rad_prev > 2.0) {
    cycle++;
  }
  rad_prev = rad;
  r2_prev = r2;
  counter++;  // デバッグ用
}
