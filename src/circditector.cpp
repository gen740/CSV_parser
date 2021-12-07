#include "circditector.hpp"

#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>

double Data_Pol::rad() { return std::atan2(x, y); }  // atan2 を返す。
double Data_Pol::r2() { return x * x + y * y; }      // 中心からの距離の二乗を返す。
double Data_Pol::r() { return std::sqrt(r2()); }     // 中心からの距離を返す。

Circ_Detector::Circ_Detector() {}

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
  // データ数が三つ以下のときは、処理できない
  if (N < 3) {
    return -1;
  }
  // 平均を求める。
  for (auto&& i : filtered_data) {
    xbar += i.x / N;
    ybar += i.y / N;
  }
  // 二乗フィッティング
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
  r = std::sqrt(r);
  // エラーを求める 観測誤差
  double error = 0;
  for (auto&& i : filtered_data) {
    double dr = std::sqrt((i.x - x) * (i.x - x) + (i.y - y) * (i.y - y)) - r;
    dr *= dr;
    error += dr;
    error /= (N - 1);
  }
  if (r < r_min || r > r_max) {
    return -1;  // 検出に失敗
  }
  if (error > error_threshold) {
    return -1;  // 検出に失敗
  }
  double t = 0;  // 時間を平均にとる。
  for (auto&& i : filtered_data) {
    t += i.t / N;
  }
  // posに格納
  pos.push_front({{t, x, y}, r, error});
  if (pos.size() > QUE_SIZE) {
    pos.pop_back();
  }
  if (pos.size() >= 2) {
    double dt = pos.at(0).t - pos.at(1).t;
    vel.push_front({(pos.at(0).t + pos.at(1).t) / 2,   //
                    (pos.at(0).x - pos.at(1).x) / dt,  //
                    (pos.at(0).y - pos.at(1).y) / dt});
  }
  if (vel.size() > QUE_SIZE) {
    vel.pop_back();
  }
  if (vel.size() >= 2) {
    double dt = vel.at(0).t - vel.at(1).t;
    vel.push_front({(vel.at(0).t + vel.at(1).t) / 2,   //
                    (vel.at(0).x - vel.at(1).x) / dt,  //
                    (vel.at(0).y - vel.at(1).y) / dt});
  }
  if (pos.size() > QUE_SIZE) {
    pos.pop_back();
  }
  return error;  // 検出に成功
}

bool Circ_Detector::filter(Data_Pol data) {  //
  // 直前の位置から、フィルターをかけるようにする。
  return (data.x * data.x + data.y * data.y) < 30'000'000 && data.y > -1000  //
         && data.y < 10000 && data.x < 10000;
}

void Circ_Detector::loop(Data_Pol data) {
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
        buf = std::vector<Data_Pol>();
        buf.push_back(data);
      } else {
        buf.push_back(data);
      }
      is_one_object = true;
    } else {
      if (buf.size() > 0) {
        filtered_data = buf;
        if (circ_fitting() < 0) {
          prev_detected++;
        } else {
          prev_detected = 0;
        }
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
