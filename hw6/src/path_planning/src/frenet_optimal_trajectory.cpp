/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-07-18 22:26:04
 */

// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#include "frenet_optimal_trajectory.h"

#include "ros/ros.h"

namespace shenlan {
#define MAX_SPEED 50.0 / 3.6     // maximum speed [m/s]
#define MAX_ACCEL 2.0            // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0        // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0       // maximum road width [m]
#define D_ROAD_W 1.0             // road width sampling length [m]
#define DT 0.2                   // time tick [s]
#define MAXT 5.0                 // max prediction time [m]
#define MINT 4.0                 // min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6  // target speed [m/s]
#define D_T_S 5.0 / 3.6          // target speed sampling length [m/s]
#define N_S_SAMPLE 1             // sampling number of target speed
#define ROBOT_RADIUS 1.5         // robot radius [m]

#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

FrenetOptimalTrajectory::FrenetOptimalTrajectory() {}
FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {}

float FrenetOptimalTrajectory::sum_of_power(std::vector<float> value_list) {
  float sum = 0;
  for (float item : value_list) {
    sum += item * item;
  }
  return sum;
};

// 01 获取采样轨迹
Vec_Path FrenetOptimalTrajectory::calc_frenet_paths(float c_speed, float c_d,
                                                    float c_d_d, float c_d_dd,
                                                    float s0) {
  std::vector<FrenetPath> fp_list;
  //完成轨迹采样
  // 根据道路宽度进行采样,此处的道路宽度指采样的道路宽度
  for(float di=-1*MAX_ROAD_WIDTH; di<MAX_ROAD_WIDTH; di+=D_ROAD_W){
    // 横向动作规划,不同的目标Ti生成不同的曲线,然后选择曲线
    for(float Ti=MINT; Ti<MAXT; Ti+=DT){
      FrenetPath fp;
      // 计算出关于目标配置di、Ti的横向多项式
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      // Ti时间内得到每个点时刻位置、速度、加速度、加加速度
      for(float t=0; t<Ti; t+=DT){
        fp.t.push_back(t);
        // 获取采样时间内每一刻d的位置
        fp.d.push_back(lat_qp.calc_point(t));
        // 获取采样时间内每一刻d的速度
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        // 获取采样时间内每一刻d的加速度
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        // 获取采样时间内每一刻d的加加速度,后续计算cost使用
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }
      // 纵向速度规划(速度保持)
      for(float tv=TARGET_SPEED - D_T_S * N_S_SAMPLE;
          tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
          tv+=D_T_S){
        FrenetPath fp_bot = fp;
        // 计算出关于目标配置di、Ti的纵向多项式
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);
        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
        for(float t_:fp.t){
          // 获取采样时间内每一刻d的位置
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          // 获取采样时间内每一刻s的速度
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          // 获取采样时间内每一刻s的加速度
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          // 获取采样时间内每一刻s的加加速度,后续计算cost使用
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
          if(fp_bot.s_d.back() > fp_bot.max_speed){
            fp_bot.max_speed = fp_bot.s_d.back();
          }
          if(fp_bot.s_dd.back() > fp_bot.max_accel){
            fp_bot.max_accel = fp_bot.s_dd.back();
          }
        }

        float Jp = sum_of_power(fp.d_ddd); // d方向加加速度的总和
        float Js = sum_of_power(fp_bot.s_ddd); // s方向加加速度的总和
        float ds = std::pow((TARGET_SPEED - fp_bot.s_d.back()),2);// 目标速度与数组内的最后一个速度差值的平方

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);// 横向损失函数
        fp_bot.cv = KJ * Js + KT * Ti + KD * ds;// 纵向损失函数
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;// 总的损失函数为d和s方向的损失函数乘以对应的系数并相加

        fp_list.push_back(fp_bot);
      }
    }
  }
  return fp_list;
};

// 02
// 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数 Frenet to Cartesian
void FrenetOptimalTrajectory::calc_global_paths(Vec_Path& path_list,
                                                Spline2D csp){
    //计算采样轨迹的其他参数
    for (Vec_Path::iterator path_p=path_list.begin(); path_p!=path_list.end();path_p++){
    for(unsigned int i=0; i<path_p->s.size(); i++){
      if (path_p->s[i] >= csp.s.back()){
        break;
      }
      std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);
      float iyaw = csp.calc_yaw(path_p->s[i]);
      float di = path_p->d[i];
      float x = poi[0] + di * std::cos(iyaw + M_PI/2.0);
      float y = poi[1] + di * std::sin(iyaw + M_PI/2.0);
      path_p->x.push_back(x);
      path_p->y.push_back(y);
    }

    for(int i=0; i<path_p->x.size()-1; i++){
      float dx = path_p->x[i + 1] - path_p->x[i];
      float dy = path_p->y[i + 1] - path_p->y[i];
      path_p->yaw.push_back(std::atan2(dy, dx));
      path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
    }

    path_p->yaw.push_back(path_p->yaw.back());
    path_p->ds.push_back(path_p->ds.back());


    path_p->max_curvature = std::numeric_limits<float>::min();
    for(int i=0; i<path_p->x.size()-1; i++){
      path_p->c.push_back((path_p->yaw[i+1]-path_p->yaw[i])/path_p->ds[i]);
      if(path_p->c.back() > path_p->max_curvature){
        path_p->max_curvature = path_p->c.back();
      }
    }
  }
};

bool FrenetOptimalTrajectory::check_collision(FrenetPath path,
                                              const Vec_Poi ob) {
  for (auto point : ob) {
    for (unsigned int i = 0; i < path.x.size(); i++) {
      float dist = std::pow((path.x[i] - point[0]), 2) +
                   std::pow((path.y[i] - point[1]), 2);
      if (dist <= ROBOT_RADIUS * ROBOT_RADIUS) {
        return false;
      }
    }
  }
  return true;
};
// 03
// 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
Vec_Path FrenetOptimalTrajectory::check_paths(Vec_Path path_list,
                                              const Vec_Poi ob) {
  Vec_Path output_fp_list;
  //补全代码
  for(FrenetPath path:path_list){
    if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE && check_collision(path, ob)){
      output_fp_list.push_back(path);
    }
  }
  return output_fp_list;
};
// to-do step 1 finish frenet_optimal_planning
FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
    Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd,
    Vec_Poi ob) {
  // 01 获取采样轨迹数组
  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  // 02
  // 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
  calc_global_paths(fp_list, csp);

  // 03
  // 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
  Vec_Path save_paths = check_paths(fp_list, ob);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for (auto path : save_paths) {
    if (min_cost >= path.cf) {
      min_cost = path.cf;
      final_path = path;
    }
  }

  return final_path;
};

FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
    Spline2D csp, const FrenetInitialConditions& frenet_init_conditions,
    Vec_Poi ob) {
  float c_speed = frenet_init_conditions.c_speed;
  float c_d = frenet_init_conditions.c_d;
  float c_d_d = frenet_init_conditions.c_d_d;
  float c_d_dd = frenet_init_conditions.c_d_dd;
  float s0 = frenet_init_conditions.s0;

  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fp_list, csp);
  Vec_Path save_paths = check_paths(fp_list, ob);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for (auto path : save_paths) {
    if (min_cost >= path.cf) {
      min_cost = path.cf;
      final_path = path;
    }
  }
  return final_path;
}

}  // namespace shenlan
