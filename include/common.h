// Declaration-only preview. Function implementations are not included.
/*
 * @Author: Shizhuo Yu
 * @Date: 2026-04-30 17:37:32
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2026-04-30 18:39:43
 */
#ifndef COMMON_H
#define COMMON_H
#include "PointTypes.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <LiDARIncludes/so3_math.h>
#include "sophus/so3.h"
#include "sophus/se3.h"

#define PLM(a) std::vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) std::vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>
#define PLVf(a) std::vector<Eigen::Matrix<float, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, a, 1>>>
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;
typedef Eigen::RowVector3d RV3D;
typedef Eigen::RowVector3f RV3F;

#define DIM_STATE (6)
#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define RVD(a) Eigen::Matrix<double, 1, (a)>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>
#define RVF(a) Eigen::Matrix<double, 1, (a)>;
#define INIT_COV (0.0000001)
#define PI_M (3.14159265358)
#define NUM_MATCH_POINTS (5)

class TicToc
{
 public:
    TicToc();

    void tic();

    double toc();

 private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

struct StatesGroup {
  StatesGroup();

  StatesGroup(const StatesGroup &b);

  StatesGroup &operator=(const StatesGroup &b);

  StatesGroup operator+(const Eigen::Matrix<double, DIM_STATE, 1> &state_add);

  StatesGroup &operator+=(const Eigen::Matrix<double, DIM_STATE, 1> &state_add);

  Eigen::Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b);

  void resetpose();

  M3D rot_end;
  V3D pos_end;
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> cov;
};

template <typename T> T deg2rad(T degrees);

inline VD(6) SophusLogToRotTrans(const Sophus::SE3& T);

inline Eigen::Matrix3d hat(const Eigen::Vector3d &v);

#endif
