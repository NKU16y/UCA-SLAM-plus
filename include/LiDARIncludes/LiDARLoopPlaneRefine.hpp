// Declaration-only preview. Function implementations are not included.
/*
 * @Author: Shizhuo Yu
 * @Date: 2025-08-27 09:35:36
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2025-08-27 11:32:13
 */
#ifndef LIDARLOOPPLANEREFINE_H
#define LIDARLOOPPLANEREFINE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <common.h>
using namespace std;
struct KFPlaneLocal {

  V3D n;
  double d;
  int    N;
};
extern const double kDeg2Rad;
using KFPlaneLocalVec = std::vector<KFPlaneLocal,Eigen::aligned_allocator<KFPlaneLocal>>;

inline void MatchPlanesR(const KFPlaneLocalVec& A, const KFPlaneLocalVec& B, M3D& R, double cos_th, \
double d_th, std::vector<std::pair<int,int>>& pairs, int topK
);

inline void MatchPlanesRT( const KFPlaneLocalVec& A, const KFPlaneLocalVec& B, const M3D& R, const V3D& t, double cos_th, double d_th,
std::vector<std::pair<int,int>>& pairs,int topK );
inline bool RefineSE3FromLocalPlanes_Mixed(const KFPlaneLocalVec& A, const KFPlaneLocalVec& B,
MD(4,4)& T_BA, double& med_abs_res, int& n_match, int max_iter , double huber_delta ,
double cos_th , double d_th , double re_match_deg , double re_match_trans ,
int topK0 , int topK  );

inline MD(4,4) Relative_BA_from_Tkw(const MD(4,4)& Tkw_A,
                                    const MD(4,4)& Tkw_B);
#endif