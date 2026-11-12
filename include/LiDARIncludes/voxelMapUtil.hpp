// Declaration-only preview. Function implementations are not included.
#ifndef VOXEL_MAP_UTIL_HPP
#define VOXEL_MAP_UTIL_HPP
#include "common.h"
#ifndef UCA_SLAM_VOXEL_PLANE_EIGEN_SOLVER_H
#define UCA_SLAM_VOXEL_PLANE_EIGEN_SOLVER_H

#include <Eigen/Core>

namespace UCA_SLAM
{

struct VoxelPlaneEigenSystem
{

    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    int min_index;
    int mid_index;
    int max_index;
};

bool VoxelPlaneSelfAdjointEigenEnabled();

bool SolveVoxelPlaneEigenSystemWithMode(
    const Eigen::Matrix3d& covariance,
    bool use_self_adjoint,
    VoxelPlaneEigenSystem& result);

bool SolveVoxelPlaneEigenSystem(
    const Eigen::Matrix3d& covariance,
    VoxelPlaneEigenSystem& result);

}

#endif
#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <stdio.h>
#include <string>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <shared_mutex>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <vector>
#define HASH_P 116101
#define MAX_N 10000000000
extern const double kPlanarityRatioThresh;
int NextVoxelPlaneId();
using WMap = std::unordered_map<int, double>;
class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx , int64_t vy , int64_t vz );

  bool operator==(const VOXEL_LOC &other) const;
};

typedef struct ptpl {

  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  Eigen::Vector3d center;
  Eigen::Matrix<double, 6, 6> plane_cov;
  double d;
  int layer;

  int pid;
  VOXEL_LOC vox;
  int src_idx;

  double w_tr;
} ptpl;

typedef struct pointWithCov {

  Eigen::Vector3d point;
  Eigen::Vector3d point_world;
  Eigen::Matrix3d cov;

  bool persistent_source_valid;
  std::uint64_t persistent_source_id;
  unsigned long persistent_frame_id;
  unsigned long persistent_anchor_id;
  bool persistent_fixed;
  std::uint64_t persistent_source_version;
  Eigen::Vector3d persistent_point_anchor;
  Eigen::Matrix3d persistent_cov_anchor;
} pointWithCov;

using PVec = std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>>;
using PVecPtr = std::shared_ptr<std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>>>;

struct PersistentVoxelSourceMoments
{

  std::uint64_t source_id;
  unsigned long frame_id;
  unsigned long anchor_id;
  bool fixed;
  std::uint64_t source_version;
  int count;
  Eigen::Matrix3d sum_outer_anchor;
  Eigen::Vector3d sum_anchor;
  Eigen::Matrix<double,6,6> cov_P_anchor;
  Eigen::Matrix<double,6,6> cov_P_second_order_anchor;
  Eigen::Matrix<double,6,3> cov_Pv_anchor;
  Eigen::Matrix3d cov_v_anchor;
  int cov_count;
  Eigen::Vector3d minimum_anchor;
  Eigen::Vector3d maximum_anchor;
  bool bounds_valid;
  std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>>
      subdivision_points;
};

typedef struct clusterPointWithCov {

  Eigen::Vector3d point;
  Eigen::Matrix3d cov;
} clusterPointWithCov;

using ClusterPointCovList = std::vector<
    clusterPointWithCov,
    Eigen::aligned_allocator<clusterPointWithCov>
>;

using ClusterPointCovMap = std::unordered_map<
    int,
    ClusterPointCovList,
    std::hash<int>,
    std::equal_to<int>,
    Eigen::aligned_allocator<std::pair<const int, ClusterPointCovList>>
>;

typedef struct voxelPlane {

  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  Eigen::Vector3d y_normal;
  Eigen::Vector3d x_normal;
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 6, 6> plane_cov;
  float radius;
  float min_eigen_value;
  float mid_eigen_value;
  float max_eigen_value;
  float d;
  int points_size;

  bool is_plane;
  bool is_init;
  int id;

  bool is_update;
  int last_update_points_size;
  bool update_enable;
} voxelPlane;

namespace std {
template <> struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const;
};
}

#include "NativeVoxelRegistration.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include "../../Thirdparty/native_robin_map/include/tsl/robin_map.h"
namespace UCA_SLAM {namespace NativeVoxel {
struct Cell {
 std::int64_t x,y,z;
 bool operator==(const Cell& b) const;
};
struct CellHash {
 std::size_t operator()(const Cell& c) const;
};
inline Cell VoxelCell(const Eigen::Vector3d& p,double size);

using RootObservedSupport=std::unordered_map<Cell,Points,CellHash>;

class ObservedSupportIndex {
 struct Reference {

  const Eigen::Vector3d* data;
  std::size_t count;
  std::uintptr_t root;
  const Points* points;
  RootObservedSupport* owner;
  void Refresh();
 };
 struct References {
  Reference first;
  std::vector<Reference> extra;
  bool Empty() const;
  template<class F> void Each(F&& f) const;
  Reference* Find(std::uintptr_t root);
  void Add(const Reference& r);
  void Remove(std::uintptr_t root);
 };
 struct Span {const Eigen::Vector3d* data;std::size_t count;};
 struct Candidates {std::vector<Span> spans;};
 std::uint64_t geometry_revision_;
 Pose grid_from_map_;
 Eigen::Vector3d GridPoint(const Eigen::Vector3d& p) const;

 uca_native_tsl::robin_map<Cell,References,CellHash> lookup_;
 std::unordered_map<std::uintptr_t,std::vector<Cell>> root_cells_;
public:

 class QueryCache {
  friend class ObservedSupportIndex;
  const ObservedSupportIndex* owner_;
  std::uint64_t revision_;
  uca_native_tsl::robin_map<Cell,Candidates,CellHash> cells_;
 public:
  explicit QueryCache(std::size_t expected_cells);
 };

 std::uint64_t GeometryRevision() const;

 explicit ObservedSupportIndex(const Pose& grid_from_map);
 bool Append(std::uintptr_t root,RootObservedSupport& owned,const Eigen::Vector3d& p);
 bool Nearest(const Eigen::Vector3d& q,Eigen::Vector3d& target,double& distance_squared) const;
 bool NearestCached(const Eigen::Vector3d& q,Eigen::Vector3d& target,double& distance_squared,QueryCache& cache) const;
 void RemoveRoot(std::uintptr_t root);
 void Prune(const Eigen::Vector3d& origin,double radius);
 void Clear();
};
}}

class OctoTree {
public:

    UCA_SLAM::NativeVoxel::RootObservedSupport native_observed_support_;
    std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> temp_points_;
    std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> new_points_;
    voxelPlane *plane_ptr_;
    int max_layer_;
    int layer_;
    int octo_state_;
    OctoTree *leaves_[8];
    double voxel_center_[3];
    std::vector<int> layer_point_size_;
    float quater_length_;
    float planer_threshold_;
    int max_plane_update_threshold_;
    int update_size_threshold_;
    int all_points_num_;
    int new_points_num_;
    int max_points_size_;
    int max_cov_points_size_;
    bool init_octo_;
    bool update_cov_enable_;
    bool update_enable_;

    MD(6,6) cov_P_;
    MD(6,6) cov_P_second_order_;
    MD(6,3) cov_Pv_;
    M3D cov_v_;
    int cov_count_;
    using PersistentSourceMomentMap = std::unordered_map<
        std::uint64_t, PersistentVoxelSourceMoments,
        std::hash<std::uint64_t>, std::equal_to<std::uint64_t>,
        Eigen::aligned_allocator<std::pair<
            const std::uint64_t, PersistentVoxelSourceMoments>>>;
    PersistentSourceMomentMap persistent_source_moments_;
    using PersistentAnchorMomentMap = std::unordered_map<
        unsigned long, PersistentVoxelSourceMoments,
        std::hash<unsigned long>, std::equal_to<unsigned long>,
        Eigen::aligned_allocator<std::pair<
            const unsigned long, PersistentVoxelSourceMoments>>>;
    PersistentAnchorMomentMap persistent_anchor_moments_;
    std::unordered_map<unsigned long,
        std::unordered_set<std::uint64_t>> persistent_anchor_sources_;
    std::vector<std::uint64_t> persistent_source_order_;

    void AccumulatePersistentSourcePoint(const pointWithCov &pv);

    void RebuildPersistentSourceMoments(
        const std::vector<pointWithCov,
            Eigen::aligned_allocator<pointWithCov>> &points);

    void RemovePersistentSource(const std::uint64_t source_id);

    void RebuildPersistentAnchorMomentsFromSources();

    void AccumulatePersistentSourceAtTerminal(const pointWithCov &pv);

    struct TerminalRefreshState {

      voxelPlane plane;
      std::vector<pointWithCov,
          Eigen::aligned_allocator<pointWithCov>> retained_points;
      std::vector<pointWithCov,
          Eigen::aligned_allocator<pointWithCov>> persistent_points;
      MD(6,6) cov_P;
      MD(6,6) cov_P_second_order;
      MD(6,3) cov_Pv;
      M3D cov_v;
      int point_count;
      int cov_count;
      bool initialized;
      bool update_cov_enable;
      bool update_enable;
      bool preserve_persistent_sources;
    };

    OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
           int max_point_size, int max_cov_points_size, float planer_threshold);

    ~OctoTree();

    void init_plane(const std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> &points, voxelPlane *plane, const bool assign_new_id );

    void update_plane_covariance_from_moments(
        const Eigen::Vector3d& eigenvalues,
        const Eigen::Matrix3d& eigenvectors,
        voxelPlane* plane);

    void update_plane(const std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> &points, voxelPlane *plane);

    bool flush_pending_plane_update();

    void cut_octo_tree();

    void init_octo_tree();

    void UpdateOctoTree(const pointWithCov &pv);

    bool PrepareTerminalRefresh(
        const std::vector<pointWithCov,
            Eigen::aligned_allocator<pointWithCov>> &points,
        TerminalRefreshState &state,
        bool &requires_recut) const;

    bool PrepareTerminalMomentRefresh(
        const std::vector<pointWithCov,
            Eigen::aligned_allocator<pointWithCov>> &points,
        const M3D &sum_outer,
        const V3D &sum_point,
        const MD(6,6) &cov_P,
        const MD(6,6) &cov_P_second_order,
        const MD(6,3) &cov_Pv,
        const M3D &cov_v,
        const int cov_count,
        TerminalRefreshState &state,
        bool &requires_recut,
        const int aggregate_point_count ,
        const bool preserve_persistent_sources ) const;

    void ApplyTerminalRefresh(TerminalRefreshState &&state);

    void ClearPersistentTerminalState();
private:
    inline void AccmulateCovZ(const pointWithCov &pv);

    inline void AccmulateCovZFromVector(const std::vector<pointWithCov,Eigen::aligned_allocator<pointWithCov>> &pts);
public:
    inline void BuildCovZ(Eigen::Matrix<double,9,9>& CovZ) const;

};

inline void buildVoxelMap(const std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> &input_points,
                   const float voxel_size, const int max_layer,
                   const std::vector<int> &layer_point_size,
                   const int max_points_size, const int max_cov_points_size,
                   const float planer_threshold,
                   std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);

inline void clearVoxelMap(std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);

inline void updateVoxelMap(const std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> &input_points,
                    const float voxel_size, const int max_layer,
                    const std::vector<int> &layer_point_size,
                    const int max_points_size, const int max_cov_points_size,
                    const float planer_threshold,
                    std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);

inline void build_single_residual(const pointWithCov &pv, const OctoTree *current_octo,
                           const int current_layer, const int max_layer,
                           const double sigma_num, bool &is_sucess,
                           double &prob, ptpl &single_ptpl, const bool &forBA);

inline void BuildResidualListOMP(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                          const double voxel_size, const double sigma_num,
                          const int max_layer,
                          const std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>> &pv_list,
                          std::vector<ptpl,Eigen::aligned_allocator<ptpl>> &ptpl_list,
                          PLV(3) &non_match,
                          const bool &forBA,
                          int* effective_threads );

inline void calcBodyCovComponents(
    const V3D& pb,
    const float range_std,
    const float azimuth_std_deg,
    const float elevation_std_deg,
    Eigen::Matrix3d& cov,
    Eigen::Matrix3d* range_component ,
    Eigen::Matrix3d* azimuth_component ,
    Eigen::Matrix3d* elevation_component );

inline void calcBodyCov(const V3D &pb, const float range_inc,
                        const float degree_inc, Eigen::Matrix3d &cov,
                        const bool &bfusion);

inline void calcBodyCovFromSensorToTarget(
    const V3D& point_target,
    const MD(4,4)& sensor_to_target,
    const float range_inc,
    const float degree_inc,
    M3D& covariance_target);

inline void calcBodyCovComponentsFromSensorToTarget(
    const V3D& point_target,
    const MD(4,4)& sensor_to_target,
    const float range_std,
    const float azimuth_std_deg,
    const float elevation_std_deg,
    M3D& covariance_target,
    M3D* range_component_target ,
    M3D* azimuth_component_target ,
    M3D* elevation_component_target );

class ClusterCovZAccumulator
{
public:

    MD(6,6) cov_P_;
    MD(6,6) cov_P_second_order_;
    MD(6,3) cov_Pv_;
    M3D cov_v_;
    int cov_count_;

    ClusterCovZAccumulator();

    inline void clear();

    inline bool empty() const;
    inline int count() const;

    inline void AccumulatePointCov(const V3D& p, const M3D& cov);

    inline void BuildCovZ(
        Eigen::Matrix<double,9,9>& CovZ,
        const bool include_second_order ) const;

};

using CovZMap = std::unordered_map<
    int,
    ClusterCovZAccumulator,
    std::hash<int>,
    std::equal_to<int>,
    Eigen::aligned_allocator<std::pair<const int, ClusterCovZAccumulator>>
>;

class PointCluster
{
public:

    M3D P;
    V3D v;
    int N;

    PointCluster();

    void push(const V3D &vec);

    M3D cov();

    PointCluster & operator+=(const PointCluster &sigv);

    PointCluster & operator-=(const PointCluster &sigv);

};
using PCMap = std::unordered_map<
    int,
    PointCluster,
    std::hash<int>,
    std::equal_to<int>,
    Eigen::aligned_allocator<std::pair<const int, PointCluster>>
>;
#endif
