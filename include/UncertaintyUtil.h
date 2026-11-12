// Declaration-only preview. Function implementations are not included.
/*
 * @Author: Shizhuo Yu
 * @Date: 2025-07-31 15:53:22
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2025-12-13 17:56:44
 */
#ifndef UNCERTAINTYUTIL_H
#define UNCERTAINTYUTIL_H
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter.h"
#include "common.h"
#include "Tracking.h"
#include "LiDARIncludes/LidarTracking.h"
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <string>
#include <vector>

namespace UCA_SLAM
{

class KeyFrame;

struct LocalBALidarRhsSensitivityBlock
{

    int pose_index;
    Eigen::Matrix<double,6,9> M;
};

struct LocalBALidarRhsSensitivityRecord
{

    KeyFrame* source_kf;
    Eigen::Matrix<double,9,9> CovZ;
    std::vector<LocalBALidarRhsSensitivityBlock,
                Eigen::aligned_allocator<LocalBALidarRhsSensitivityBlock>> blocks;
};

struct LocalBAReducedPoseCovarianceResult
{
    bool valid;
    std::string reason;

    Eigen::MatrixXd P;

    Eigen::MatrixXd H_inv;

};

enum class LocalBAOutcome
{
    NotRun = 0,
    CompletedAndCommitted,
    AbortedNoCommit,
    RejectedByGuard,
    Failed
};

inline const char* LocalBAOutcomeName(const LocalBAOutcome outcome);

struct LocalBAPoseCovPackage
{
    bool valid;
    std::vector<KeyFrame*> ordered_kfs;
    Eigen::MatrixXd P_pose;

    Eigen::MatrixXd H_pose_inv;
    std::vector<LocalBALidarRhsSensitivityRecord,
                Eigen::aligned_allocator<LocalBALidarRhsSensitivityRecord>> lidar_rhs_records;
    std::string mode;

    bool audit_localba_called;
    bool audit_optimizer_returned;
    bool audit_commit_done;
    bool audit_pose_commit_done;
    bool audit_point_commit_done;
    bool audit_pose_cov_update_done;
    bool audit_point_cov_update_done;
    bool audit_stop_flag_at_entry;
    bool audit_stop_flag;
    bool audit_stop_before_opt;
    bool audit_stop_after_visual;
    bool audit_stop_after_joint;
    bool audit_stop_at_commit_barrier;
    bool audit_late_stop_after_complete;
    bool audit_late_stop_commit_allowed;
    bool audit_transactional_policy;
    bool audit_localba_completed;
    bool audit_joint_required;
    bool audit_joint_attempted;
    bool audit_joint_optimize_called;
    bool audit_joint_skipped_due_stop;
    bool audit_joint_effective_contract;
    bool audit_joint_effective;
    bool audit_legacy_lidar_active;
    bool audit_contract_lidar_active;
    bool audit_lidar_posterior_used;
    bool audit_lidar_posterior_write_done;
    bool audit_plane_writeback_allowed;
    bool audit_plane_update_requested;
    bool audit_plane_update_done;
    int audit_visual_opt_done;
    int audit_joint_opt_done;
    int audit_visual_iterations_requested;
    int audit_joint_iterations_requested;
    int audit_local_kfs;
    int audit_fixed_kfs;
    int audit_local_mps;
    int audit_visual_edges;
    int audit_source_depth_edges;
    int audit_lidar_edges;
    int audit_final_active_balm_edges;
    int audit_visual_erase_candidates;
    int audit_visual_erase_applied;
    int audit_mp_cov_full;
    int audit_mp_cov_cond;
    int audit_mp_cov_keep;
    int audit_mp_cov_default;
    double audit_total_ms;
    double audit_cancel_stage_elapsed_ms;
    double audit_map_lock_wait_ms;
    double audit_map_lock_hold_ms;
    std::string audit_abort_reason;
    std::string audit_status;
    std::string audit_cancel_stage;

    void Clear();
};

class MapPoint;
class Frame;
class UncertaintyUtil
{
public:
    UncertaintyUtil();

    static void CalculateMapPointCovDepth(const Frame &Frame, const int &i, Eigen::Matrix3d &cov);

    static void ComputeLidarMatrixH7(const vector<ptpl,Eigen::aligned_allocator<ptpl>> &ptpl_list, const vector<int> &inlierIndices, const vector<double> &weights, const M3D &Rcw, const V3D &tcw, const MD(6,6) &J_pose, MD(6,6) &H7);

    static void ComputePoseUncertaintyFromLidar(const PLV(3) &points, const PLV(3) &norms, const PLV(3) &centers, const vector<double> &ds, const vector<double> &weights, const PLM(3) &point_covs, const PLM(3) &norm_covs, const PLM(6) &plane_covs, const M3D &Rcw, const V3D &tcw, const MD(6,6) &J_pose, PLM(6) &term_covs);

    static void ComputePoseUncertaintyFromLidar(const PLV(3) &points, const PLV(3) &norms, const vector<double> &ds, const vector<double> &weights, const PLM(3) &point_covs, const PLM(3) &norm_covs, const M3D &Rcw, const V3D &tcw, const MD(6,6) &J_pose, PLM(6) &term_covs);

    static void ComputeOneUncertaintyFromPoint(const Frame &Frame, const MD(6,6) &J_pose, const cv::KeyPoint &kp, const V3D &point, const M3D &cov_point,
    MD(6,6) &cov_pose_point);

    static void ComputePointMatrixH4(const Frame &Frame, const vector<int> &inlierIndices, const vector<double> &weights, const MD(6,6) &J_pose, MD(6,6) &H4);
    static void ComputeUncertaintyFromPoints(const Frame &Frame, const vector<int> &inlierIndices, const vector<double> &weights, const MD(6,6) &J_pose, const MD(6,6) &H4, const MD(6,6) &H7, MD(6,6) &pose_uncertainty_point);

    static void ComputeUncertaintyFromLidar(const vector<ptpl,Eigen::aligned_allocator<ptpl>> &ptpl_list, const PLM(3) &all_point_covs, const vector<int> &inlierIndices, const vector<double> &weights, const M3D &Rcw, const V3D &tcw, const MD(6,6) &H4, const MD(6,6) &H7, const MD(6,6) &J_pose, MD(6,6) &pose_uncertainty_Lidar);

    static bool ComputeLocalBAPoseSandwichCovariance(
        const Eigen::MatrixXd& H_in,
        const Eigen::MatrixXd& S_in,
        LocalBAReducedPoseCovarianceResult& result);

};
}
#endif
