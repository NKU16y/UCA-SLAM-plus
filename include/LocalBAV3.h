// Declaration-only preview. Function implementations are not included.
#ifndef UCA_SLAM_LOCAL_BA_V3_H
#define UCA_SLAM_LOCAL_BA_V3_H

#include "KeyFrame.h"
#include "LiDARIncludes/LidarVoxelSourceLedger.h"
#include "LiDARIncludes/voxelMapUtil.hpp"
#include "UncertaintyUtil.h"
#include "LocalBALifecycle.h"
#include "LocalBAGeometry.h"
#include "VoxelPlaneCovarianceUpdate.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <opencv2/core/core.hpp>

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace g2o
{
class EdgeLocalBAV3NormalizedLambda;
}

namespace UCA_SLAM
{

class Map;
class MapPoint;

struct LocalBAV3HistoricalPointSnapshot
{

    V3D world_point;
    M3D world_covariance;
};

using LocalBAV3HistoricalPointVector = std::vector<
    LocalBAV3HistoricalPointSnapshot,
    Eigen::aligned_allocator<LocalBAV3HistoricalPointSnapshot>>;

struct LocalBAV3HistoricalRootSnapshot
{

    VOXEL_LOC root;
    LocalBAV3HistoricalPointVector points;
};

struct LocalBAV3SharedVoxelLeafSnapshot
{

    VOXEL_LOC root;
    std::uint64_t path_code;
    int depth;
    V3D center;
    V3D normal;
    double lambda0;
    double lambda1;
    double residual_rms;
    int points;
    int plane_id;
};

using LocalBAV3SharedVoxelLeafVector = std::vector<
    LocalBAV3SharedVoxelLeafSnapshot,
    Eigen::aligned_allocator<LocalBAV3SharedVoxelLeafSnapshot>>;

using LocalBAV3HistoricalRootVector = std::vector<
    LocalBAV3HistoricalRootSnapshot,
    Eigen::aligned_allocator<LocalBAV3HistoricalRootSnapshot>>;

struct LocalBAV3LidarReferenceContributor
{
    KeyFrame* keyframe;
    unsigned long keyframe_id;
    cv::Mat pose_tcw;
    UcaLidarSourceDescriptor source;
};

struct LocalBAV3LidarPoseCommitUpdate
{
    KeyFrame* keyframe;
    unsigned long keyframe_id;
    cv::Mat tcw;
};

struct LocalBAV3LidarReference
{
    const std::unordered_map<VOXEL_LOC, OctoTree*>* voxel_map;
    double voxel_size;
    int max_layer;
    double ranging_cov;
    double angle_cov;
    std::size_t historical_keyframes;
    std::size_t historical_points;
    std::size_t historical_voxels;
    std::size_t historical_sketch_cache_hits;
    std::size_t historical_sketch_cache_misses;
    double build_ms;
    std::string status;
    std::vector<LocalBAV3LidarReferenceContributor> contributors;
    std::uint64_t deterministic_digest;

    std::shared_ptr<const LocalBAV3HistoricalRootVector> historical_roots;

    std::shared_ptr<const LocalBAV3SharedVoxelLeafVector>
        shared_voxel_leaves;

    std::shared_ptr<const LidarVoxelSourceLedger::LeafMomentVector>
        shared_voxel_leaf_moments;
    LidarVoxelSourceLedger::LeafMomentSnapshotStats
        shared_voxel_leaf_moment_stats;
    unsigned long shared_voxel_map_version;
    unsigned long shared_voxel_map_update_seq;
    std::uint64_t shared_voxel_ledger_generation;
    std::function<unsigned long()> read_current_voxel_map_version;
    std::function<bool(double&, double&, double&, int&)>
        read_current_configuration;

    std::function<bool(
        const std::vector<LocalBAV3LidarPoseCommitUpdate>&,
        const PostBAVoxelPlaneCovarianceRequest*,
        PostBAVoxelPlaneCovarianceAudit&,
        std::string&)> commit_source_ledger_pose_updates;

    bool Valid() const;

    bool NativeValid() const;

    bool SharedVoxelValid() const;

};

struct LocalBAV3Config
{
    bool enabled;

    bool balm_enabled;
    bool lidar_enabled;

    bool diagnostics_enabled;

    bool timing_enabled;
    bool summary_enabled;
    bool visual_measurement_only;

    bool factor_objective_v2;

    bool balm_source_profile_v2;

    bool postba_plane_covariance_v2;
    bool balm_optimize_enabled;

    bool balm_contribution_ablation;

    bool balm_native_only_contribution_ablation;

    bool balm_group_nis_covariance_calibration;

    bool balm_support_effective_sample_covariance;

    bool balm_support_cold_start_covariance;

    bool balm_joint_uncertainty_enabled;
    std::string balm_joint_solver;
    std::string balm_joint_objective;

    std::string balm_input_pose_covariance_mode;

    std::string balm_continuation_schedule;

    std::string final_refine_mode;
    bool l0_diag_enabled;

    bool pose_schur_family_audit_enabled;

    bool kf_marginal_shadow_enabled;

    bool pre_balm_information_audit_enabled;
    bool growth_diag_enabled;

    bool balm_paired_replay_diag_enabled;

    bool native_adaptive_support_required;

    bool native_adaptive_shadow_diag_enabled;

    bool defer_legacy_covariance_crosscheck;

    bool defer_covariance_v2_dense_reference_crosscheck;

    std::string uncertainty_update_mode;

    std::string covariance_update_v2_mode;

    std::string coupled_point_covariance_mode;

    bool covariance_v2_point_writeback_ablation;

    bool covariance_v2_point_prior_audit;

    bool covariance_v2_nonsource_prior_union;
    int visual_iterations;
    int joint_stage_iterations;
    int final_iterations;
    int association_threads;

    std::string source_depth_mode;
    std::string source_constraint_mode;
    std::string lidar_pipeline;
    double pseudo_stereo_depth_sigma_floor;
    double pseudo_stereo_right_sigma_floor;
    double pseudo_stereo_pixel_sigma2_scale;

    bool anchor_allocation;
    bool source_plane_audit;
    bool source_plane_shadow;
    bool source_plane_enable;
    int source_plane_audit_every;
    int source_plane_min_support;
    int source_plane_knn;
    double source_plane_radius;
    double source_plane_max_query_distance;
    double source_plane_max_fit_rms;
    double source_plane_max_planarity_ratio;
    double source_plane_min_incidence;
    double source_plane_max_initial_residual;
    double source_plane_sigma_floor;
    double source_plane_sigma_assoc;
    double source_plane_information_scale;
    double source_plane_information_min;
    double source_plane_information_max;
    int source_plane_max_per_kf;
    int source_plane_max_per_cell;

    bool source_plane_legacy_fallback;
    bool source_plane_prune_outlier;
    unsigned char visual_bad_streak;
    unsigned char visual_good_streak;
    unsigned int visual_probe_interval;

    bool PrincipledLidarPipeline() const;

    bool AdaptiveVoxelLidarPipeline() const;

    bool VoxelSlamNativeLidarPipeline() const;

    bool VoxelSlamSharedLidarPipeline() const;

    bool PostVisualLidarPipeline() const;

    bool CovarianceUpdateV2Enabled() const;

    bool CovarianceUpdateV2Writeback() const;

    static LocalBAV3Config FromEnvironment();
};

struct LocalBAV3G1CenteredPreparationAudit
{
    bool raw_member;
    int raw_covariance_source_count;
    int centered_covariance_source_count;
    std::string reason;
};

bool PrepareLocalBAV3G1CenteredMeasurement(
    g2o::EdgeLocalBAV3NormalizedLambda& edge,
    LocalBAV3G1CenteredPreparationAudit& audit);

bool LocalBAV3NativeAdaptiveSupportRequiredFromEnvironment();

struct LocalBAV3WindowRoleCandidate
{
    unsigned long kf_id;
    unsigned long frame_id;
    int temporal_rank;
    int covisibility_weight;
    int shared_anchor_mappoints;
    int branch_distance;
    bool legacy_covisible;
    bool valid;
};

struct LocalBAV3WindowPolicyConfig
{
    int recent_candidate_limit;
    int recent_core_limit;
    int minimum_recent_core;
    int minimum_recent_recall_percent;
    int recent_min_covisibility;
    int recent_min_shared_anchor_mappoints;
    int maximum_branch_distance;
    int same_branch_min_covisibility;
    int same_branch_min_shared_anchor_mappoints;
    int movable_neighbor_budget;
};

struct LocalBAV3WindowRoleSelection
{
    bool mixed_v1_effective;
    bool fallback_to_legacy;
    std::string reason;
    int recent_available;
    int recent_eligible;
    int recent_selected;
    int same_branch_selected;
    double recent_core_recall;
    std::vector<unsigned long> movable_neighbor_ids;
    std::vector<unsigned long> fixed_revisit_ids;
};

LocalBAV3WindowRoleSelection SelectLocalBAV3MixedWindowRoles(
    unsigned long anchor_id,
    const std::vector<LocalBAV3WindowRoleCandidate>& candidates,
    const LocalBAV3WindowPolicyConfig& config );

struct LocalBAV3ObservationSnapshot
{
    KeyFrame* kf;
    MapPoint* mp;
    unsigned long kf_id;
    unsigned long mp_id;
    std::size_t feature_index;
    cv::KeyPoint keypoint;
    float right_u;
    float inv_level_sigma2;
    float depth;
    double depth_variance;
    bool depth_variance_valid;
    bool reciprocal;
    bool visual_state_valid;
    UcaLocalBAVisualObsState visual_state;
};

struct LocalBAV3KeyFrameSnapshot
{

    KeyFrame* kf;
    unsigned long id;
    unsigned long frame_id;
    double timestamp;
    bool fixed;
    bool explicit_fixed_revisit;
    cv::Mat initial_tcw;
    g2o::SE3Quat initial_se3;
    bool initial_se3_valid;
    MD(6,6) initial_pose_covariance;
    bool initial_pose_covariance_valid;
    double fx;
    double fy;
    double cx;
    double cy;
    double bf;
};

struct LocalBAV3LidarPointSnapshot
{

    V3D body_point;
    M3D body_covariance;
    int birth_plane_id;

    std::size_t source_index;
    std::size_t logical_finite_index;
    std::size_t raw_source_index;
};

struct LocalBAV3LidarKeyFrameSnapshot
{

    unsigned long kf_id;
    KeyFrame* kf;
    std::string source;
    UcaLidarSourceDescriptor source_descriptor;
    std::size_t source_point_count;
    std::vector<
        LocalBAV3LidarPointSnapshot,
        Eigen::aligned_allocator<LocalBAV3LidarPointSnapshot>> points;
};

struct LocalBAV3LidarSamplingStats
{
    std::string mode;
    int eligible_kfs;
    std::size_t available_points;
    std::size_t selected_points;
    std::size_t selected_min;
    double selected_median;
    std::size_t selected_max;
    std::size_t anchor_available;
    std::size_t anchor_selected;
    int zero_selected_kfs;
    int max_points_per_kf;
    int max_points_total;
    std::size_t invalid_covariance_points;
    std::size_t covariance_cache_hits;
    std::size_t covariance_cache_misses;
    std::size_t source_generation_failures;
    double sampling_ms;
};

std::vector<std::size_t> ComputeLocalBAV3LidarPointQuotas(
    const std::vector<std::size_t>& available_points,
    std::size_t max_points_per_kf,
    std::size_t max_points_total);

struct LocalBAV3MapPointSnapshot
{

    MapPoint* mp;
    unsigned long id;
    cv::Mat initial_world_pos;
    cv::Mat initial_world_cov;
    V3D initial_world_pos_eigen;
    M3D initial_world_cov_eigen;
    bool initial_world_pos_valid;
    bool initial_world_cov_valid;
    bool initial_world_cov_floor_applied;
    bool triangulated;
    int observation_count;

    bool has_source_depth;
    KeyFrame* source_kf;
    unsigned long source_kf_id;
    cv::Mat source_xc;
    cv::Mat source_cov_xc;
    V3D source_xc_eigen;
    M3D source_cov_xc_eigen;
    bool source_xc_valid;
    bool source_cov_xc_valid;
    int source_feature_index;
    int source_type;

    struct SourcePlaneMeasurement
    {

        bool attempted;
        bool accepted;
        bool stored_measurement;
        int support_count;
        V3D normal;
        double d;
        V3D centroid;
        V3D eigenvalues;
        double planarity_ratio;
        double fit_rms;
        double initial_residual;
        double incidence;
        double variance;
        double information;
        bool propagated_covariance_valid;
        int covariance_source_count;
        double sensor_variance;
        double fit_variance;
        double numerical_floor_variance;
        double association_variance;
        std::shared_ptr<const LocalPlaneMeasurementSnapshot>
            propagated_measurement;
        bool allocation_valid;
        double alpha_direct;
        double alpha_plane;
        std::uint64_t source_generation;
        int cell_x;
        int cell_y;
        int cell_z;
    } source_plane;

    std::vector<LocalBAV3ObservationSnapshot> observations;
};

struct LocalBAV3KeyFrameLookup
{
    bool fixed;
    std::size_t index;
};

struct LocalBAV3Snapshot
{
    bool valid;
    std::string reason;
    unsigned long anchor_id;
    bool stop_at_entry;
    int reciprocal_mismatch_count;

    std::vector<
        LocalBAV3KeyFrameSnapshot,
        Eigen::aligned_allocator<LocalBAV3KeyFrameSnapshot>> local_kfs;
    std::vector<
        LocalBAV3KeyFrameSnapshot,
        Eigen::aligned_allocator<LocalBAV3KeyFrameSnapshot>> fixed_kfs;
    std::vector<
        LocalBAV3MapPointSnapshot,
        Eigen::aligned_allocator<LocalBAV3MapPointSnapshot>> local_mps;
    std::unordered_map<KeyFrame*, LocalBAV3KeyFrameLookup>
        keyframe_by_pointer;
    std::unordered_map<unsigned long, LocalBAV3KeyFrameLookup>
        keyframe_by_id;
    std::vector<
        LocalBAV3LidarKeyFrameSnapshot,
        Eigen::aligned_allocator<LocalBAV3LidarKeyFrameSnapshot>> local_lidar;
    bool kf_birth_association;
    LocalBAV3LidarSamplingStats lidar_sampling;
    bool has_lidar_reference_transaction;
    LocalBAV3LidarReference lidar_reference_transaction;
    double lock_wait_ms;
    double lock_hold_ms;
    double membership_ms;
    double observation_snapshot_ms;
    double lidar_snapshot_ms;
    double source_plane_snapshot_ms;
    bool source_plane_fit_deferred;
    int source_plane_cloud_snapshots;
};

struct LocalBAV3CommitPackage
{
    struct PoseUpdate
    {
        KeyFrame* kf;
        unsigned long id;
        cv::Mat tcw;
        cv::Mat covariance;
        bool covariance_valid;
    };

    struct PointUpdate
    {
        MapPoint* mp;
        unsigned long id;
        cv::Mat world_pos;
        cv::Mat covariance;
        bool covariance_valid;
    };

    struct ObservationStateUpdate
    {
        KeyFrame* kf;
        MapPoint* mp;
        unsigned long kf_id;
        unsigned long mp_id;
        std::size_t feature_index;
        UcaLocalBAVisualObsState state;
    };

    struct SourcePlaneClearUpdate
    {
        MapPoint* mp;
        unsigned long id;
    };

    PostBAVoxelPlaneCovarianceRequest postba_plane_covariance;

    bool ready;
    std::vector<PoseUpdate> poses;
    std::vector<PointUpdate> points;
    std::vector<ObservationStateUpdate> observation_states;
    std::vector<SourcePlaneClearUpdate> source_plane_clears;
};

struct LocalBAV3CommitTiming
{
    bool detailed;
    double map_lock_wait_ms;
    double map_lock_hold_ms;
    double pose_point_validation_ms;
    double observation_validation_ms;
    double pose_point_write_ms;
    double observation_state_write_ms;
    double update_normal_depth_ms;
    int observation_lookup_mps;
    int feature_lock_keyframes;
    int feature_lock_acquisitions;
    int observation_state_targets;
    int observation_state_writes;
    int observation_state_unchanged_skips;
    int observation_state_anchor_only_skips;
    int normal_depth_camera_center_cache_entries;
};

enum class LocalBAV3CommitStopDecision
{
    Proceed,
    ProceedAfterSoftStop,
    AbortSoftStop,
    AbortHardStop,
    AbortUnknownStop
};

class LocalBAV3
{
public:
    static LocalBAV3CommitStopDecision DecideCommitStop(
        bool raw_stop_flag,
        bool soft_stop_requested,
        bool hard_stop_requested,
        bool allow_soft_late_commit);

    static LocalBAOutcome Run(
        KeyFrame* anchor,
        const LocalBAV3CancellationToken* cancellation_token,
        Map* map,
        const std::vector<KeyFrame*>& neighbor_kfs,
        const std::vector<KeyFrame*>& explicit_fixed_kfs,
        LocalBAPoseCovPackage* pose_cov_package,
        const LocalBAV3LidarReference* lidar_reference ,
        std::uint64_t lifecycle_generation );

    static bool BuildSnapshot(
        KeyFrame* anchor,
        const LocalBAV3CancellationToken* cancellation_token,
        Map* map,
        const std::vector<KeyFrame*>& neighbor_kfs,
        const std::vector<KeyFrame*>& explicit_fixed_kfs,
        LocalBAV3Snapshot& snapshot,
        const LocalBAV3LidarReference* lidar_reference,
        const LocalBAV3Config& config);

    static bool Commit(
        const LocalBAV3Snapshot& snapshot,
        const LocalBAV3CommitPackage& commit,
        const LocalBAV3CancellationToken* cancellation_token,
        Map* map,
        int& map_write_count,
        std::string& reason,
        bool* soft_late_commit_used ,
        LocalBAV3CommitTiming* timing ,
        PostBAVoxelPlaneCovarianceAudit* postba_audit );
};

}

#endif
