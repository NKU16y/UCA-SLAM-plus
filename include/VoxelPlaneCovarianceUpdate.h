// Declaration-only preview. Function implementations are not included.
#ifndef UCA_SLAM_VOXEL_PLANE_COVARIANCE_UPDATE_H
#define UCA_SLAM_VOXEL_PLANE_COVARIANCE_UPDATE_H

#include "LiDARIncludes/voxelMapUtil.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace UCA_SLAM
{

struct PostBAVoxelPlaneIdentity
{
    VOXEL_LOC root;
    std::uint64_t path_code;
    int depth;
    int plane_id;
};

struct PostBAVoxelPlaneSourceIdentity
{
    std::uint64_t factor_source_uid;
    std::uint64_t ledger_source_id;
    unsigned long anchor_id;
};

struct PostBAVoxelPlaneTarget
{
    int factor_id;
    PostBAVoxelPlaneIdentity identity;
    std::vector<PostBAVoxelPlaneSourceIdentity> sources;
};

struct PostBAVoxelPlanePoseBlock
{
    unsigned long anchor_id;
    int pose_index;
};

struct PostBAVoxelPlaneCovarianceRequest
{
    bool requested;
    bool joint_effective;
    unsigned long expected_map_version;
    std::uint64_t expected_ledger_generation;
    Eigen::MatrixXd pose_covariance;
    std::vector<PostBAVoxelPlanePoseBlock> pose_blocks;
    std::vector<PostBAVoxelPlaneTarget> targets;
};

struct PostBAVoxelPlaneCovarianceAudit
{
    bool requested;
    bool joint_effective;
    bool transaction_attempted;
    bool transaction_committed;
    bool pose_covariance_valid;
    bool measurement_covariance_valid;
    bool source_identity_valid;
    int committed_keyframes;
    int target_factors;
    int target_roots;
    int target_leaves;
    int target_sources;
    int refit_leaves;
    int covariance_updated_leaves;
    int skipped_leaves;
    int missing_sources;
    int invalid_pose_blocks;
    double measurement_covariance_trace;
    double pose_covariance_trace;
    double plane_covariance_trace_before;
    double plane_covariance_trace_after;
    unsigned long map_version_before;
    unsigned long map_version_after;
    unsigned long map_update_seq_before;
    unsigned long map_update_seq_after;
    std::uint64_t ledger_generation_before;
    std::uint64_t ledger_generation_after;
    std::string reason;
};

struct PostBAVoxelPlaneMomentSource
{

    std::uint64_t source_id;
    unsigned long anchor_id;
    int pose_index;
    M3D rotation_world_anchor;
    V3D translation_world_anchor;
    PointCluster moments_anchor;
};

using PostBAVoxelPlaneMomentSourceVector = std::vector<
    PostBAVoxelPlaneMomentSource,
    Eigen::aligned_allocator<PostBAVoxelPlaneMomentSource>>;

struct PostBAVoxelPlaneCovarianceResult
{

    bool valid;
    Eigen::Matrix<double,9,9> measurement_covariance;
    Eigen::Matrix<double,9,9> pose_covariance;
    Eigen::Matrix<double,9,9> total_covariance;
    MD(6,6) plane_covariance;
    int pose_sources;
    int pose_rank;
    double pose_condition;
    std::string reason;
};

struct PostBAVoxelPlaneLinearization
{

    V3D center;
    M3D eigenvectors;
    V3D eigenvalues;
    int point_count;
};

bool PropagateVoxelPlaneCovZAtFixedLinearization(
    const PointCluster& aggregate_world_moments,
    const Eigen::Matrix<double,9,9>& covariance,
    const PostBAVoxelPlaneLinearization& fixed_linearization,
    MD(6,6)& plane_covariance);

bool BuildPostBAVoxelPlaneCovariance(
    const PointCluster& aggregate_world_moments,
    const Eigen::Matrix<double,9,9>& measurement_covariance,
    const PostBAVoxelPlaneMomentSourceVector& sources,
    const Eigen::MatrixXd& pose_covariance,
    PostBAVoxelPlaneCovarianceResult& result,
    const PostBAVoxelPlaneLinearization* fixed_linearization );

}

#endif
