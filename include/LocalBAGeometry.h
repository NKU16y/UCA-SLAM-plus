// Declaration-only preview. Function implementations are not included.
#pragma once

#ifndef UCA_SLAM_LOCAL_BA_V3_ADAPTIVE_VOXEL_H
#define UCA_SLAM_LOCAL_BA_V3_ADAPTIVE_VOXEL_H

#include "common.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace UCA_SLAM
{

struct LocalBAV3AdaptiveVoxelPoint
{

    V3D world_point;
    std::size_t payload_index;
};

using LocalBAV3AdaptiveVoxelPointVector = std::vector<
    LocalBAV3AdaptiveVoxelPoint,
    Eigen::aligned_allocator<LocalBAV3AdaptiveVoxelPoint>>;

struct LocalBAV3AdaptiveVoxelConfig
{
    double root_voxel_size;
    int max_depth;
    int maximum_threads;
    int minimum_points;
    double maximum_planarity_ratio;
    double maximum_residual_rms;
    double maximum_residual_p90;
};

struct LocalBAV3AdaptiveVoxelLeaf
{

    std::array<std::int64_t, 3> root;
    std::uint64_t path_code;
    int depth;
    V3D normal;
    V3D center;
    double lambda0;
    double lambda1;
    double residual_rms;
    double residual_p90;
    bool persistent_topology;
    std::vector<std::size_t> payload_indices;
};

using LocalBAV3AdaptiveVoxelLeafVector = std::vector<
    LocalBAV3AdaptiveVoxelLeaf,
    Eigen::aligned_allocator<LocalBAV3AdaptiveVoxelLeaf>>;

struct LocalBAV3AdaptiveVoxelStats
{
    int input_points;
    int invalid_points;
    int root_voxels;
    int nodes_visited;
    int split_nodes;
    int planar_leaves;
    int rejected_too_few;
    int rejected_nonplanar_max_depth;
    int max_depth_reached;
    int threads_used;
};

bool BuildLocalBAV3AdaptiveVoxelLeaves(
    const LocalBAV3AdaptiveVoxelPointVector& points,
    const LocalBAV3AdaptiveVoxelConfig& config,
    LocalBAV3AdaptiveVoxelLeafVector& leaves,
    LocalBAV3AdaptiveVoxelStats& stats);

}

#endif

#include "common.h"

#include <cstdint>
#include <vector>

namespace UCA_SLAM
{

struct LocalPlaneSupportPoint
{

    V3D point_xc;
    M3D covariance_xc;
    std::uint64_t source_uid;
};

using LocalPlaneSupportVector = std::vector<
    LocalPlaneSupportPoint,
    Eigen::aligned_allocator<LocalPlaneSupportPoint>>;

struct LocalPlaneMeasurementSnapshot
{

    int version;
    bool valid;
    bool covariance_valid;
    int support_count;
    int covariance_source_count;
    unsigned long source_kf_id;
    unsigned long source_frame_id;
    int source_keypoint_index;
    std::uint64_t source_generation;
    V3D query_xc;
    V3D centroid_xc;
    V3D eigenvalues;
    V3D normal_xc;
    double d;
    double fit_rms;
    double planarity_ratio;
    MD(4,4) sensor_parameter_covariance;
    MD(4,4) fit_parameter_covariance;
    MD(4,4) total_parameter_covariance;
    double sensor_residual_variance;
    double fit_residual_variance;
    double total_residual_variance;
};

bool BuildLocalPlaneMeasurementSnapshot(
    const V3D& query_xc,
    const LocalPlaneSupportVector& support,
    std::uint64_t source_generation,
    LocalPlaneMeasurementSnapshot& snapshot);

bool EvaluateLocalPlaneResidualVariance(
    const LocalPlaneMeasurementSnapshot& snapshot,
    const V3D& query_xc,
    double& sensor_variance,
    double& fit_variance,
    double& total_variance);

}

