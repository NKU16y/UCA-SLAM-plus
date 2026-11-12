// Declaration-only preview. Function implementations are not included.
#ifndef UCA_SLAM_LIDAR_VOXEL_SOURCE_LEDGER_H
#define UCA_SLAM_LIDAR_VOXEL_SOURCE_LEDGER_H

#include "voxelMapUtil.hpp"
#include "VoxelPlaneCovarianceUpdate.h"

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace UCA_SLAM
{

class LidarVoxelSourceLedger
{
public:

    struct Parameters
    {
        double voxel_size;
        int max_layer;
        std::vector<int> layer_point_size;
        int max_points_size;
        int max_cov_points_size;
        double planar_threshold;
        bool topology_preserving_leaf_refresh;
        bool moment_first_leaf_refresh;
        int leaf_refresh_threads;
    };

    struct AnchorPose
    {

        unsigned long anchor_id;
        bool fixed;
        M3D rotation_world_anchor;
        V3D translation_world_anchor;
    };

    struct AnchorPoseUpdate
    {

        unsigned long anchor_id;
        M3D rotation_world_anchor;
        V3D translation_world_anchor;
    };

    struct PointMeasurementCovarianceComponents
    {

        M3D range_world;
        M3D angular_world;
        M3D other_world;
        bool valid;
    };

    using PointMeasurementCovarianceComponentVector = std::vector<
        PointMeasurementCovarianceComponents,
        Eigen::aligned_allocator<PointMeasurementCovarianceComponents>>;

    struct PlaneMeasurementCovarianceComponents
    {

        bool requested;
        bool valid;
        int plane_id;
        int support_points;
        MD(6,6) range_plane_covariance;
        MD(6,6) angular_plane_covariance;
        MD(6,6) other_plane_covariance;
        MD(6,6) kappa1_plane_covariance;
        double kappa1_relative_error;
        std::string reason;
    };

    struct CommitStats
    {
        std::size_t requested_anchors;
        std::size_t changed_anchors;
        std::size_t changed_sources;
        std::size_t touched_roots;
        std::size_t refreshed_roots;
        std::size_t refreshed_leaves;
        std::size_t structural_rebuild_roots;
        std::size_t moment_first_roots;
        std::size_t moment_first_leaves;
        std::size_t rebuilt_roots;
        std::size_t rebuilt_points;
        std::size_t incremental_leaf_roots;
        std::size_t incremental_leaves;
        std::size_t incremental_sources_removed;
        std::size_t incremental_sources_added;
        std::size_t incremental_points_moved;
        std::size_t incremental_recut_leaves;
        std::size_t incremental_fallbacks;
        std::string incremental_fallback_reason;
        std::size_t missing_anchors;
        double rebin_ms;
        double planning_ms;
        double publish_ms;
        double total_ms;
        PostBAVoxelPlaneCovarianceAudit postba_plane_covariance;
    };

    struct LeafSourceMoments
    {

        std::uint64_t source_id;
        unsigned long frame_id;
        unsigned long anchor_id;
        bool fixed;
        M3D rotation_world_anchor;
        V3D translation_world_anchor;
        PointCluster moments_anchor;
        ClusterCovZAccumulator covz_anchor;

        std::vector<clusterPointWithCov,
            Eigen::aligned_allocator<clusterPointWithCov>>
            boundary_reservoir_anchor;
        V3D minimum_anchor;
        V3D maximum_anchor;
        bool bounds_valid;
        std::uint64_t source_version;
    };

    using LeafSourceMomentVector = std::vector<
        LeafSourceMoments, Eigen::aligned_allocator<LeafSourceMoments>>;

    struct LeafMoments
    {

        VOXEL_LOC root;
        std::uint64_t path_code;
        int depth;
        int plane_id;
        LeafSourceMomentVector sources;
    };

    using LeafMomentVector = std::vector<
        LeafMoments, Eigen::aligned_allocator<LeafMoments>>;

    struct LeafMomentSnapshotStats
    {
        std::size_t requested_roots;
        std::size_t cache_hit_roots;
        std::size_t rebuilt_roots;
        std::size_t source_bins;
        std::size_t classified_points;
        std::size_t unmatched_points;
        std::size_t planar_leaves;
        double total_ms;
    };

    explicit LidarVoxelSourceLedger(std::size_t maximum_points_per_source );

    std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>>
    RegisterSource(
        std::uint64_t source_id,
        unsigned long frame_id,
        const AnchorPose& anchor,
        const std::vector<pointWithCov,
            Eigen::aligned_allocator<pointWithCov>>& world_points,
        const Parameters& parameters,
        std::string& reason,
        const PointMeasurementCovarianceComponentVector*
            measurement_components );

    bool ComputePlaneMeasurementCovarianceComponents(
        const VOXEL_LOC& root,
        int plane_id,
        const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        PlaneMeasurementCovarianceComponents& output) const;

    bool RegisterExistingSource(
        std::uint64_t source_id,
        unsigned long frame_id,
        const AnchorPose& anchor,
        const std::vector<pointWithCov,
            Eigen::aligned_allocator<pointWithCov>>& world_points,
        const Parameters& parameters,
        std::string& reason,
        const PointMeasurementCovarianceComponentVector*
            measurement_components );

    bool ApplyAnchorPoseUpdates(
        const std::vector<AnchorPoseUpdate,
            Eigen::aligned_allocator<AnchorPoseUpdate>>& updates,
        const Parameters& parameters,
        std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        CommitStats& stats,
        std::string& reason,
        const PostBAVoxelPlaneCovarianceRequest* postba_request );

    bool SnapshotLeafMoments(
        const std::vector<VOXEL_LOC>& requested_roots,
        const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        LeafMomentVector& output,
        LeafMomentSnapshotStats& stats,
        std::string& reason,
        const std::vector<unsigned long>* active_anchor_ids ,
        int minimum_points_per_anchor ,
        int minimum_total_active_points ,
        bool allow_single_active_with_fixed );

    bool SnapshotPhysicalLeafMoments(
        const std::vector<VOXEL_LOC>& requested_roots,
        const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        LeafMomentVector& output,
        LeafMomentSnapshotStats& stats,
        std::string& reason,
        const std::vector<unsigned long>* active_anchor_ids ,
        int minimum_points_per_anchor ,
        int minimum_total_active_points ,
        bool allow_single_active_with_fixed ) const;

    bool FreezeAnchor(
        unsigned long anchor_id, std::string& reason,
        std::unordered_map<VOXEL_LOC, OctoTree*>* voxel_map );

    void RemoveRoots(const std::unordered_set<VOXEL_LOC>& roots);

    void Clear();
    std::size_t SourceCount() const;
    std::size_t RootCount() const;
    std::size_t PointCount() const;
    std::uint64_t ContentGeneration() const;

    bool CheckInvariants(std::string& reason) const;

private:
    struct LocalPoint
    {

        V3D point_anchor;
        M3D covariance_anchor;
        std::size_t measurement_component_index;
    };

    struct PointMeasurementCovarianceComponentsAnchor
    {

        M3D range_anchor;
        M3D angular_anchor;
        M3D other_anchor;
    };

    using PointMeasurementCovarianceComponentsAnchorVector = std::vector<
        PointMeasurementCovarianceComponentsAnchor,
        Eigen::aligned_allocator<
            PointMeasurementCovarianceComponentsAnchor>>;

    using LocalPointVector = std::vector<
        LocalPoint, Eigen::aligned_allocator<LocalPoint>>;
    using SourceBins = std::unordered_map<VOXEL_LOC, LocalPointVector>;

    struct SourceRecord
    {

        std::uint64_t source_id;
        unsigned long frame_id;
        AnchorPose anchor;
        SourceBins bins;
        PointMeasurementCovarianceComponentsAnchorVector
            measurement_components;
        std::size_t points;
        std::uint64_t version;
    };

    using SourceMap = std::unordered_map<std::uint64_t, SourceRecord>;
    using RootSourceSet = std::unordered_set<std::uint64_t>;
    using RootSourceMap = std::unordered_map<VOXEL_LOC, RootSourceSet>;

    VOXEL_LOC RootForWorldPoint(const V3D& point,
                                const Parameters& parameters) const;
    void AddSourceToRootIndex(const SourceRecord& source);
    void RemoveSourceFromRootIndex(const SourceRecord& source);
    SourceRecord RebinSource(const SourceRecord& source,
                             const AnchorPose& new_anchor,
                             const Parameters& parameters) const;
    bool RebuildLeafMomentRoot(
        const VOXEL_LOC& root,
        const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        LeafMomentSnapshotStats& stats,
        std::string& reason);
    void MarkLeafMomentRootsDirty(const SourceRecord& source);

    std::size_t maximum_points_per_source_;
    std::size_t point_count_;
    std::uint64_t content_generation_;
    SourceMap sources_;
    RootSourceMap root_sources_;
    std::unordered_map<VOXEL_LOC, LeafMomentVector> leaf_moment_cache_;
    std::unordered_set<VOXEL_LOC> dirty_leaf_moment_roots_;
};

}

#endif
