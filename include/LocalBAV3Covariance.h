// Declaration-only preview. Function implementations are not included.
#pragma once

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace UCA_SLAM
{

enum class LocalBAV3CovarianceFactorFamily
{
    VisualMono = 0,
    VisualStereo,
    DirectZ,
    DirectXYZ,
    LocalPlane,
    CrossZ,
    CrossXYZ,
    Count
};

constexpr int LocalBAV3CovarianceFactorFamilyCount();

const char* LocalBAV3CovarianceFactorFamilyName(
    LocalBAV3CovarianceFactorFamily family);

struct LocalBAV3CovarianceResidualRecord
{

    std::uint64_t factor_id;
    std::uint64_t point_id;
    int pose_index;
    int dimension;
    bool active;
    bool information_contract_expected;
    LocalBAV3CovarianceFactorFamily family;
    Eigen::Matrix<double,3,3> point_jacobian;
    Eigen::Matrix<double,3,6> pose_jacobian;

    Eigen::Matrix3d base_measurement_covariance;

    Eigen::Matrix3d base_information;

    Eigen::Matrix3d pre_robust_information;
    bool matrix_weighting_valid;

    Eigen::Matrix3d effective_information;
    double robust_scale;
    double allocation_scale;
};

using LocalBAV3CovarianceResidualRecordVector = std::vector<
    LocalBAV3CovarianceResidualRecord,
    Eigen::aligned_allocator<LocalBAV3CovarianceResidualRecord>>;

struct LocalBAV3CovariancePointGroup
{
    std::uint64_t point_id;
    LocalBAV3CovarianceResidualRecordVector factors;
};

struct LocalBAV3CovariancePointContribution
{

    std::uint64_t point_id;
    std::vector<int> active_pose_indices;
    Eigen::MatrixXd reduced_bread;
    Eigen::MatrixXd reduced_meat;
    Eigen::Matrix3d point_bread_inverse;
    Eigen::Matrix3d point_meat;
    Eigen::MatrixXd pose_point_bread;
    Eigen::MatrixXd pose_point_meat;
    Eigen::Matrix3d conditional_covariance;
    bool conditional_valid;
    bool valid;
};

struct LocalBAV3CovarianceAudit
{
    std::array<int, LocalBAV3CovarianceFactorFamilyCount()>
        active_by_family;
    std::array<int, LocalBAV3CovarianceFactorFamilyCount()>
        inactive_by_family;
    int point_groups;
    int point_groups_valid;
    int point_groups_invalid;
    int invalid_records;
    int duplicate_factor_ids;
    int information_contract_checked;
    int information_contract_mismatch;
    int base_covariance_information_mismatch;
    int balm_active;
    int balm_inactive;
    int balm_covariance_sources;
    double effective_information_relative_error_max;
    double base_covariance_information_relative_error_max;
    bool schur_dense_reference_checked;
    double schur_dense_reference_relative_error_max;
    double schur_dense_bread_relative_error_max;
    double schur_dense_meat_relative_error_max;
    double schur_dense_bread_absolute_error_max;
    double schur_dense_meat_absolute_error_max;
    double schur_dense_bread_reference_norm_at_max;
    double schur_dense_meat_reference_norm_at_max;
    std::uint64_t schur_dense_max_point_id;
    int schur_dense_max_point_factors;
    int schur_dense_max_point_active_poses;
    bool schur_dense_max_is_meat;
    int pose_dimension;
    int pose_rank;
    double pose_condition;
    bool pose_covariance_valid;
};

struct LocalBAV3CovarianceSchurResult
{

    Eigen::MatrixXd reduced_bread;
    Eigen::MatrixXd reduced_meat;
    std::vector<LocalBAV3CovariancePointContribution,
                Eigen::aligned_allocator<
                    LocalBAV3CovariancePointContribution>> points;
    LocalBAV3CovarianceAudit audit;
    bool valid;
    std::string reason;
};

struct LocalBAV3CovarianceSandwichResult
{

    Eigen::MatrixXd covariance;
    Eigen::MatrixXd bread_inverse;
    int rank;
    double condition;
    bool valid;
    std::string reason;
};

struct LocalBAV3PlaneJackknifeDecomposition
{

    Eigen::Matrix3d normal_covariance;
    Eigen::Matrix3d rotation_covariance;
    double normal_induced_variance;
    double offset_variance;
    double total_parameter_variance;
    int samples;
    bool valid;
    std::string reason;
};

struct LocalBAV3PoseModelErrorSample
{

    Eigen::Matrix<double,6,1> delta;
    Eigen::Matrix<double,6,1> innovation_variance;
};

using LocalBAV3PoseModelErrorSampleVector = std::vector<
    LocalBAV3PoseModelErrorSample,
    Eigen::aligned_allocator<LocalBAV3PoseModelErrorSample>>;

struct LocalBAV3PoseModelErrorCalibration
{

    Eigen::Matrix<double,6,1> added_variance;

    Eigen::Matrix<double,6,6> group_covariance;
    Eigen::Matrix<double,6,6> excess_correlation;
    Eigen::Matrix<double,6,1> lag1_correlation;
    Eigen::Matrix<double,6,1> effective_samples;
    std::array<bool,6> eligible;
    int samples;
    int eligible_axes;
    int positive_axes;
    bool valid;
    std::string reason;
};

bool EstimateLocalBAV3PoseModelErrorCovariance(
    const LocalBAV3PoseModelErrorSampleVector& history,
    LocalBAV3PoseModelErrorCalibration& result);

bool ApplyLocalBAV3GroupCorrelatedPoseModelErrorCovariance(
    const Eigen::MatrixXd& paper,
    const Eigen::Matrix<double,6,1>& added_variance,
    Eigen::MatrixXd& calibrated,
    std::string& reason);

bool ApplyLocalBAV3GroupCorrelatedPoseModelErrorCovariance(
    const Eigen::MatrixXd& paper,
    const Eigen::Matrix<double,6,6>& group_covariance,
    Eigen::MatrixXd& calibrated,
    std::string& reason);

bool BuildLocalBAV3SchurCovarianceSystem(
    const std::vector<LocalBAV3CovariancePointGroup>& point_groups,
    int pose_count,
    const Eigen::MatrixXd& pose_only_bread,
    const Eigen::MatrixXd& pose_only_meat,
    LocalBAV3CovarianceSchurResult& result,
    bool dense_reference_crosscheck ,
    bool compact_block_assembly );

bool SolveLocalBAV3CovarianceSandwich(
    const Eigen::MatrixXd& bread,
    const Eigen::MatrixXd& meat,
    LocalBAV3CovarianceSandwichResult& result);

using LocalBAV3PointCovarianceVector = std::vector<
    Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;

bool ValidateLocalBAV3StoredPointCovariance(const Eigen::Matrix3d& covariance);

bool BuildLocalBAV3CoupledPointCovariances(
    const LocalBAV3CovarianceSchurResult& system,
    const Eigen::MatrixXd& pose_covariance,
    const Eigen::MatrixXd& schur_inverse,
    LocalBAV3PointCovarianceVector& marginals,
    std::string& reason);

bool BuildLocalBAV3CovariancePriorUnion(
    const Eigen::Matrix3d& paper,
    const Eigen::Matrix3d& prior,
    Eigen::Matrix3d& result,
    int& contracted_directions,
    std::string& reason);

bool ConvertLocalBAV3LambdaModelVarianceToRawResidualVariance(
    double lambda,
    double lambda_variance,
    double& residual_variance);

bool ComputeLocalBAV3SourceJackknifeVariance(
    const std::vector<double>& leave_one_out_residuals,
    double& variance);

bool ComputeLocalBAV3AddedVarianceInformationRatio(
    double paper_variance,
    double added_variance,
    double& information_ratio);

bool AccumulateLocalBAV3IndependentScalarModelCovariance(
    const Eigen::MatrixXd& jacobian,
    double effective_weight,
    double model_variance,
    Eigen::MatrixXd& meat);

bool ComputeLocalBAV3PlaneJackknifeDecomposition(
    const Eigen::Matrix3d& full_scatter,
    const Eigen::Vector3d& full_normal,
    const Eigen::Vector3d& full_center,
    const std::vector<Eigen::Vector3d,
        Eigen::aligned_allocator<Eigen::Vector3d>>& leave_one_normals,
    const std::vector<Eigen::Vector3d,
        Eigen::aligned_allocator<Eigen::Vector3d>>& leave_one_centers,
    LocalBAV3PlaneJackknifeDecomposition& result);

bool ExtractLocalBAV3PlaneNormalNuisanceVariance(
    const LocalBAV3PlaneJackknifeDecomposition& decomposition,
    double& variance);

}

#ifndef UCA_SLAM_LOCAL_BA_V3_UNCERTAINTY_H
#define UCA_SLAM_LOCAL_BA_V3_UNCERTAINTY_H

#include <Eigen/Core>

namespace UCA_SLAM
{

bool BuildLocalBAV3MeasurementOnlyResidualInformation(
    const Eigen::MatrixXd& measurement_covariance,
    Eigen::MatrixXd& information);

bool BuildLocalBAV3ResidualInformation(
    const Eigen::MatrixXd& measurement_covariance,
    const Eigen::MatrixXd& point_jacobian,
    const Eigen::Matrix3d* frozen_point_covariance,
    Eigen::MatrixXd& information);

bool ComputeLocalBAV3FreshPointConditionalCovariance(
    const Eigen::MatrixXd& point_jacobian,
    const Eigen::MatrixXd& information,
    const Eigen::MatrixXd& measurement_covariance,
    Eigen::Matrix3d& covariance);

}

#endif

#include "common.h"
#include "PoseCovarianceMetadata.h"

#include <cstdint>
#include <string>

namespace UCA_SLAM
{

class KeyFrame;
class MapPoint;

struct LocalBAAnchorAllocation
{
    bool valid;
    bool direct_available;
    bool plane_available;
    double alpha_direct;
    double alpha_plane;
    double direct_normal_variance;
    double plane_variance;
    double log_quality_direct;
    double log_quality_plane;
};

bool BuildLocalBASourceMeasurementInformation(
    const M3D& source_covariance_xc,
    M3D& information);

LocalBAAnchorAllocation ComputeLocalBAAnchorAllocation(
    bool direct_available,
    const M3D& source_covariance_xc,
    bool plane_available,
    const V3D& plane_normal_xc,
    double plane_variance);

struct LocalBAMapPointInitialCovariance
{
    bool valid;
    bool source_pose_applied;
    M3D measurement;
    M3D source_pose;
    M3D total;
};

enum class MapPointBirthPath
{
    Unknown = 0,
    TrackingVisualDepthInitialization = 1,
    TrackingKeyFrameDepth = 2,
    LocalMappingCurrentDepth = 3,
    LocalMappingNeighborDepth = 4,
    LocalMappingTriangulated = 5,
    LocalMappingDepthCandidate = 6
};

const char* MapPointBirthPathName(MapPointBirthPath path);

struct MapPointBirthCovarianceSnapshot
{

    int version;
    MapPointBirthPath path;
    std::uint64_t source_generation;
    unsigned long source_kf_id;
    unsigned long source_frame_id;
    int source_keypoint_index;
    bool valid;
    bool source_measurement_valid;
    bool source_pose_covariance_valid;
    bool source_pose_applied;
    bool gauge_fixed_source_pose;
    bool covariance_components_separable;
    M3D source_measurement_covariance_xc;
    M3D measurement_covariance_xw;
    M3D source_pose_covariance_xw;
    M3D total_covariance_xw;
    PoseCovarianceMetadata source_pose_metadata;
    std::string fallback_reason;
};

struct MapPointBirthCovarianceAudit
{
    int attempted;
    int accepted;
    int rejected;
    int source_measurement_valid;
    int source_pose_covariance_valid;
    int source_pose_applied;
    int trusted_tracking_pose_v2;
    int gauge_fixed;
    int covariance_components_nonseparable;
    int fallback;
    double measurement_trace_sum;
    double source_pose_trace_sum;
    double total_trace_sum;
    std::string last_fallback_reason;

    void Add(const MapPointBirthCovarianceSnapshot& snapshot,
             bool was_accepted);
};

void PrintMapPointBirthCovarianceAudit(
    MapPointBirthPath path,
    unsigned long source_kf_id,
    const MapPointBirthCovarianceAudit& audit);

bool MapPointLifecycleAuditEnabled();

bool NoLaterObservationAttributionAuditEnabled();
void PrintMapPointLifecycleAudit(
    MapPoint* map_point,
    const char* event,
    const char* outcome,
    unsigned long current_kf_id,
    MapPointBirthPath path );

MapPointBirthCovarianceSnapshot BuildMapPointBirthCovarianceSnapshot(
    MapPointBirthPath path,
    unsigned long source_kf_id,
    unsigned long source_frame_id,
    int source_keypoint_index,
    const M3D& source_covariance_xc,
    const MD(4,4)& pose_tcw,
    const V3D& point_xw,
    bool source_pose_requested,
    const MD(6,6)* pose_covariance_tcw,
    const PoseCovarianceMetadata& source_pose_metadata,
    bool gauge_fixed_source_pose );

MapPointBirthCovarianceSnapshot
BuildMapPointBirthCovarianceSnapshotFromKeyFrame(
    MapPointBirthPath path,
    KeyFrame* source_keyframe,
    int source_keypoint_index,
    const M3D& source_covariance_xc,
    const V3D& point_xw,
    bool source_pose_requested,
    bool gauge_fixed_source_pose );

LocalBAMapPointInitialCovariance BuildLocalBAMapPointInitialCovariance(
    const M3D& source_covariance_xc,
    const MD(4,4)& pose_tcw,
    const V3D& point_xw,
    const MD(6,6)* pose_covariance_tcw);

}

