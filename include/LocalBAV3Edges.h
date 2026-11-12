// Declaration-only preview. Function implementations are not included.
#pragma once

#ifndef UCA_SLAM_LOCAL_BA_V3_EDGES_H
#define UCA_SLAM_LOCAL_BA_V3_EDGES_H

#include "LiDARIncludes/voxelMapUtil.hpp"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace g2o
{

struct LocalBAV3BalmPoint
{

    V3D body_point;
    M3D body_covariance;
};

using LocalBAV3BalmPointVector = std::vector<
    LocalBAV3BalmPoint,
    Eigen::aligned_allocator<LocalBAV3BalmPoint>>;

struct LocalBAV3BalmSourceSlice
{

    unsigned long long source_uid;

    std::uint64_t physical_source_id;
    int pid;
    bool has_vox;
    VOXEL_LOC vox;
    int layer;
    PointCluster moments;
    ClusterCovZAccumulator covz;
    Eigen::Matrix<double,9,9> centered_covz_cache;
    bool centered_covz_cache_valid;
};

using LocalBAV3BalmSourceSliceVector = std::vector<
    LocalBAV3BalmSourceSlice,
    Eigen::aligned_allocator<LocalBAV3BalmSourceSlice>>;

struct LocalBAV3BalmCluster
{

    unsigned long keyframe_id;
    PointCluster moments;
    LocalBAV3BalmPointVector points;
    ClusterCovZAccumulator covz;
    LocalBAV3BalmSourceSliceVector sources;
};

enum class LocalBAV3BalmObjectiveMode
{
    NormalizedExcess = 0,
    NativeLambdaGn,
    ClusterCovariance
};

enum class LocalBAV3BalmClusterResidualModel
{
    RawSqrtLambda = 0,
    CenteredLambda
};

enum class LocalBAV3BalmQuadraticFormMode
{
    Legacy = 0,
    CustomGnParity = 1,
    ExactIrls = 2
};

struct LocalBAV3BalmCovarianceSource
{

    unsigned long keyframe_id;
    bool fixed_reference;
    unsigned long long source_uid;
    int pid;
    bool has_vox;
    VOXEL_LOC vox;
    int layer;
    PointCluster moments;
    Eigen::Matrix<double,9,9> CovZ;
    Eigen::Matrix<double,1,9> Jres_z;
    double expected_lambda;
};

using LocalBAV3BalmCovarianceSourceVector = std::vector<
    LocalBAV3BalmCovarianceSource,
    Eigen::aligned_allocator<LocalBAV3BalmCovarianceSource>>;

struct LocalBAV3BalmFixedMoments
{

    double N;
    V3D v;
    M3D P;
    double point_variance;
    int pid_count;
    unsigned long long source_uid;
    int pid;
    bool has_vox;
    VOXEL_LOC vox;
    int layer;
    ClusterCovZAccumulator covz;
};

using LocalBAV3PoseJacobian = Eigen::Matrix<double,1,6>;
using LocalBAV3PoseJacobianVector = std::vector<
    LocalBAV3PoseJacobian,
    Eigen::aligned_allocator<LocalBAV3PoseJacobian>>;

struct LocalBAV3BalmSecondOrderSystem
{

    double objective;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd hessian;
};

class EdgeLocalBAV3NormalizedLambda
    : public BaseMultiEdge<1, double>
{
public:

    EdgeLocalBAV3NormalizedLambda();

    bool read(std::istream& input) override;
    bool write(std::ostream& output) const override;
    void computeError() override;
    void linearizeOplus() override;
    void constructQuadraticForm() override;

    bool QuadraticFormFailed() const;
    const std::string& QuadraticFormFailureReason() const;

    static LocalBAV3BalmQuadraticFormMode QuadraticFormModeFromEnvironment();
    static const char* QuadraticFormModeName(
        LocalBAV3BalmQuadraticFormMode mode);

    static bool BalmHuberEnabledFromEnvironment();
    void SetClusters(const std::vector<
        LocalBAV3BalmCluster,
        Eigen::aligned_allocator<LocalBAV3BalmCluster>>& clusters);
    void SetFixedMoments(const LocalBAV3BalmFixedMoments& fixed_moments);
    void SetContinuationScale(double scale);
    void SetPoseJacobianTranslationOnly(bool enabled);

    void SetObjectiveMode(LocalBAV3BalmObjectiveMode mode);
    void SetClusterResidualModel(
        LocalBAV3BalmClusterResidualModel model);
    void SetModelParameters(double smooth_epsilon,
                            double model_sigma_lambda,
                            double correlation_inflation);
    void SetAssociationModelVariance(double variance);

    bool PrepareStage();
    bool PrepareMeasurementModel();

    double Lambda() const;
    double MuLambda() const;
    double LambdaVariance() const;

    double StandardizedExcess() const;
    double EffectivePointCount() const;
    double ContinuationScale() const;
    double PositiveDerivative() const;
    const V3D& StageNormal() const;

    bool UsesClusterCovariance() const;
    double MeasurementVariance() const;
    double PropagatedMeasurementVariance() const;
    double MeasurementMeanLambda() const;
    double IndependentModelVariance() const;
    double CorrelationInflation() const;
    double ModelSigmaLambda() const;
    double AssociationModelVariance() const;
    double SmoothEpsilon() const;
    double MeasurementInformation() const;
    int CovarianceSourceCount() const;
    bool ComputeCovarianceSources(
        LocalBAV3BalmCovarianceSourceVector& sources) const;

    bool ComputePoseJacobiansReadOnly(
        LocalBAV3PoseJacobianVector& jacobians) const;

    bool ComputeErrorAndPoseJacobians(
        LocalBAV3PoseJacobianVector& jacobians);

    bool ComputeVoxelSlamLambdaSecondOrder(
        LocalBAV3BalmSecondOrderSystem& system) const;
    bool ComputeVoxelSlamLambda(double& lambda) const;

    bool ComputeVoxelSlamObjective(double& objective) const;
    bool UsesCenteredClusterCovariance() const;
    bool SupportsNisGate() const;
    double SourceNoiseJacobianScale() const;
    const char* ClusterResidualModelName() const;
    const std::vector<
        LocalBAV3BalmCluster,
        Eigen::aligned_allocator<LocalBAV3BalmCluster>>& Clusters() const;
    const LocalBAV3BalmFixedMoments& FixedMoments() const;
    LocalBAV3BalmObjectiveMode ObjectiveMode() const;
    LocalBAV3BalmClusterResidualModel ClusterResidualModel() const;
    Eigen::Matrix<double,1,6> Jacobian(
        std::size_t index) const;

private:
    friend struct LocalBAQuadraticFormTestAccess;
    bool ComputeDenseLegacyQuadraticForm(
        Eigen::MatrixXd& hessian,
        Eigen::VectorXd& b,
        bool blockwise,
        std::string& reason) const;
    bool ComputeDenseExactIrlsQuadraticForm(
        Eigen::MatrixXd& hessian,
        Eigen::VectorXd& b,
        std::string& reason) const;
    bool ComputeExactIrlsComponents(
        LocalBAV3BalmSecondOrderSystem& raw,
        std::vector<MD(6,6), Eigen::aligned_allocator<MD(6,6)>>&
            voxel_from_g2o,
        std::vector<M3D, Eigen::aligned_allocator<M3D>>& chart_mixed_terms,
        Eigen::VectorXd& b,
        double& exact_weight,
        std::string& reason) const;
    bool ScatterExactIrlsQuadraticForm(std::string& reason);

    void UpdateErrorFromLambda(double lambda);
    bool ComputePoseJacobiansFromMoments(
        const MD(4,4)& moments,
        const V3D& center,
        const V3D& normal,
        double lambda,
        LocalBAV3PoseJacobianVector& jacobians) const;
    bool AssembleMoments(MD(4,4)& moments,
                         M3D& scatter,
                         V3D& center,
                         V3D& normal,
                         double& lambda) const;
    static MD(4,4) ClusterMomentMatrix(const PointCluster& cluster);
    static MD(4,4) ToMatrix4d(const SE3Quat& transform);
    static MD(4,4) CovZBasisMatrix(int index);
    static double DlambdaFromMomentPerturbation(
        const MD(4,4)& perturbation,
        const M3D& second_moment,
        const V3D& first_moment,
        double point_count,
        const V3D& center,
        const V3D& normal);

    std::vector<
        LocalBAV3BalmCluster,
        Eigen::aligned_allocator<LocalBAV3BalmCluster>> clusters_;

    bool stage_valid_;
    bool quadratic_form_failed_;
    std::string quadratic_form_failure_reason_;
    bool pose_jacobian_translation_only_;
    double continuation_scale_;
    double smooth_epsilon_;
    double model_sigma_lambda_;
    double correlation_inflation_;
    double lambda_;
    double mu_lambda_;
    double lambda_variance_;
    double denominator_;
    double effective_point_count_;
    double positive_derivative_;
    V3D stage_normal_;
    LocalBAV3BalmFixedMoments fixed_moments_;
    LocalBAV3BalmObjectiveMode objective_mode_;
    LocalBAV3BalmClusterResidualModel cluster_residual_model_;
    double measurement_variance_;
    double propagated_measurement_variance_;
    double measurement_mean_lambda_;
    double independent_model_variance_;
    double association_model_variance_;
    double measurement_information_;
    int covariance_source_count_;
    Eigen::Matrix<double,9,9> fixed_centered_covz_cache_;
    bool fixed_centered_covz_cache_valid_;
    mutable MD(4,4) measurement_moments_cache_;
    mutable M3D measurement_scatter_cache_;
    mutable V3D measurement_center_cache_;
    mutable V3D measurement_normal_cache_;
    mutable double measurement_lambda_cache_;
    mutable bool measurement_geometry_cache_valid_;
};

enum class LocalBAV3BalmJointObjectiveMode
{
    CenteredCorrelated,
    NativeLambdaWeighted
};

struct LocalBAV3BalmJointMeasurementModel
{

    std::vector<EdgeLocalBAV3NormalizedLambda*> edges;
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd information;
    LocalBAV3BalmJointObjectiveMode objective_mode;

    Eigen::VectorXd native_lambda_weights;
    int unique_source_count;
    double maximum_absolute_correlation;
    double covariance_condition;
    double maximum_diagonal_relative_error;
    bool valid;
};

struct LocalBAV3BalmJointDerivativeInput
{

    Eigen::VectorXd lambda;
    Eigen::MatrixXd lambda_jacobian;
    std::vector<Eigen::MatrixXd> lambda_hessians;
    Eigen::VectorXd smooth_epsilon;
    double continuation_scale;
};

struct LocalBAV3BalmJointSecondOrderSystem
{

    double objective;
    Eigen::VectorXd residual;
    Eigen::VectorXd residual_derivative;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd hessian;

    Eigen::MatrixXd gn_hessian;

    Eigen::MatrixXd score_covariance;
};

bool BuildLocalBAV3BalmJointMeasurementModel(
    const std::vector<EdgeLocalBAV3NormalizedLambda*>& edges,
    LocalBAV3BalmJointMeasurementModel& model,
    std::string& reason);

bool ConfigureLocalBAV3BalmJointObjective(
    LocalBAV3BalmJointMeasurementModel& model,
    LocalBAV3BalmJointObjectiveMode mode,
    std::string& reason);

bool InflateLocalBAV3BalmJointMeasurementCovariance(
    const LocalBAV3BalmJointMeasurementModel& base_model,
    const Eigen::MatrixXd& lambda_jacobian,
    const Eigen::MatrixXd& pose_covariance,
    LocalBAV3BalmJointMeasurementModel& inflated_model,
    double& covariance_trace_ratio,
    std::string& reason);

bool ScaleLocalBAV3BalmJointMeasurementCovariance(
    const LocalBAV3BalmJointMeasurementModel& base_model,
    const Eigen::VectorXd& standard_deviation_scale,
    LocalBAV3BalmJointMeasurementModel& scaled_model,
    double& covariance_trace_ratio,
    std::string& reason);

bool ComputeLocalBAV3BalmJointSecondOrder(
    const LocalBAV3BalmJointMeasurementModel& model,
    const LocalBAV3BalmJointDerivativeInput& input,
    bool build_system,
    LocalBAV3BalmJointSecondOrderSystem& system,
    std::string& reason);

class EdgeLocalBAV3JointUncertaintyGn
    : public BaseMultiEdge<-1, Eigen::VectorXd>
{
public:

    EdgeLocalBAV3JointUncertaintyGn();

    bool read(std::istream& input) override;
    bool write(std::ostream& output) const override;
    void computeError() override;
    void linearizeOplus() override;
    void linearizeOplus(JacobianWorkspace& workspace) override;

    bool Configure(
        const LocalBAV3BalmJointMeasurementModel& model,
        std::string& reason);
    bool Valid() const;
    Eigen::MatrixXd Jacobian(std::size_t index) const;

private:
    bool EvaluateResidual(
        Eigen::VectorXd& residual,
        Eigen::VectorXd* derivative,
        std::string& reason) const;

    LocalBAV3BalmJointMeasurementModel model_;
    std::unordered_map<const OptimizableGraph::Vertex*, int>
        vertex_index_;
    bool valid_;
};

}

#endif

#ifndef UCA_SLAM_LOCAL_BA_V3_BALM_SOURCE_INFO_H
#define UCA_SLAM_LOCAL_BA_V3_BALM_SOURCE_INFO_H

#include <cstdint>
#include <string>
#include <vector>

namespace UCA_SLAM
{

struct LocalBAV3BalmSourceProvenance
{
    std::uint64_t source_uid;
    std::uint64_t content_digest;
    bool fixed;
};

struct LocalBAV3BalmFactorProvenance
{
    int factor_id;
    std::uint64_t content_digest;
    std::size_t cluster_count;
    std::size_t point_count;
    std::size_t movable_source_count;
    std::size_t fixed_source_count;
    std::size_t invalid_source_count;
    std::size_t geometry_source_mismatch;
    std::size_t covariance_source_mismatch;
    std::vector<LocalBAV3BalmSourceProvenance> sources;
    bool valid;
};

struct LocalBAV3BalmSourceAudit
{
    std::size_t factor_count;
    std::size_t cluster_count;
    std::size_t point_count;
    std::size_t movable_source_count;
    std::size_t fixed_source_count;
    std::size_t unique_source_count;
    std::size_t invalid_source_count;
    std::size_t source_uid_conflicts;
    std::size_t geometry_source_mismatch;
    std::size_t covariance_source_mismatch;
    std::uint64_t snapshot_digest;
    bool valid;
};

bool BuildLocalBAV3BalmFactorProvenance(
    int factor_id,
    const std::vector<
        g2o::LocalBAV3BalmCluster,
        Eigen::aligned_allocator<g2o::LocalBAV3BalmCluster>>& clusters,
    const g2o::LocalBAV3BalmFixedMoments* fixed_moments,
    const V3D& initial_normal,
    const V3D& world_center,
    double lambda0,
    double lambda1,
    double residual_rms,
    double residual_p90,
    LocalBAV3BalmFactorProvenance& provenance,
    std::string& reason);

LocalBAV3BalmSourceAudit SummarizeLocalBAV3BalmSourceProvenance(
    const std::vector<LocalBAV3BalmFactorProvenance>& factors);

std::string FormatLocalBAV3BalmDigest(std::uint64_t digest);

}

#endif

