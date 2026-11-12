// Declaration-only preview. Function implementations are not included.
#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
namespace UCA_SLAM { namespace NativeVoxel {
using Pose=Eigen::Matrix4d;
using Points=std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>;
using Matrix6=Eigen::Matrix<double,6,6>;
struct Measurement {

 std::uint64_t frame;
 double stamp,sigma,milliseconds;
 Pose map_from_camera;
 Matrix6 conditional_body_right_covariance;
 std::size_t correspondences;
 int iterations;
 bool converged,covariance_valid;
};
using Measurements=std::vector<Measurement,Eigen::aligned_allocator<Measurement>>;

class Registration {
public:

 using Nearest=std::function<bool(const Eigen::Vector3d&,Eigen::Vector3d&,double&)>;
 explicit Registration(double sensor_range_sigma,const Pose& camera_from_lidar);
 void Initialize(std::uint64_t frame,double stamp,const Pose& map_from_camera);
 const Measurement& Prepare(std::uint64_t frame,double stamp,const Points&,const Nearest&);

 struct PreparationState {

  Pose camera_from_lidar,last,delta;
  std::uint64_t last_frame,model_samples;
  double last_stamp,sensor_noise_variance,model_sse;
 };
 struct Prepared {

  PreparationState state;
  Measurement measurement;
  Pose prediction;
 };
 PreparationState CapturePreparationState() const;
 static std::unique_ptr<Prepared> ComputePreparation(
     const PreparationState&,std::uint64_t,double,const Points&,const Nearest&);
 bool AdoptPreparation(const Prepared&);
 void Commit();
 void Cancel();
 bool Pending() const;
 const Measurement& Current() const;
 const Measurements& History() const;
 Pose LastPose() const;
 static Pose ExpBody(const Eigen::Matrix<double,6,1>&);
 static Eigen::Matrix3d Skew(const Eigen::Vector3d&);
private:
 Pose camera_from_lidar_;
 Pose last_,delta_,prediction_;
 Measurements history_;
 Measurement current_;
 bool pending_;
 double sensor_noise_variance_;
 double model_sse_;
 std::uint64_t model_samples_;
};
}}

#include "NativeVoxelRegistration.h"
namespace UCA_SLAM {namespace NativeVoxel {
void SamplePreparedScan(const Points& lidar, const Pose& camera_from_lidar, Points& map, Points& query);
}}

