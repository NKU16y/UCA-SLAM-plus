// Declaration-only preview. Function implementations are not included.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace UCA_SLAM {

class NativeVoxelFrame {
 public:

  using Pose = Eigen::Matrix4d;

  struct Snapshot {

    Pose world_from_map;
    std::uint64_t epoch;

    Eigen::Vector3d MapPoint(const Eigen::Vector3d& world) const;
    Eigen::Vector3d WorldPoint(const Eigen::Vector3d& map) const;
    Eigen::Matrix3d MapCovariance(const Eigen::Matrix3d& world) const;
    Eigen::Matrix<double, 6, 6> WorldPlaneCovariance(
        const Eigen::Matrix<double, 6, 6>& map) const;
    void WorldPlane(const Eigen::Vector3d& map_normal,
                    const Eigen::Vector3d& map_center, double map_offset,
                    Eigen::Vector3d& normal, Eigen::Vector3d& center,
                    double& offset) const;
    Pose MapPose(const Pose& world) const;
  };

  static Pose Inverse(const Pose& pose);

  static bool IsRigid(const Pose& pose);

  static Pose CanonicalRigid(const Pose& pose);

  Snapshot Read() const;

  void ResetWorldMap();

  bool BeginFrame(std::uint64_t previous_frame, const Pose& previous_world,
                  bool reanchor );

  bool CanCommit(std::uint64_t frame, const Pose& accepted_world,
                 const Snapshot& queried) const;

  bool Commit(std::uint64_t frame, const Pose& accepted_world,
              const Snapshot& queried);

 private:
  Snapshot current_;
  bool has_previous_;
  std::uint64_t previous_frame_;
  Pose previous_map_;
};
}

#include "NativeVoxelRegistration.h"
#include "common.h"
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
namespace UCA_SLAM {
class LidarTracking;
class Frame;
namespace NativeMeasurementPreparation {
bool Enabled();
struct Packet {

 std::unique_ptr<NativeVoxel::Registration::Prepared> registration;
 NativeVoxel::Points map_input;
 PointCloud::ConstPtr corrected_cloud;
 NativeVoxel::Pose calibration;
 const void* measurement_owner;
 const void* support_owner;
 std::uint64_t owner_generation,support_revision,frame;
 double stamp,lock_wait_ms,sample_ms,job_ms;
 std::size_t source_points;
 bool usable;
 std::string reason;
};
class Worker;

class Ticket {
 friend class Worker;
 std::future<std::unique_ptr<Packet>> result_;
 bool submitted_;
 explicit Ticket(std::future<std::unique_ptr<Packet>>&& result);
public:
 ~Ticket();
 Ticket(const Ticket&)=delete;
 Ticket& operator=(const Ticket&)=delete;
 std::unique_ptr<Packet> Get();
};
class Worker {
 std::mutex mutex_;
 std::condition_variable wake_,idle_;
 bool stopping_,busy_;
 std::function<void()> job_;
 std::thread worker_;
 void Loop();
public:
 Worker();
 ~Worker();
 Worker(const Worker&)=delete;
 Worker& operator=(const Worker&)=delete;
 std::unique_ptr<Ticket> Submit(LidarTracking&,const PointCloud::Ptr&,
     std::uint64_t,double,const NativeVoxel::Pose&);
 void Drain();
 void Stop();
};
bool SameSource(const Packet&,const Frame&);
bool SameCalibration(const Packet&,const NativeVoxel::Pose&);
}}

/*
 * @Author: Shizhuo Yu
 * @Date: 2025-07-23 22:00:19
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2025-08-27 11:06:53
 */
#ifndef LIDARTRACKING_H
#define LIDARTRACKING_H
#include "LidarHandler.h"
#include "voxelMapUtil.hpp"
#include "LidarVoxelSourceLedger.h"
#include "NativeVoxelRegistration.h"
#include "LiDARIncludes/voxelMapUtil.hpp"
#include "common.h"
#include "Frame.h"
#include <atomic>
#include <memory>
#include <string>
namespace UCA_SLAM
{
class Frame;
class KeyFrame;

enum class PointWorldCovMode
{
    SensorOnly = 0,
    IncludePoseCov = 1
};

struct ScanMatchLocalMapTiming
{
    double downsample_ms;
    double point_covariance_ms;
    double local_covariance_copy_ms;
    double world_covariance_ms;
    double map_lock_wait_ms;
    double map_search_ms;
    double export_ms;
    size_t downsampled_points;
    size_t matched_residuals;
    size_t unmatched_points;
    int map_search_caller_active_level;
    int map_search_effective_threads;
};

class LidarTracking
{
private:
    std::unique_ptr<LidarPara> mOwnedParameters;
public:

    LidarTracking();
    ~LidarTracking();

    LidarPara *mpLP;

    bool bfusion;

    MD(4,4) Tvl;

    bool is_init;

    double first_lidar_time;

    bool init_map;

    void setLidarPara(LidarPara *LP_);
    bool FreezeKeyFrameBalmAssociations(KeyFrame* keyframe);

    StatesGroup state;
    std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;

    mutable std::mutex feat_map_mtx;

    bool mbNativeVoxelFrameEnabled;
    bool mbNativeVoxelEventAnchor;
    bool mbNativeVoxelAdaptiveAnchor;
    bool mbNativeVoxelAdaptiveLatched;
    bool mbNativeVoxelAdaptiveReleased;
    MD(4,4) mNativeVoxelAdaptiveIgnoredCorrection;
    MD(4,4) mNativeVoxelLastAcceptedWorld;
    MD(4,4) mNativeVoxelLastAcceptedFinalWorld;
    unsigned long mnNativeVoxelLastAcceptedFrame;
    bool mbNativeVoxelLastAcceptedValid;

    std::atomic<unsigned long> mnNativeVoxelWorldCorrectionRevision;
    unsigned long mnNativeVoxelWorldCorrectionConsumed;
    NativeVoxelFrame mNativeVoxelFrame;
    NativeVoxelFrame::Snapshot mNativeVoxelTrackingSnapshot;
    unsigned long mnNativeVoxelTrackingFrame;
    bool mbNativeVoxelTrackingPrepared;
    void ConfigureNativeVoxelFrame();
    bool IsNativeVoxelFrameEnabled() const;
    void BeginNativeVoxelFrame(unsigned long previous_frame,
                               unsigned long current_frame,
                               const MD(4,4)& previous_world_pose);
    void ResetNativeVoxelFrameLocked(const char* reason);
    NativeVoxelFrame::Snapshot ReadNativeVoxelFrameLocked() const;
    void QueryNativeVoxelMapLocked(
        const std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>>& world_queries,
        std::vector<ptpl, Eigen::aligned_allocator<ptpl>>& world_matches,
        PLV(3)& non_match, bool for_ba, const char* consumer,
        unsigned long frame_id, int* effective_threads );
    void ConvertNativeVoxelInsertionLocked(
        std::vector<pointWithCov, Eigen::aligned_allocator<pointWithCov>>& world_points,
        unsigned long frame_id, const MD(4,4)& accepted_world_pose);
    void CommitNativeVoxelFrameLocked(unsigned long frame_id,
                                     const MD(4,4)& accepted_world_pose,
                                     std::size_t inserted_points,
                                     const MD(4,4)& immediate_final_world_pose);

    std::unique_ptr<NativeVoxel::Registration> mpNativeVoxelRegistration;
    std::unique_ptr<NativeVoxel::ObservedSupportIndex> mpNativeObservedIndex;
    NativeVoxel::Points mNativeMapInput;
    double mNativeInputPreparationMs;

    std::uint64_t mnNativeMeasurementOwnerGeneration;
    MD(4,4) mNativeMeasurementWorld;
    void PrepareNativeMeasurement(Frame& current, const Frame& last);
    void PrepareNativeMeasurementLocked(Frame& current);
    void PublishNativePreparedMeasurementLocked(Frame& current, std::size_t source_points);
    void PrepareNativeMeasurementWithPreparation(Frame& current, const Frame& last,
        NativeMeasurementPreparation::Ticket* ticket);
    std::unique_ptr<NativeMeasurementPreparation::Ticket> StartNativeMeasurementPreparation(
        const PointCloud::Ptr& cloud, unsigned long frame, double stamp, const MD(4,4)& calibration);
    void StopNativeMeasurementPreparation();
    void InitializeNativeMeasurementLocked(Frame& current);
    void AppendNativeObservedSupportLocked();
    void RemoveNativeObservedRootLocked(OctoTree* root);
    void ClearNativeObservedIndexLocked();
    void AuditNativeMeasurement(const Frame& current, bool initialization) const;
    const NativeVoxel::Measurements& NativeMeasurementHistory() const;

    std::unique_ptr<LidarVoxelSourceLedger> mpLidarVoxelSourceLedger;
    std::atomic<std::uint64_t> mnLidarVoxelSourceToken;
    bool mbLidarVoxelSourceLedgerEnabled;

    std::atomic<unsigned long> mnLidarVoxelMapVersion;
    std::atomic<unsigned long> mnLidarVoxelMapUpdateSeq;
    std::atomic<bool> mbLidarVoxelMapDirty;
    std::atomic<bool> mbLidarVoxelMapRebuilding;
    mutable unsigned long mnLidarMeasurementPlaneCacheUpdateSeq;
    using LidarMeasurementPlaneCacheValue = std::pair<
        const int,
        LidarVoxelSourceLedger::PlaneMeasurementCovarianceComponents>;
    mutable std::unordered_map<
        int,
        LidarVoxelSourceLedger::PlaneMeasurementCovarianceComponents,
        std::hash<int>,
        std::equal_to<int>,
        Eigen::aligned_allocator<LidarMeasurementPlaneCacheValue>>
            mLidarMeasurementPlaneCache;

    unsigned long GetLidarVoxelMapVersion() const;
    unsigned long GetLidarVoxelMapUpdateSeq() const;
    void MarkLidarVoxelMapDirty(const std::string& reason );
    void BeginLidarVoxelMapRebuild(const std::string& reason );
    unsigned long FinishLidarVoxelMapRebuild(const std::string& reason , bool success );
    unsigned long BumpLidarVoxelMapVersion(const std::string& reason );
    unsigned long BumpLidarVoxelMapUpdateSeq(const std::string& reason );
    bool IsLidarVoxelMapDirty() const;
    bool IsLidarVoxelMapRebuilding() const;
    bool IsLidarVoxelMapUsable() const;
    bool IsLidarVoxelSourceLedgerEnabled() const;
    LidarVoxelSourceLedger::Parameters GetLidarVoxelSourceLedgerParameters() const;
    bool CommitLidarVoxelSourcePoseUpdates(
        const std::vector<LidarVoxelSourceLedger::AnchorPoseUpdate,
            Eigen::aligned_allocator<LidarVoxelSourceLedger::AnchorPoseUpdate>>& updates,
        LidarVoxelSourceLedger::CommitStats& stats,
        std::string& reason,
        const PostBAVoxelPlaneCovarianceRequest* postba_request );
    bool FreezeLidarVoxelSourceAnchor(unsigned long anchor_id,
                                      std::string& reason);

    void ClearLidarVoxelSourceLedger();

    void ResetForNewMap();
    bool RegisterExistingLidarVoxelSource(
        std::uint64_t source_id,
        unsigned long frame_id,
        const LidarVoxelSourceLedger::AnchorPose& anchor,
        const std::vector<pointWithCov,
            Eigen::aligned_allocator<pointWithCov>>& world_points,
        std::string& reason);
    bool ComputeVoxelPlaneMeasurementCovarianceComponents(
        const ptpl& association,
        LidarVoxelSourceLedger::PlaneMeasurementCovarianceComponents&
            output) const;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    pcl::VoxelGrid<PointT> downSizeFilterSurfforBA;
    PointCloud::Ptr laserCloudOri;
    PointCloud::Ptr corr_normvect;

    void calibLaser(Frame &CurrentFrame);
    void initMapBuilding(Frame &CurrentFrame);
    void downsamplingInScan(Frame &CurrentFrame);
    void PrepareFrameGraphLidar(Frame &CurrentFrame, PointCloud::Ptr &pc_local, PLM(3) &body_var, PLM(3) &crossmat_list);
    void scanMatchPreparedCloud(
        const PointCloud::Ptr &pc,
        const PLM(3) &body_var,
        const PLM(3) &crossmat_list,
        int &effct_feat_num,
        std::vector<ptpl,Eigen::aligned_allocator<ptpl>> &ptpl_list,
        const bool &forBA,
        PointWorldCovMode cov_mode ,
        unsigned long frame_id );
    void scanMatchforLocalMap(
        Frame &CurrentFrame,
        int &effct_feat_num,
        std::vector<ptpl, Eigen::aligned_allocator<ptpl>> &ptpl_list,
        const bool &forBA,
        PointWorldCovMode cov_mode ,
        PLM(3)* pOutLocalPointCovs ,
        ScanMatchLocalMapTiming* pTiming );
    void addPointsToVoxelMap(
        Frame &CurrentFrame,
        const PLM(3) &body_var,
        const PLM(3) &crossmat_list,
        PointWorldCovMode cov_mode );

    std::unique_ptr<NativeMeasurementPreparation::Worker> mpNativeMeasurementPreparationWorker;

};

}
#endif

