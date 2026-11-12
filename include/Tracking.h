// Declaration-only preview. Function implementations are not included.


#ifndef UCA_TRACKING_FRONTEND_TYPES_H
#define UCA_TRACKING_FRONTEND_TYPES_H
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>
#include "PoseCovarianceMetadata.h"
namespace UCA_SLAM { class Map; class Tracking; class KeyFrame; class Frame;
struct TrackingVisualEdgeState
{
    std::vector<unsigned char> created;
    std::vector<unsigned char> final_level0;

    void Reset(const std::size_t size);
};

}

namespace UcaKissFrontend {
using Jacobian3x6 = Eigen::Matrix<double,3,6>;
struct PointToPointFactor {

 Eigen::Vector3d source_camera, target_world;
 static Eigen::Matrix3d Skew(const Eigen::Vector3d& p);
 Eigen::Vector3d Residual(const Eigen::Matrix4d& Tcw) const;
 Jacobian3x6 LeftTcwJacobian(const Eigen::Matrix4d& Tcw) const;
 static double PinnedUpstreamIRLSWeight(double residual_squared, double kernel_scale);
};
}

namespace UcaKissFrontend {
struct AcceptedCameraPose {

 std::uint64_t frame;
 double stamp;
 Eigen::Matrix4d initial_camera_from_current_camera;
};
using AcceptedCameraPoses=std::vector<AcceptedCameraPose,Eigen::aligned_allocator<AcceptedCameraPose>>;
struct Correspondence {

 PointToPointFactor factor;
 std::size_t source_index;
 double initial_weight;
};
struct PreparedFrame {

 std::uint64_t frame,map_revision,world_epoch;
 double stamp,sigma;
 bool map_empty;
 Eigen::Matrix4d aligned_Tcw;

 Eigen::Matrix4d aligned_odometry_from_lidar;
 Eigen::Matrix4d world_from_odometry;
 std::size_t source_count;
 std::vector<Correspondence,Eigen::aligned_allocator<Correspondence>> correspondences;
};

class SingleLidarFrontend {
 public:

 explicit SingleLidarFrontend(const Eigen::Matrix4d& camera_from_lidar);
 ~SingleLidarFrontend();
 SingleLidarFrontend(const SingleLidarFrontend&)=delete;
 SingleLidarFrontend& operator=(const SingleLidarFrontend&)=delete;

 const PreparedFrame& Prepare(const std::vector<Eigen::Vector3d>& prepared_lidar,
                              std::uint64_t frame,double stamp);
 void Commit(const Eigen::Matrix4d& accepted_Tcw);

 void CommitICPMeasurement();
 void Cancel();

 void SetWorldFromOdometry(const Eigen::Matrix4d& world_from_odometry);
 Eigen::Matrix4d WorldFromOdometry() const;
 Eigen::Matrix4d LastOdometryFromLidar() const;
 std::vector<Eigen::Vector3d> LocalMapOdometry() const;
 std::uint64_t MapRevision() const;
 bool HasPendingFrame() const;
 const PreparedFrame& PendingFrame() const;

 const AcceptedCameraPoses& AcceptedPosesInInitialCamera() const;
 private:
 struct Impl;
 std::unique_ptr<Impl> impl_;
};
}

namespace UCA_SLAM { namespace SingleFrontendBackend {

inline bool IsExactFloatPoseStorage(const Eigen::Matrix4d& solved,
                                    const Eigen::Matrix4d& stored);
}}

namespace UCA_SLAM { namespace SingleFrontendBackend {
struct AcceptedPose {

 std::uint64_t frame;
 double stamp;
 Eigen::Matrix4d initial_camera_from_current_camera;
};
using AcceptedPoses=std::vector<AcceptedPose,Eigen::aligned_allocator<AcceptedPose>>;
struct KeyFrameState {

 unsigned long id;
 std::uint64_t frame;
 double stamp;
 Eigen::Matrix4d Tcw;
};
using KeyFrameStates=std::vector<KeyFrameState,Eigen::aligned_allocator<KeyFrameState>>;
struct Pair {
 std::uint64_t first,last;
 double path_m;
};
struct Result {
 KeyFrameStates states;
 std::vector<Pair> pairs;
 unsigned long native_regularizers,frontend_regularizers;
 double native_chi2_before,native_chi2_after;
 double frontend_chi2_before,frontend_chi2_after;
 double robust_cost_before,robust_cost_after;
 double seconds;
 int iterations;

 bool production_map_committed;
};

Result Solve(const KeyFrameStates&,const AcceptedPoses&,bool enabled);
}}

namespace UCA_SLAM {class Map;class Tracking;class KeyFrame;class Frame;}
namespace UCA_SLAM {namespace SingleFrontendBackend {

void RunAfterWorkersJoin(Map*,Tracking*);

bool OnlineRequested();
using PendingKeyFrames=std::function<std::vector<KeyFrame*>()>;
void RunOnline(Map*,Tracking*,KeyFrame*,const PendingKeyFrames&);

void ResetOnlineState(Map*);

void RecordOnlineTrackingConsumption(Map*,Tracking*,const Frame&);
}}

namespace UCA_SLAM { namespace SingleFrontend {
struct Result {

 Eigen::Matrix<double,6,6> covariance;
 bool covariance_valid;
 PoseCovarianceMetadata metadata;
 TrackingVisualEdgeState visual_edges;
 std::vector<int> lidar_indices;
 std::vector<double> lidar_weights,visual_weights;
 std::vector<unsigned char> visual_observations_validated;
 int visual_inliers;
 int point_solver_rounds,point_iterations_requested,point_iterations_completed;
 double lidar_hessian_trace,visual_hessian_trace;
 double correction_translation,correction_rotation;
};

bool VisualPoseFactorsEnabled(int mode);

bool Optimize(Frame& frame,const UcaKissFrontend::PreparedFrame& scan,int mode,Result& result);
}}

namespace UCA_SLAM {namespace NativeVoxel {
using GraphVector6=Eigen::Matrix<double,6,1>;
using GraphMatrix6=Eigen::Matrix<double,6,6>;
GraphVector6 RelativeLog(const Eigen::Matrix4d&);
GraphMatrix6 LeftLogInverse(const GraphVector6& rotation_translation);
}}

#endif

#ifndef UCA_TRACKING_TYPES_ONLY
#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "LidarTracking.h"
#include "UncertaintyUtil.h"
#include "KeyFrame.h"
#include <mutex>
#include <memory>

namespace UCA_SLAM
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class LidarTracking;
namespace NativeMeasurementPreparation {class Ticket;}
class UncertaintyUtil;
struct ProjectionMatchStats;
class Tracking
{

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    ~Tracking();

    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
    cv::Mat GrabImageMonocularwithLidar(const cv::Mat &im, const PointCloud::Ptr pc, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    void SetLidarTracking(LidarTracking* pLidarTracking);
    void setExtrinsics(const cv::FileStorage fSettings);

    void ChangeCalibration(const string &strSettingPath);

    void InformOnlyTracking(const bool &flag);

public:

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    int mSensor;

    Frame mCurrentFrame;
    cv::Mat mImGray;

    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    bool mbOnlyTracking;

    void Reset();

    LidarTracking *mpLT;
    std::shared_ptr<UcaKissFrontend::SingleLidarFrontend> mSingleLidarFrontend;
    int SingleLidarFrontendMode() const;
    const std::vector<unsigned char>& VisualMapObservationMask() const;
    int VisualMapObservationCount() const;
    int VisualMapTrackedObservationCount() const;
    void PrepareSingleLidarFrontend(bool refresh_last);
    void CommitSingleLidarFrontend(bool initialization);
    void AuditSingleLidarFrontendCommit(bool initialization,
        const Eigen::Matrix4d* icp_measurement_Tcw);
    bool OptimizeSingleLidarFrontend(
        vector<int>& visual_indices, vector<double>& visual_weights,
        vector<int>& lidar_indices, vector<double>& lidar_weights,
        MD(6,6)* covariance, bool* covariance_valid,
        PoseCovarianceMetadata* covariance_metadata);

protected:

    void Track();

    void TrackwithLidarFusion(NativeMeasurementPreparation::Ticket* native_preparation);

    void StereoInitialization();

    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void VisualInitialization(bool &flagVisual);
    void LidarInitialization(bool &flagLidar);
    void MonocularwithLiDARFusionInitialization();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    int SearchLocalPoints(float searchRadiusScale , ProjectionMatchStats* pStatsOut , bool countVisibility );

    int VisualAssociation(float searchRadiusScale , ProjectionMatchStats* pStatsOut );

    void LidarAssociation(const PointCloud::Ptr &pc_local, const PLM(3) &body_var, const PLM(3) &crossmat_list, int &effct_feat_num, std::vector<ptpl,Eigen::aligned_allocator<ptpl>> &ptpl_list);

    bool TrackLocalMapwithLidar(vector<ptpl,Eigen::aligned_allocator<ptpl>> &ptpl_list, PLM(3) &all_point_covs, PLM(3) &all_crossmat_list, PLM(3) &point_covs,
    vector<int> &inlierVisualIndices, vector<double> &inlierVisualWeights, vector<int> &inlierLidarIndices, vector<double> &inlierLidarWeights,
    MD(6,6)* pHybridPoseCovOut , bool* pHybridPoseCovValidOut , PoseCovarianceMetadata* pHybridPoseCovMetadataOut );

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    bool NeedNewKeyFramewithLidar();
    void CreateNewKeyFramewithLidar(const PLM(3) &all_point_covs, const PLM(3) &all_crossmat_list);

    bool mbVO;

    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    std::unique_ptr<ORBextractor> mpORBextractorLeft, mpORBextractorRight;
    std::unique_ptr<ORBextractor> mpIniORBextractor;

    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    std::unique_ptr<Initializer> mpInitializer;

    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    System* mpSystem;

    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    Map* mpMap;

    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    int mMinFrames;
    int mMaxFrames;

    float mThDepth;

    float mDepthMapFactor;

    int mnMatchesInliers;

    std::vector<unsigned char> mvSingleFrontendValidatedVisualObservations;
    int mSingleFrontendValidatedVisualCount;
    int mSingleFrontendValidatedTrackedVisualCount;
    unsigned long mSingleFrontendVisualObservationFrame;
    int mLastLocalMapVisualInliers;
    int mLastLocalMapLidarInliers;
    unsigned long mLastLocalMapInlierFrameId;
    std::vector<unsigned char> mvLastLocalMapVisualEdgeCreated;
    std::vector<unsigned char> mvLastLocalMapVisualEdgeFinalLevel0;
    unsigned long mLastLocalMapVisualEdgeStateFrameId;

    bool mbKFAllowVisualMPCreate;

    bool mbKFQueueWithoutInterrupt;

    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    cv::Mat mVelocity;

    bool bFusion;
    cv::Mat mExtrinsics;

    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

};

}

#endif

#endif
