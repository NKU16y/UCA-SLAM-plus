// Declaration-only preview. Function implementations are not included.
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "LocalBAV3Covariance.h"
#include "LocalBAGeometry.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <string>
#include<mutex>
#include<unordered_map>

namespace UCA_SLAM
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:

    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    void SetWorldGeometryInMapTransaction(
        const cv::Mat& position, const cv::Mat& covariance,
        const std::unique_lock<std::mutex>& map_transaction,
        const std::unique_lock<std::mutex>& geometry_transaction);

    void SetWorldCov(const cv::Mat &Cov);
    cv::Mat GetWorldCov();

    void SetBirthCovarianceSnapshot(
        const MapPointBirthCovarianceSnapshot& snapshot);
    bool GetBirthCovarianceSnapshot(
        MapPointBirthCovarianceSnapshot& snapshot);

    void SetSourceDepthObservation(KeyFrame* pKF,
                                   const cv::Mat &Xc,
                                   const cv::Mat &CovXc,
                                   const int idx,
                                   const int source_type);
    bool GetSourceDepthObservation(KeyFrame*& pKF,
                                   cv::Mat &Xc,
                                   cv::Mat &CovXc,
                                   int &idx,
                                   int &source_type);
    bool HasSourceDepthObservation();

    void SetSourcePlaneObservation(const cv::Mat &normal_xc,
                                   const double d,
                                   const float fit_rms,
                                   const float planarity_ratio,
                                   const int support_count);
    void SetSourcePlaneObservation(
        const LocalPlaneMeasurementSnapshot& snapshot);
    bool GetSourcePlaneObservation(cv::Mat &normal_xc,
                                   double &d,
                                   float &fit_rms,
                                   float &planarity_ratio,
                                   int &support_count);
    bool GetSourcePlaneObservation(
        LocalPlaneMeasurementSnapshot& snapshot);

    void ClearSourcePlaneObservation();
    void SetUcaSourceDepthWeakMetadata(unsigned char source_depth_type,
                                       float info_scale);
    void SetUcaSourceDepthWeakMetadata(unsigned char source_depth_type,
                                       float info_scale,
                                       float candidate_depth,
                                       float candidate_sigma_z,
                                       unsigned long source_kf_id,
                                       int source_keypoint_idx);

    bool GetUcaSourceDepthWeakMetadata(unsigned char &source_depth_type,
                                       float &info_scale,
                                       float &candidate_depth,
                                       float &candidate_sigma_z,
                                       unsigned long &source_kf_id,
                                       int &source_keypoint_idx);

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();

    void IncreaseVisible(int n);
    void IncreaseFound(int n);
    float GetFoundRatio();
    inline int GetFound();
    inline int GetVisible();

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void UpdateNormalAndDepth(
        std::unordered_map<KeyFrame*, cv::Mat>* camera_center_cache);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    bool iftriangulation;

    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    long unsigned int mnBALocalForKF;
    long unsigned int mnLiDARBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

protected:

     cv::Mat mWorldPos;

     cv::Mat mWorldCov;

     MapPointBirthCovarianceSnapshot mBirthCovarianceSnapshot;
     bool mbHasBirthCovarianceSnapshot;

     KeyFrame* mpSourceDepthKF;
     cv::Mat mSourceDepthXc;
     cv::Mat mSourceDepthCovXc;
     int mnSourceDepthIdx;
     int mnSourceDepthType;
     bool mbHasSourceDepthObservation;
     cv::Mat mSourcePlaneNormalXc;
     double mdSourcePlane;
     float mfSourcePlaneFitRms;
     float mfSourcePlanePlanarityRatio;
     int mnSourcePlaneSupportCount;
     bool mbHasSourcePlaneObservation;
     LocalPlaneMeasurementSnapshot mSourcePlaneMeasurementSnapshot;
     bool mbHasSourcePlaneMeasurementSnapshot;
     unsigned char mUcaSourceDepthType;
     float mUcaSourceDepthInfoScale;
     float mUcaSourceDepthCandidateDepth;
     float mUcaSourceDepthCandidateSigmaZ;
     unsigned long mnUcaSourceDepthCandidateKFId;
     int mnUcaSourceDepthCandidateIdx;

     std::map<KeyFrame*,size_t> mObservations;

     cv::Mat mNormalVector;

     cv::Mat mDescriptor;

     KeyFrame* mpRefKF;

     int mnVisible;
     int mnFound;

     bool mbBad;
     MapPoint* mpReplaced;

     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

}

#endif
