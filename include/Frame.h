// Declaration-only preview. Function implementations are not included.
#ifndef FRAME_H
#define FRAME_H

#include "PoseCovarianceMetadata.h"
#include "VisualUncertaintyModel.h"

#include<vector>

namespace UCA_SLAM
{

struct UcaLidarDepthCandidate
{
    bool valid;
    float depth;
    float sigma_z;
    float info_scale;
    unsigned char level;
    unsigned char source;
    unsigned char count_current;
    unsigned char count_last;
};

}

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "LiDARIncludes/LidarHandler.h"
#include <opencv2/opencv.hpp>

namespace UCA_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:

    Frame();

    Frame(const Frame &frame);

    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    void VisualFeatureExtraction();
    void LidarFeatureExtractionFusion();

    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
          PointCloud::Ptr pc, LidarPara *LP);

    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
          PointCloud::Ptr pc, LidarPara *LP, const Eigen::Matrix4d &Tvl);

    void ExtractORB(int flag, const cv::Mat &im);

    void ComputeBoW();

    void SetPose(cv::Mat Tcw);

    void UpdatePoseMatrices();

    inline cv::Mat GetCameraCenter();

    inline cv::Mat GetRotationInverse();

    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const;

    void ComputeStereoMatches();

    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    void ComputeStereoFromLiDAR();

    cv::Mat UnprojectStereo(const int &i);

public:

    ORBVocabulary* mpORBvocabulary;

    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    double mTimeStamp;

    double mUcaPTrackOrbMs;
    double mUcaPTrackLidarPreprocessMs;
    double mUcaPTrackLidarScanMs;
    double mUcaPTrackLidarTransformMs;
    double mUcaPTrackLidarProjectionMs;

    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    float mbf;

    float mb;

    float mThDepth;

    int N;

    cv::Mat mimg;

    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    std::vector<VisualCovariance2DStorage> mvUcaLocationCovariances;
    std::vector<VisualCovariance2DStorage> mvUcaMatchingCovariances;

    std::vector<float> mvuRight;
    std::vector<float> mvDepth;
    std::vector<double> mvDepthUncertainty;
    std::vector<UcaLidarDepthCandidate> mvUcaLidarDepthCandidates;

    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    cv::Mat mDescriptors, mDescriptorsRight;

    std::vector<MapPoint*> mvpMapPoints;

    std::vector<bool> mvbOutlier;

    std::vector<unsigned char> mvbF2MVisualDepthConflict;

    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    cv::Mat mTcw;

    cv::Mat poseCov;

    cv::Mat obsPoseCov;
    bool mbObsPoseCovValid;
    PoseCovarianceMetadata mPoseCovMetadata;

    static long unsigned int nNextId;
    long unsigned int mnId;

    KeyFrame* mpReferenceKF;

    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    LidarHandler mLH;

    int validLidarInlierCount;
private:

    void UndistortKeyPoints();

    void ComputeImageBounds(const cv::Mat &imLeft);

    void AssignFeaturesToGrid();

    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw;
};

}

#endif
