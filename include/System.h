// Declaration-only preview. Function implementations are not included.
/*
 * @Author: Shizhuo Yu
 * @Date: 2025-07-18 14:17:46
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2025-07-23 22:11:01
 */
#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include "PointTypes.h"
#include "ORBVocabulary.h"

namespace UCA_SLAM
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class MapDrawer;
class LidarTracking;
class MapPoint;
class KeyFrameDatabase;

class System
{
public:

    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        MONOCULARWITHLIDAR=3
    };

public:

    System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer );

    System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer, const bool bwithLidar, const bool bFusion);

    ~System();
    System(const System&) = delete;
    System& operator=(const System&) = delete;

    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    cv::Mat TrackMonocularwithLidar(const cv::Mat &im, const double &timestamp, PointCloud::Ptr lidarPointCloud);

    void ActivateLocalizationMode();

    void DeactivateLocalizationMode();

    bool MapChanged();

    void Reset();

    void Shutdown();

    void SaveTrajectoryTUM(const std::string &filename);

    void SaveKeyFrameTrajectoryTUM(const std::string &filename);

    void SaveTrajectoryKITTI(const std::string &filename);

    int GetTrackingState();

    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    friend class TrackingLifecycleTestAccess;

    void StopWorkers();

    eSensor mSensor;

    std::unique_ptr<ORBVocabulary> mpVocabulary;

    std::unique_ptr<KeyFrameDatabase> mpKeyFrameDatabase;

    std::unique_ptr<Map> mpMap;

    std::unique_ptr<LidarTracking> mpOwnedLidarTracking;

    std::unique_ptr<Tracking> mpTracker;

    std::unique_ptr<LocalMapping> mpLocalMapper;

    std::unique_ptr<LoopClosing> mpLoopCloser;

    std::unique_ptr<Viewer> mpViewer;

    std::unique_ptr<FrameDrawer> mpFrameDrawer;
    std::unique_ptr<MapDrawer> mpMapDrawer;

    std::unique_ptr<std::thread> mptLocalMapping;
    std::unique_ptr<std::thread> mptLoopClosing;
    std::unique_ptr<std::thread> mptViewer;

    std::mutex mMutexReset;
    bool mbReset;

    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}

#endif
