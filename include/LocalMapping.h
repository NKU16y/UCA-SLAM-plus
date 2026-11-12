// Declaration-only preview. Function implementations are not included.
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "UncertaintyUtil.h"
#include "LocalBALifecycle.h"
#include <thread>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace UCA_SLAM
{

class Tracking;
class LoopClosing;
class Map;
struct LocalBAPoseCovPackage;
class LocalMapping
{
public:
    struct QueueAuditSnapshot
    {
        int length;
        long long newest_kf_id;
        long long newest_mapped_kf_id;
    };

    LocalMapping(Map* pMap, const float bMonocular);

    LocalMapping(Map* pMap, const bool bMonocularWithLidar, const bool bFusion);

    LocalMapping(const LocalMapping&) = delete;
    LocalMapping& operator=(const LocalMapping&) = delete;

    void SetFeatMap(std::unordered_map<VOXEL_LOC, OctoTree*> &map, std::mutex &mtx);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    void Run();

    void RunwithLidar();

    void InsertKeyFrame(KeyFrame* pKF, bool interrupt_inflight_ba );

    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue();

    QueueAuditSnapshot GetQueueAuditSnapshot();

    bool RebuildLidarVoxelMapFromKeyFrames(const std::string& reason );
    size_t ReleaseOldLidarAssociationCaches(int keep_recent_kfs );
    size_t PruneLidarVoxelMapAroundKeyFrame(KeyFrame* pCenterKF, double radius );
    size_t ReleaseLidarAssociationCachesOutsideRadius(KeyFrame* pCenterKF, double radius, bool xz_only);
    void SanitizeLidarAssociationsForCurrentMap(const std::vector<KeyFrame*>& kfs);
    bool ReassociateLidarForKeyFrame(KeyFrame* pKF, unsigned long target_version, const std::string& reason );
    size_t EnsureLidarAssociationsForKeyFrames(const std::vector<KeyFrame*>& kfs, unsigned long target_version, size_t max_reassociate , const std::string& reason );
    size_t RefreshActiveLidarAssociationsBudgeted(const std::vector<KeyFrame*>& kfs, unsigned long target_version, size_t max_reassociate, const std::string& reason );

protected:

    bool RunOneLocalMappingLoopStepWithLidar();
    void ProcessNewKeyFrameLoopWithLidar();
    void RunLidarVisualJointBA(
        KeyFrame* pKF_anchor,
        std::uint64_t coalesced_request_generation );

    bool CheckNewKeyFrames();

    void ProcessNewKeyFramewithLidar();
    void CreateNewMapPointswithLidar();

    void SearchInNeighbors();

    void MapPointCullingwithLidar();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    bool mbMonocularwithLidar;
    bool mbFusion;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    LocalBAV3CancellationToken mLocalBAV3CancellationToken;
    LocalBAV3LatestRequestCoalescer mLocalBAV3RequestCoalescer;

    int mnLocalBAConsecutiveCancelCount;

    std::atomic<long long> mnLifecycleLatestMappedKF;
    std::atomic<std::uint64_t> mnLifecycleActiveBAGeneration;
    std::uint64_t mnLifecycleCommitAwaitingNextKF;
    std::uint64_t mnLifecycleCurrentAnchorRequestGeneration;
    std::unordered_map<unsigned long, std::uint64_t>
        mLifecycleRequestGenerationByKF;

    std::unordered_set<unsigned long> mUcaMovableLocalBACommittedKFs;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    bool mbLocalBAL0LastCommitValid;
    unsigned long mnLocalBAL0LastCommitKFId;
    unsigned long mnLocalBAL0LastCommitFrameId;
    cv::Mat mLocalBAL0LastCommitPose;
    bool mbLocalBAL0LastSampleValid;
    cv::Mat mLocalBAL0LastSampleCenter;
    double mLocalBAL0PathSinceCommitM;

    bool mbLocalBAFixedGapLastAttemptValid;
    unsigned long mnLocalBAFixedGapLastAttemptKFId;
    LocalBAReplaySchedule mLocalBAReplaySchedule;
    bool mbLocalBAReplayScheduleLoadAttempted;

private:

    std::unordered_map<VOXEL_LOC,OctoTree*>* pFeatMap;
    std::mutex *pFeatMapMtx;

};

}

#endif
