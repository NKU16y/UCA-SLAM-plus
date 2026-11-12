// Declaration-only preview. Function implementations are not included.
#pragma once

#ifndef UCA_SLAM_GLOBAL_BA_WORKER_H
#define UCA_SLAM_GLOBAL_BA_WORKER_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace UCA_SLAM
{

class GlobalBAWorker
{
public:
    struct TaskContext
    {
        explicit TaskContext(const std::uint64_t id);

        bool IsCancelled() const;

        const std::uint64_t generation;
        std::atomic<bool> stop_requested;
        std::atomic<bool> running;
    };

    GlobalBAWorker();
    GlobalBAWorker(const GlobalBAWorker&) = delete;
    GlobalBAWorker& operator=(const GlobalBAWorker&) = delete;

    ~GlobalBAWorker();

    template<class Task>
    void Start(Task task);

    void CancelAndJoin();

    void Join();

    bool IsRunning() const;

private:
    void JoinLocked(const bool cancel);

    mutable std::mutex mOwnershipMutex;
    std::thread mThread;
    std::shared_ptr<TaskContext> mContext;
    std::uint64_t mNextGeneration;
};

}

#endif

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace UCA_SLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;

class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;
    typedef map<KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
    Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    void Run();

    void RunwithLiDAR();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    void RunGlobalBundleAdjustment(unsigned long nLoopKF,
                                   const GlobalBAWorker::TaskContext& task);

    void RequestFinish();

    bool isFinished();

    bool IsScaleFixed() const;

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    bool ComputeSim3withLiDAR();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void CorrectLoopwithLiDAR();

    void StartGlobalBundleAdjustment(unsigned long nLoopKF);

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    float mnCovisibilityConsistencyTh;

    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    GlobalBAWorker mGlobalBAWorker;

    const bool mbFixScale;

};

}

#endif

