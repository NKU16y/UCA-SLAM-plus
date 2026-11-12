// Declaration-only preview. Function implementations are not included.
#pragma once

#ifndef UCA_SLAM_LOCAL_BA_BUDGET_POLICY_H
#define UCA_SLAM_LOCAL_BA_BUDGET_POLICY_H

#include <algorithm>
#include <cmath>

namespace UCA_SLAM
{

struct LocalBAFixedGapDecision
{
    bool run;
    long long prior_attempt_kf_gap;
    const char* reason;
};

inline LocalBAFixedGapDecision EvaluateLocalBAFixedGap(
    const unsigned long anchor_kf_id,
    const bool has_prior_attempt,
    const unsigned long last_attempt_kf_id,
    const int requested_gap);

}

#endif

#ifndef UCA_SLAM_LOCAL_BA_V3_CANCELLATION_H
#define UCA_SLAM_LOCAL_BA_V3_CANCELLATION_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>

namespace UCA_SLAM
{

struct LocalBAV3CancellationToken
{
    std::atomic<bool> raw_stop_requested;
    std::atomic<bool> soft_stop_requested;
    std::atomic<bool> hard_stop_requested;
    bool allow_soft_late_commit;

    std::function<bool()> raw_stop_observer;
    std::function<bool()> soft_stop_observer;
    std::function<bool()> hard_stop_observer;

    void Reset();

    void RequestUnknownStop();

    void RequestSoftStop();

    void RequestHardStop();

    bool RawStopRequested() const;

    bool SoftStopRequested() const;

    bool HardStopRequested() const;

    bool StopRequested() const;
};

struct LocalBAV3LatestRequestCoalescer
{
    struct Snapshot
    {
        std::uint64_t generation;
        std::uint64_t acknowledged_generation;
        unsigned long latest_anchor_id;
        unsigned long latest_anchor_frame_id;
        bool latest_interrupt_inflight_ba;
        bool valid;

        std::uint64_t PendingCount() const;
    };

    void Reset();

    std::uint64_t Publish(const unsigned long anchor_id,
                          const unsigned long anchor_frame_id,
                          const bool interrupt_inflight_ba);

    Snapshot Read() const;

    void AcknowledgeThrough(const std::uint64_t completed_generation);

private:

    mutable std::mutex mutex;
    std::uint64_t generation;
    std::uint64_t acknowledged_generation;
    unsigned long latest_anchor_id;
    unsigned long latest_anchor_frame_id;
    bool latest_interrupt_inflight_ba;
};

struct LocalBAV3LatestRequestScheduleDecision
{
    bool run;
    bool fallback;
    std::uint64_t pending_requests;
    std::uint64_t covered_requests;
    std::uint64_t superseded_requests;
    const char* reason;
};

inline LocalBAV3LatestRequestScheduleDecision
DecideLocalBAV3LatestRequestSchedule(
    const LocalBAV3LatestRequestCoalescer::Snapshot& snapshot,
    const unsigned long current_anchor_id,
    const bool authoritative_fifo_nonempty);

}

#endif

#ifndef UCA_SLAM_LOCAL_BA_PRECOMMIT_BARRIER_H
#define UCA_SLAM_LOCAL_BA_PRECOMMIT_BARRIER_H

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>

namespace UCA_SLAM
{

inline bool LocalBAPrecommitBarrierEnabled();

inline int LocalBAPrecommitBarrierWatchdogMs();

enum class LocalBAPrecommitPhase
{
    Idle = 0,
    Eligible,
    Ready,
    Consumed
};

struct LocalBAPrecommitWaitResult
{
    bool enabled;
    bool eligible;
    bool waited;
    bool ready;
    bool watchdog_timeout;
    std::uint64_t generation;
    unsigned long anchor_id;
    double wait_ms;
    std::string outcome;
};

struct LocalBAPrecommitDrainResult
{
    bool enabled;
    bool pending;
    bool waited;
    bool watchdog_timeout;
    std::uint64_t generation;
    unsigned long anchor_id;
    double wait_ms;
    std::string outcome;
};

class LocalBAPrecommitBarrier
{
public:
    void PublishEligible(
        const std::uint64_t generation,
        const unsigned long anchor_id);

    void PublishReady(
        const std::uint64_t generation,
        const unsigned long anchor_id);

    void Finish(
        const std::uint64_t generation,
        const unsigned long anchor_id);

    LocalBAPrecommitWaitResult WaitForReady();

    LocalBAPrecommitDrainResult WaitForConsumedCommit();

private:
    std::mutex mMutex;
    std::condition_variable mCondition;
    LocalBAPrecommitPhase mPhase;
    std::uint64_t mGeneration;
    unsigned long mAnchorId;
};

inline LocalBAPrecommitBarrier& GetLocalBAPrecommitBarrier();

}

#endif

#ifndef UCA_LOCALBA_REPLAY_SCHEDULE_H
#define UCA_LOCALBA_REPLAY_SCHEDULE_H

#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace UCA_SLAM
{

struct LocalBAReplayScheduleEntry
{
    std::size_t ordinal;
    unsigned long source_anchor_id;
    unsigned long source_frame_id;
};

struct LocalBAReplayScheduleDecision
{
    bool run;
    bool valid;
    std::size_t target_ordinal;
    unsigned long target_source_anchor_id;
    unsigned long target_source_frame_id;
    std::size_t missed_now;
    std::size_t consumed;
    std::size_t remaining;
    const char* reason;
};

class LocalBAReplaySchedule
{
public:
    bool Load(const std::string& path,
              const std::size_t expected_count,
              std::string* error);

    LocalBAReplayScheduleDecision Decide(
        const unsigned long current_anchor_id,
        const unsigned long current_frame_id);

    bool loaded() const;
    std::size_t size() const;
    std::size_t consumed() const;
    std::size_t remaining() const;
    std::size_t missed_total() const;
    const std::string& path() const;

private:
    bool Fail(const std::string& message, std::string* error);

    std::vector<LocalBAReplayScheduleEntry> entries_;
    std::size_t next_;
    std::size_t missed_total_;
    bool loaded_;
    std::string path_;
};

}

#endif

