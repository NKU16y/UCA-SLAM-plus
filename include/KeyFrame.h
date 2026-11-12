// Declaration-only preview. Function implementations are not included.
#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "common.h"
#include <mutex>
#include <cstdint>
#include <limits>
#include <memory>
#include "LiDARIncludes/LidarHandler.h"
#include <unordered_map>
#include <array>
#include <map>
#include <string>
#include "LiDARIncludes/LiDARLoopPlaneRefine.hpp"
#include "LocalBAGeometry.h"
namespace UCA_SLAM
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class LocalBAV3;

enum UcaLocalBAVisualObsStatus
{
    UCA_LOCALBA_VISUAL_OBS_ACTIVE = 0,
    UCA_LOCALBA_VISUAL_OBS_PROBATION = 1,
    UCA_LOCALBA_VISUAL_OBS_SUPPRESSED = 2
};

enum UcaLocalBAVisualObsLookupFailure
{
    UCA_LOCALBA_VISUAL_OBS_LOOKUP_OK = 0,
    UCA_LOCALBA_VISUAL_OBS_FEATURE_INDEX_OOB = 1,
    UCA_LOCALBA_VISUAL_OBS_STATE_INDEX_OOB = 2,
    UCA_LOCALBA_VISUAL_OBS_MAPPOINT_MISMATCH = 3,
    UCA_LOCALBA_VISUAL_OBS_STATE_DECISION_CHANGED = 4
};

struct UcaLocalBAVisualObsState
{
    unsigned char status;
    unsigned char consecutive_bad;
    unsigned char consecutive_good;
    unsigned int suppressed_commits;
    unsigned long last_update_anchor_id;
};

struct UcaLocalBAV3LidarAssociation
{

    std::size_t source_index;
    VOXEL_LOC voxel;
    int layer;
    int pid_hint;
    V3D normal;
    V3D center;
    double d;
};

using UcaLocalBAV3LidarAssociationVector = std::vector<
    UcaLocalBAV3LidarAssociation,
    Eigen::aligned_allocator<UcaLocalBAV3LidarAssociation>>;

enum class UcaLidarSourceKind : unsigned char
{
    None = 0,
    SurfaceDense = 1,
    SurfaceSparse = 2,
    Rebuild = 3,
    BirthBalm = 4
};

struct UcaLidarSourceDescriptor
{
    UcaLidarSourceKind kind;
    std::uint64_t generation;
    std::size_t raw_count;
    std::size_t finite_count;
    std::uint64_t deterministic_digest;
};

bool UseKeyFrameBirthBalmAssociations();

struct UcaLidarSelectedPoint
{

    V3D body_point;
    std::size_t logical_finite_index;
    std::size_t raw_source_index;
    int birth_plane_id;
};

using UcaLidarSelectedPointVector = std::vector<
    UcaLidarSelectedPoint,
    Eigen::aligned_allocator<UcaLidarSelectedPoint>>;

struct UcaSourcePlaneFitOptions
{
    int minimum_support;
    int maximum_support;
    double radius;
    double maximum_query_distance;
    double ranging_cov;
    double angle_cov;
    bool require_support_covariance;
};

struct UcaSourcePlaneFitResult
{

    bool valid;
    int support_count;
    double neighbor_radius;
    double maximum_distance;
    V3D centroid;
    V3D eigenvalues;
    V3D normal;
    double d;
    double fit_rms;
    double planarity_ratio;
    std::uint64_t source_generation;
    std::shared_ptr<const LocalPlaneMeasurementSnapshot> measurement;
};

struct UcaSourcePlaneCloudSnapshot
{

    PLVf(3) points_xc;
    std::uint64_t source_generation;
    unsigned long source_frame_id;
    MD(4,4) sensor_to_camera;
    bool valid;
};

struct UcaSourcePlaneCloudSpatialIndex
{
    using CellKey = std::array<int,3>;
    double cell_size;
    std::uint64_t source_generation;
    std::size_t point_count;
    std::map<CellKey, std::vector<std::size_t>> cells;
    bool valid;
};

struct UcaLocalBAV3HistoricalSketchPoint
{

    V3D body_point;
    M3D body_covariance;
    std::size_t raw_source_index;
};

using UcaLocalBAV3HistoricalSketch = std::vector<
    UcaLocalBAV3HistoricalSketchPoint,
    Eigen::aligned_allocator<UcaLocalBAV3HistoricalSketchPoint>>;

class KeyFrame
{
    friend class LocalBAV3;

    friend class LocalBAContractTestAccess;

public:

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB, const bool bWithLidar, const bool bFusion);

    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    cv::Mat GetConstPoseCov() const;

    void ComputeBoW();

    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);
    bool GetUcaLocalBAVisualObsState(
        const size_t idx,
        MapPoint* expected_map_point,
        UcaLocalBAVisualObsState& state,
        int* lookup_failure );
    bool SetUcaLocalBAVisualObsState(
        const size_t idx,
        MapPoint* expected_map_point,
        const UcaLocalBAVisualObsState& state,
        int* lookup_failure );

    cv::Mat GetPoseCov();
    void SetPoseCov(const cv::Mat &poseCov_);
    PoseCovarianceMetadata GetPoseCovMetadata();

    std::vector<double> GetDepthUncertainty();
    bool GetDepthUncertaintyAt(const size_t idx, double& value);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    bool IsInImage(const float &x, const float &y) const;

    void SetNotErase();
    void SetErase();

    void SetBadFlag();
    bool isBad();

    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b);

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2);

public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    MD(4,4) mLidarTvl;

    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    long unsigned int mnLiDARBALocalForKF;

    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    const int N;

    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;

    const std::vector<VisualCovariance2DStorage> mvUcaLocationCovariances;
    const std::vector<VisualCovariance2DStorage> mvUcaMatchingCovariances;
    const std::vector<float> mvuRight;
    const std::vector<float> mvDepth;
    std::vector<UcaLidarDepthCandidate> mvUcaLidarDepthCandidates;
    const cv::Mat mDescriptors;

    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    cv::Mat mTcp;

    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    PCMap lidar_clusters_;
    CovZMap lidar_cluster_covz_;
    ClusterPointCovMap lidar_cluster_point_covs_;
    std::unordered_map<int, VOXEL_LOC>    lidar_plane_vox_;
    WMap lidar_plane_w_;
    void ComputeLidarPlanesOnce(int Nmin , double abs_gap_th , double rel_gap_th , int max_planes , bool use_gershgorin );

    KFPlaneLocalVec GetLidarPlanesLocalFrozenCopy() const;

    void SetLidarRebuildCloud(const PointCloud::Ptr& cloud, size_t max_points);
    void GetLidarRebuildPointsBody(PointVector& out) const;
    size_t LidarRebuildPointCount() const;
    void SetLidarBASurfaceDenseBody(const PointCloud::Ptr& cloud,
                                    size_t max_points,
                                    double voxel_size);
    void GetLidarBASurfaceDenseBody(PLVf(3)& out) const;

    bool SetBirthBalmAssociations(const PLVf(3)& points,
                                  const std::vector<int>& plane_ids);

    bool FitLocalSourcePlane(const V3D& query_xc,
                             UcaSourcePlaneFitResult& result,
                             const UcaSourcePlaneFitOptions& options) const;
    bool CopyLocalSourcePlaneCloudSnapshot(
        UcaSourcePlaneCloudSnapshot& snapshot) const;
    static bool FitLocalSourcePlaneFromSnapshot(
        const UcaSourcePlaneCloudSnapshot& snapshot,
        const V3D& query_xc,
        UcaSourcePlaneFitResult& result,
        const UcaSourcePlaneFitOptions& options);
    static bool BuildLocalSourcePlaneCloudSpatialIndex(
        const UcaSourcePlaneCloudSnapshot& snapshot,
        double cell_size,
        UcaSourcePlaneCloudSpatialIndex& index);
    static bool FitLocalSourcePlaneFromSnapshotIndexed(
        const UcaSourcePlaneCloudSnapshot& snapshot,
        const UcaSourcePlaneCloudSpatialIndex& index,
        const V3D& query_xc,
        UcaSourcePlaneFitResult& result,
        const UcaSourcePlaneFitOptions& options);
    bool HasLidarBASurfaceDense() const;
    bool HasLidarBASurfaceSparse() const;

    void DemoteLidarBASurfaceDenseToSparse(size_t max_points, double voxel_size);
    void ClearLidarBASurfaceDense();
    bool DescribeLocalBAV3LidarSource(UcaLidarSourceDescriptor& descriptor) const;
    bool DescribeLidarSource(UcaLidarSourceKind kind,
                             UcaLidarSourceDescriptor& descriptor) const;
    bool CopyLocalBAV3LidarSelectedPoints(
        const UcaLidarSourceDescriptor& expected,
        const std::vector<std::size_t>& logical_finite_indices,
        UcaLidarSelectedPointVector& selected) const;
    bool GetOrComputeLocalBAV3BodyCovariance(
        const UcaLidarSourceDescriptor& expected,
        std::size_t raw_source_index,
        const V3D& body_point,
        double ranging_cov,
        double angle_cov,
        bool strict_validation,
        M3D& covariance,
        bool& covariance_valid,
        bool& cache_hit) const;
    bool CopyLocalBAV3HistoricalSketch(
        const UcaLidarSourceDescriptor& expected,
        std::size_t maximum_points,
        double ranging_cov,
        double angle_cov,
        UcaLocalBAV3HistoricalSketch& sketch,
        bool& cache_hit) const;
    std::uint64_t GetLidarSourceGeneration() const;
    std::size_t LocalBAV3BodyCovarianceCacheSize() const;
    std::size_t LocalBAV3HistoricalSketchCacheSize() const;
    void MarkLidarBAFactorRound(unsigned long round);
    unsigned long GetLastLidarBAFactorRound() const;
    bool GetLocalBAV3LidarAssociationCache(
        const std::string& source,
        std::uint64_t source_generation,
        std::size_t source_point_count,
        UcaLocalBAV3LidarAssociationVector& associations,
        unsigned long& committed_anchor_id) const;
    void MergeLocalBAV3LidarAssociationCache(
        const std::string& source,
        std::uint64_t source_generation,
        std::size_t source_point_count,
        const UcaLocalBAV3LidarAssociationVector& associations,
        unsigned long committed_anchor_id,
        std::size_t maximum_entries);
    void ClearLocalBAV3LidarAssociationCache();
    void ReleaseLidarAssociationCaches();
    void ReplaceLidarAssociationCaches(PCMap&& clusters,
                                       CovZMap&& covz,
                                       ClusterPointCovMap&& point_covs,
                                       std::unordered_map<int, VOXEL_LOC>&& plane_vox,
                                       WMap&& plane_w,
                                       unsigned long version);
    void SetLidarAssociationMapVersion(unsigned long version);

    bool HasCurrentLidarAssociation(unsigned long version) const;
    bool HasStaleLidarAssociation(unsigned long version) const;
    bool MarkLidarAssociationRebuildAttempt(unsigned long version);
    unsigned long GetLastLidarAssociationRebuildAttemptVersion() const;
    void ReleaseLidarRebuildCloud();
    void ReleaseLidarCaches();
private:
    struct LocalBAV3BodyCovarianceCacheKey
    {
        UcaLidarSourceKind source_kind;
        std::uint64_t source_generation;
        std::size_t raw_source_index;
        double ranging_cov;
        double angle_cov;
        bool strict_validation;

        bool operator==(const LocalBAV3BodyCovarianceCacheKey& other) const;
    };

    struct LocalBAV3BodyCovarianceCacheKeyHash
    {
        std::size_t operator()(
            const LocalBAV3BodyCovarianceCacheKey& key) const;
    };

    struct LocalBAV3BodyCovarianceCacheValue
    {

        V3D body_point;
        M3D covariance;
        bool covariance_valid;
    };

    struct LocalBAV3HistoricalSketchCache
    {
        bool valid;
        UcaLidarSourceDescriptor source;
        std::size_t maximum_points;
        double ranging_cov;
        double angle_cov;
        UcaLocalBAV3HistoricalSketch points;
    };

    void AdvanceLidarSourceGenerationLocked();

    mutable std::mutex  mMutexLidarCaches;
    bool mbBirthBalmCaptured;
    PLVf(3) mvBirthBalmPointsBody;
    std::vector<int> mvBirthBalmPlaneIds;
    KFPlaneLocalVec     planes_local_frozen_;
    std::atomic<bool>   planes_frozen_;
    PointVector         mvLidarRebuildPointsBody;
    PLVf(3)             mvLidarBASurfaceDenseBody;
    PLVf(3)             mvLidarBASurfaceSparseBody;
    std::uint64_t       mnLidarSourceGeneration;
    mutable std::unordered_map<
        LocalBAV3BodyCovarianceCacheKey,
        LocalBAV3BodyCovarianceCacheValue,
        LocalBAV3BodyCovarianceCacheKeyHash>
        mLocalBAV3BodyCovarianceCache;
    mutable LocalBAV3HistoricalSketchCache
        mLocalBAV3HistoricalSketchCache;
    bool                mbLidarBADenseResident;
    unsigned long       mnLastLidarBAFactorRound;
    std::string         mLocalBAV3LidarAssocSource;
    std::uint64_t       mnLocalBAV3LidarAssocSourceGeneration;
    std::size_t         mnLocalBAV3LidarAssocSourcePointCount;
    unsigned long       mnLocalBAV3LidarAssocCommittedAnchorId;
    UcaLocalBAV3LidarAssociationVector mvLocalBAV3LidarAssociations;
    unsigned long       mnLidarAssocMapVersion;
    unsigned long       mnLastLidarAssocRebuildAttemptVersion;

protected:

    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat poseCov;
    PoseCovarianceMetadata mPoseCovMetadata;
    std::vector<double> mvDepthUncertainty;

    std::vector<MapPoint*> mvpMapPoints;

    std::vector<UcaLocalBAVisualObsState> mvUcaLocalBAVisualObsStates;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    Map* mpMap;

    mutable std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;

    mutable std::mutex mMutexCov;
};

}

#endif
