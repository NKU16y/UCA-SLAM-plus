// Declaration-only preview. Function implementations are not included.
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "LidarTracking.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "UncertaintyUtil.h"
#include "LiDARIncludes/LiDARLoopPlaneRefine.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <atomic>
namespace UCA_SLAM
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations , bool *pbStopFlag, const unsigned long nLoopKF,
                                 const bool bRobust ,
                                 const std::atomic<bool>* pStopToken );
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool *pbStopFlag,
                                       const unsigned long nLoopKF, const bool bRobust ,
                                       const std::atomic<bool>* pStopToken );
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    int static PoseOptimization(Frame* pFrame);

    int static PoseOptimizationwithLidar(Frame* pFrame, const int &effct_feat_num, const std::vector<ptpl,Eigen::aligned_allocator<ptpl>>& ptpl_list, PLM(3) &point_covs, const LidarTracking* const pLT, vector<double> &allvisualWeights, vector<int> &inlierLidarIndices, vector<double> &inlierLidarWeights, MD(6,6)* pHybridPoseCovOut , bool* pHybridPoseCovValidOut , PoseCovarianceMetadata* pHybridPoseCovMetadataOut , TrackingVisualEdgeState* pVisualEdgeStateOut , bool* pHybridSolveSucceededOut );

    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    void static OptimizeEssentialGraphwithLiDAR(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

}

#endif
