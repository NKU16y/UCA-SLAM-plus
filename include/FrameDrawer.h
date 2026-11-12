// Declaration-only preview. Function implementations are not included.
#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include <vector>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>

namespace UCA_SLAM
{

class Tracking;
class Map;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    void Update(Tracking *pTracker);

    cv::Mat DrawFrame();

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    cv::Mat mIm;
    int N;
    std::vector<cv::KeyPoint> mvCurrentKeys;
    std::vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    std::vector<cv::KeyPoint> mvIniKeys;
    std::vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    std::mutex mMutex;
};

}

#endif
