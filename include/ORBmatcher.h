// Declaration-only preview. Function implementations are not included.
#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"

namespace UCA_SLAM
{

struct ProjectionMatchStats
{
    int localMapPoints;
    int skipAlreadySeen;
    int skipBadLocal;
    int frustumRejected;
    int initialAssigned;
    int inView;
    int projectedWithCandidates;
    int candidateFeatures;
    int freeCandidateFeatures;
    int occupiedSkipInitial;
    int occupiedSkipNew;
    int rejectDepthGate;
    int rejectDescriptor;
    int descriptorAccepted;
    int assigned;
    int bestFeatureConflicts;
};

class ORBmatcher
{
public:

    ORBmatcher(float nnratio, bool checkOri);

    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    static void PrintNoLaterObservationAttribution(
        const Frame& frame, MapPoint* mapPoint,
        const char* reason, const char* detail,
        float projectedX, float projectedY, float legacyRadius,
        int allowedCandidates, int allOctaveCandidates,
        int freeCandidates, int occupiedCandidates,
        int geometryRejected, int bestDistance, int secondDistance,
        int featureIndex );

    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints,
                           const float th, ProjectionMatchStats* pStats);

    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

     int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize);

    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs,
                               const bool bOnlyStereo,
                               const bool bMutualBest);

    int SearchForTriangulationDualOutputs(
        KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
        std::vector<pair<size_t, size_t> >& vForwardMatchedPairs,
        std::vector<pair<size_t, size_t> >& vMutualMatchedPairs,
        const bool bOnlyStereo);

    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th);

    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}

#endif
