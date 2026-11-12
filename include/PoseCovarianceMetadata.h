// Declaration-only preview. Function implementations are not included.
#ifndef UCA_SLAM_POSE_COVARIANCE_METADATA_H
#define UCA_SLAM_POSE_COVARIANCE_METADATA_H

#include <string>

namespace UCA_SLAM
{

enum class PoseCovarianceSource
{
    Unknown = 0,
    DefaultFallback = 1,
    LegacyTrackingHybrid = 2,
    TrackingFinalEffectiveSandwichV2 = 3,
    LocalBASchurV2 = 4,
    TrackingD6FinalConditionalV1 = 5,
    TrackingCommonReferenceScalarSandwichV1 = 6,
    SingleFrontendConditionalInformationV1 = 7,
    TrackingTwoWeightsConditionalIRLSV1 = 8
};

enum class PoseCovarianceConvention
{
    Unknown = 0,
    TcwLeftRotationTranslation = 1
};

struct PoseCovarianceMetadata
{
    int version;
    PoseCovarianceSource source;
    PoseCovarianceConvention convention;
    bool final_active_snapshot;
    bool clean_base_contract;
    bool scale_chain_contract;
    unsigned long source_frame_id;
};

inline const char* PoseCovarianceSourceName(
    const PoseCovarianceSource source);

inline const char* PoseCovarianceConventionName(
    const PoseCovarianceConvention convention);

inline bool IsTrackingPoseCovarianceV2(
    const PoseCovarianceMetadata& metadata);

}

#endif
