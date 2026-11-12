// Declaration-only preview. Function implementations are not included.
#ifndef UCA_SLAM_VISUAL_UNCERTAINTY_MODEL_H
#define UCA_SLAM_VISUAL_UNCERTAINTY_MODEL_H

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace UCA_SLAM
{

class Frame;
class KeyFrame;

struct VisualCovariance2DStorage
{
    double xx;
    double xy;
    double yy;
    bool valid;
};

enum class LocationUncertaintyMode
{
    LegacyFixedLocation = 0,
    OrbLevelLocation = 1,
    OrbPlusConstantV1 = 2,
    LearnedLocation = 3
};

enum class MatchingUncertaintyMode
{
    LegacyMatching = 0,
    FixedMatching = 1,
    LearnedMatching = 2
};

const char* LocationUncertaintyModeName(LocationUncertaintyMode mode);
const char* MatchingUncertaintyModeName(MatchingUncertaintyMode mode);
bool ParseLocationUncertaintyMode(const std::string& text,
                                  LocationUncertaintyMode& mode);
bool ParseMatchingUncertaintyMode(const std::string& text,
                                  MatchingUncertaintyMode& mode);

struct VisualObservationUncertainty2D
{

    Eigen::Matrix2d location_covariance;
    Eigen::Matrix2d matching_covariance;
    Eigen::Matrix2d total_covariance;
    bool valid;
    std::string provenance;
    std::string fallback_reason;
};

class KeypointLocationUncertaintyProvider
{
public:
    KeypointLocationUncertaintyProvider(
        LocationUncertaintyMode mode,
        double kappa_location ,
        double extra_variance_px2 );

    static KeypointLocationUncertaintyProvider ForMapPointBirth();
    static KeypointLocationUncertaintyProvider ForCurrentObservation();

    bool GetLocationCovariance(const Frame& frame,
                               std::size_t keypoint_id,
                               Eigen::Matrix2d& covariance,
                               std::string* provenance ) const;
    bool GetLocationCovariance(const KeyFrame& keyframe,
                               std::size_t keypoint_id,
                               Eigen::Matrix2d& covariance,
                               std::string* provenance ) const;

private:
    bool BuildCovariance(int octave,
                         const std::vector<float>& level_sigma2,
                         const std::vector<VisualCovariance2DStorage>& stored,
                         std::size_t keypoint_id,
                         Eigen::Matrix2d& covariance,
                         std::string* provenance) const;

    LocationUncertaintyMode mode_;
    double kappa_location_;
    double extra_variance_px2_;
};

class MatchingUncertaintyProvider
{
public:
    MatchingUncertaintyProvider(
        MatchingUncertaintyMode mode,
        double kappa_matching ,
        double fixed_variance_px2 );

    static MatchingUncertaintyProvider ForCurrentObservation();

    bool GetMatchingCovariance(const Frame& frame,
                               std::size_t keypoint_id,
                               Eigen::Matrix2d& covariance,
                               std::string* provenance ) const;
    bool GetMatchingCovariance(const KeyFrame& keyframe,
                               std::size_t keypoint_id,
                               Eigen::Matrix2d& covariance,
                               std::string* provenance ) const;

private:
    bool BuildCovariance(
        const std::vector<VisualCovariance2DStorage>& stored,
        std::size_t keypoint_id,
        Eigen::Matrix2d& covariance,
        std::string* provenance) const;

    MatchingUncertaintyMode mode_;
    double kappa_matching_;
    double fixed_variance_px2_;
};

bool BuildCurrentVisualObservationUncertainty(
    const Frame& frame,
    std::size_t keypoint_id,
    VisualObservationUncertainty2D& output);
bool BuildCurrentVisualObservationUncertainty(
    const KeyFrame& keyframe,
    std::size_t keypoint_id,
    VisualObservationUncertainty2D& output);

bool LiftVisualCovarianceToStereo(
    const Eigen::Matrix2d& visual_covariance,
    const Eigen::Matrix3d& retained_stereo_covariance,
    bool pseudo_stereo,
    Eigen::Matrix3d& stereo_covariance);

bool GetMapPointBirthLocationCovariance(
    const Frame& frame,
    std::size_t keypoint_id,
    Eigen::Matrix2d& covariance,
    std::string* provenance );
bool GetMapPointBirthLocationCovariance(
    const KeyFrame& keyframe,
    std::size_t keypoint_id,
    Eigen::Matrix2d& covariance,
    std::string* provenance );

bool MapPointBirthOrbPlusConstantV1Enabled();
double MapPointBirthVisualExtraVariancePx2();
double MapPointBirthVisualVariancePx2(
    int octave,
    const std::vector<float>& level_sigma2);

}

#endif
