// Declaration-only preview. Function implementations are not included.
#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace UCA_SLAM
{

class PnPsolver {
 public:
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();

  void SetRansaUCArameters(double probability , int minInliers  , int maxIterations , int minSet , float epsilon ,
                           float th2 );

  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

  void CheckInliers();
  bool Refine();

  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  double reprojection_error(const double R[3][3], const double t[3]);

  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);

  double uc, vc, fu, fv;

  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  double cws[4][3], ccs[4][3];

  vector<MapPoint*> mvpMapPointMatches;

  vector<cv::Point2f> mvP2D;
  vector<float> mvSigma2;

  vector<cv::Point3f> mvP3Dw;

  vector<size_t> mvKeyPointIndices;

  double mRi[3][3];
  double mti[3];
  vector<bool> mvbInliersi;
  int mnInliersi;

  int mnIterations;
  vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  cv::Mat mRefinedTcw;
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  int N;

  vector<size_t> mvAllIndices;

  double mRansacProb;

  int mRansacMinInliers;

  int mRansacMaxIts;

  float mRansacEpsilon;

  int mRansacMinSet;

  vector<float> mvMaxError;

};

}

#endif
