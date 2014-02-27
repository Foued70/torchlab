// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2010, 2011, 2012 Google Inc. All rights reserved.
// http://code.google.com/p/ceres-solver/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

extern "C" {
#include <TH.h>
#include <luaT.h>
}

#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/gpu/gpu.hpp"


using namespace cv;
using namespace std;
// Input a quaternion as the camera, representating a rigid transformation and translation
struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y, double observed_z)
  : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z) {}
  template <typename T>
  bool operator()(const T* const camera,
    const T* const point,
    T* residuals) const {

    // camera[0,1,2] are the angle-axis rotation.  
    T p[3];
    T quat[4];
    T norm = sqrt(camera[0]*camera[0] + camera[1]*camera[1]+camera[2]*camera[2]+camera[3]*camera[3]);
    quat[0] = camera[0]/norm;
    quat[1] = camera[1]/norm;
    quat[2] = camera[2]/norm;
    quat[3] = camera[3]/norm;

    ceres::UnitQuaternionRotatePoint(quat, point, p);    // camera[3,4,5] are the translation.
    p[0] += camera[4]; p[1] += camera[5]; p[2] += camera[6];


    // The error is the difference between the predicted and observed position.
    residuals[0] = p[0] - T(observed_x);
    residuals[1] = p[1] - T(observed_y);
    residuals[2] = p[2]- T(observed_z);

    return true;
  }

  double observed_x;
  double observed_y;
  double observed_z;

};

LUA_EXTERNC DLL_EXPORT void minimize_global_pair_dis(int* cam_index, int ncam, 
  int* pt_index,  int npts, 
  double *obs,  int nobs,
  double *params, int nparams){

  int i;

  cout << "npoints: " << npts << endl;
  cout << "ncameras: " << ncam << endl;
  cout << "nobservations: " << nobs << endl;
  cout << "nparams: " << nparams << endl;
  if (nparams != 7 * ncam + 3 * npts){
    cerr << "Error wrong number of parameters" << endl;
    return;
  }
  double *mutable_cameras = params;
  double *mutable_points  = params + 7 * ncam;

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < nobs; ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<ReprojectionError, 3, 7, 3>(
      new ReprojectionError(obs[3 * i + 0], obs[3 * i + 1], obs[3 * i + 2]));

    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
     mutable_cameras + cam_index[i] * 7,
     mutable_points  +  pt_index[i] * 3);
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.FullReport() << "\n";

}


struct ReprojectionErrorForRigidTransformation {
  ReprojectionErrorForRigidTransformation(double observed_x, double observed_y, double observed_z,
    double predicted_x, double predicted_y, double predicted_z)
  : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z),
  predicted_x(predicted_x), predicted_y(predicted_y), predicted_z(predicted_z) { 
  }

  template <typename T>
  bool operator()(const T* const camera, T* residuals) const {
    // The error is the difference between the predicted and observed position.
    T p[3];
    T point[3];
    point[0] = T(observed_x);
    point[1] = T(observed_y);
    point[2] = T(observed_z);
    T quat[4];
    T norm = sqrt(camera[0]*camera[0] + camera[1]*camera[1]+camera[2]*camera[2]+camera[3]*camera[3]);
    quat[0] = camera[0]/norm;
    quat[1] = camera[1]/norm;
    quat[2] = camera[2]/norm;
    quat[3] = camera[3]/norm;

    ceres::UnitQuaternionRotatePoint(quat, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[4]; p[1] += camera[5]; p[2] += camera[6];


    // The error is the difference between the predicted and observed position.

    residuals[0] = ((p[0]-T(predicted_x))*(p[0]-T(predicted_x)));
    residuals[1] = ((p[1]-T(predicted_y))*(p[1]-T(predicted_y)));
    residuals[2] = ((p[2]-T(predicted_z))*(p[2]-T(predicted_z)));

    return true;
  }

  double observed_x;
  double observed_y;
  double observed_z;
  double predicted_x;
  double predicted_y;
  double predicted_z;
};

LUA_EXTERNC DLL_EXPORT void predict_best_rigid_transformation(
  double *params, int nparams, 
  Mat* orig_location, Mat* goal_location){

  int i;

  if (nparams != 7 ){
    cerr << "Error wrong number of parameters" << endl;
    return;
  }
  double *mutable_camera = params;

  cout << mutable_camera[0] << mutable_camera[1] << mutable_camera[2] << mutable_camera[3] << mutable_camera[4] << endl;
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for(int i=0; i<orig_location->rows; i++) {
    ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<ReprojectionErrorForRigidTransformation, 3, 7>(
      new ReprojectionErrorForRigidTransformation(orig_location->at<double>(i, 0), 
        orig_location->at<double>(i, 1),
        orig_location->at<double>(i, 2),
        goal_location->at<double>(i, 0),
        goal_location->at<double>(i, 1),
        goal_location->at<double>(i, 2)));
    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
     mutable_camera);
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.FullReport() << "\n";

}

void  printMinAndMax(Mat test) {

  Point minLoc; Point maxLoc;
  double minVal; double maxVal; 
  minMaxLoc( test, &minVal, &maxVal, &minLoc, &maxLoc);
  cout << "min " << minVal << " max " << maxVal << endl;
}

void getDepthImage(Mat points, Mat transformM, Mat result)
{
  double size_x = 900;
  double size_y = 600;
  Mat concatWithOnes;

  vconcat(points.t(),Mat::ones(1,points.rows, points.type()), concatWithOnes);

  Mat pts = (transformM)*concatWithOnes;

  pts = (pts.rowRange(0,3)).t();

  Mat exp1, exp2, exp3, dis;
  pow(pts.col(0),2, exp1);
  pow(pts.col(1),2, exp2);
  pow(pts.col(2),2, exp3);
  sqrt(exp1+exp2+exp3, dis);  

  Mat azimuth;
  phase(pts.col(0), pts.col(1), azimuth);


  azimuth = azimuth*-1+M_PI*2;

  azimuth = azimuth * (size_x-1)/(2.0*M_PI)+1.0;

  Mat elevation;
  Mat opposite,dis_sq;
  pow(dis, 2, dis_sq);


  sqrt(dis_sq-exp3,opposite);

  phase(pts.col(2), opposite, elevation);

  elevation = elevation * (size_y-1)/(150.0/180.0*M_PI)+1.0;

  Mat goodMat;
  bitwise_and((azimuth>=1) , (azimuth<=size_x-1), goodMat);
  bitwise_and(goodMat, (elevation>=1), goodMat);
  bitwise_and(goodMat,(elevation<=size_y-1), goodMat);
  Mat result_test = Mat::zeros(result.rows, result.cols, CV_8U);
  for (int i=0; i<pts.rows; i++) {
    if(goodMat.at<bool>(i,0)) {
      if (result_test.at<char>((int)elevation.at<double>(i,0), (int)azimuth.at<double>(i,0)) ==0) {
        result.at<double>((int)elevation.at<double>(i,0), (int)azimuth.at<double>(i,0)) = dis.at<double>(i,0); 
        result_test.at<char>((int)elevation.at<double>(i,0), (int)azimuth.at<double>(i,0)) =1;
      } else {
        double distance = result.at<double>((int)elevation.at<double>(i,0), (int)azimuth.at<double>(i,0));
        if (distance < (dis.at<double>(i,0))) {
          result.at<double>((int)elevation.at<double>(i,0), (int)azimuth.at<double>(i,0)) = distance; 
        } 
      }
    }
  }
}
double getScore(Mat f1, Mat f2, double* closenessScore) {

  Mat mask;
  multiply((f2 != 0), (f1!=0), mask);
  Mat f1_zeroed, f2_zeroed;

  mask.convertTo(mask,f1.type());
  multiply(f1, mask/255.0, f1_zeroed);
  multiply(f2, mask/255.0, f2_zeroed);


  double s1;
  Mat diff;
  subtract(f1_zeroed, f2_zeroed, diff);  

  Mat mask2;
  Mat good_pts_mask;
  double maxValue = 500.0;
  double minValue = 10.0;
  Mat mask_too_big = (diff >maxValue)/255.0;
  multiply((diff >= 0)/255.0,  (diff<=maxValue)/255.0, mask2);

  multiply((diff < minValue)/255.0,  (diff>=-minValue)/255.0, good_pts_mask);

  mask_too_big.convertTo(mask_too_big,diff.type());

  mask2.convertTo(mask2,f1.type());

  multiply(diff, mask2, diff);
  multiply(diff, mask/255.0, diff);

  diff = (diff+mask_too_big*maxValue); //25.0;
  multiply(diff,diff,diff);
  diff = diff/100;
  //exp(diff/15.0, diff);
  //diff = diff-exp(1);

  //diff = (diff - 1)/1000;


  s1 = sum(diff)[0];
  mask.convertTo(mask,good_pts_mask.type());

  multiply(good_pts_mask, mask/255.0, good_pts_mask);
  *closenessScore  = 1/(sum(good_pts_mask)[0]+1)*pow(10,10);
  return s1;
}

struct PairwiseError {
  PairwiseError(Mat points, Mat other_points, int i, int j)
  : points(points), other_points(other_points), i(i), j(j) {}

  bool operator()(const double* const camera1,
    const double* const camera2,
    double* residuals) const {
    Mat E = (Mat::zeros(4,4,CV_64F));
    Mat H2 = (Mat::zeros(4,4,CV_64F));
    Mat H1 = (Mat::zeros(4,4,CV_64F));
    
    E.at<double>(0,0) = 1;
    E.at<double>(1,1) = 1;
    E.at<double>(2,2) = 1;
    E.at<double>(3,3) = 1;


    double quat1[4];
    double norm1 = sqrt(camera1[0]*camera1[0] + camera1[1]*camera1[1]+camera1[2]*camera1[2]+camera1[3]*camera1[3]);
    quat1[0] = camera1[0]/norm1;
    quat1[1] = camera1[1]/norm1;
    quat1[2] = camera1[2]/norm1;
    quat1[3] = camera1[3]/norm1;

    double R1[3*3];
    ceres::QuaternionToScaledRotation(quat1, R1);    // camera[3,4,5] are the translation.


    double quat2[4];
    double norm2 = sqrt(camera2[0]*camera2[0] + camera2[1]*camera2[1]+camera2[2]*camera2[2]+camera2[3]*camera2[3]);
    quat2[0] = camera2[0]/norm2;
    quat2[1] = camera2[1]/norm2;
    quat2[2] = camera2[2]/norm2;
    quat2[3] = camera2[3]/norm2;
    double R2[3*3];
    ceres::QuaternionToScaledRotation(quat2, R2);    // camera[3,4,5] are the translation.



    H1.at<double>(0,0) = (double)R1[0];
    H1.at<double>(0,1) = (double)R1[1];
    H1.at<double>(0,2) = (double)R1[2];

    H1.at<double>(1,0) = (double)R1[3];
    H1.at<double>(1,1) = (double)R1[4];
    H1.at<double>(1,2) = (double)R1[5];


    H1.at<double>(2,0) = (double)R1[6];
    H1.at<double>(2,1) = (double)R1[7];
    H1.at<double>(2,2) = (double)R1[8];

    H1.at<double>(0,3) = (double)camera1[4];
    H1.at<double>(1,3) = (double)camera1[5];
    H1.at<double>(2,3) = (double)camera1[6];

    H1.at<double>(3,3) = 1;

    H2.at<double>(0,0) = (double)R2[0];
    H2.at<double>(0,1) = (double)R2[1];
    H2.at<double>(0,2) = (double)R2[2];

    H2.at<double>(1,0) = (double)R2[3];
    H2.at<double>(1,1) = (double)R2[4];
    H2.at<double>(1,2) = (double)R2[5];


    H2.at<double>(2,0) = (double)R2[6];
    H2.at<double>(2,1) = (double)R2[7];
    H2.at<double>(2,2) = (double)R2[8];

    H2.at<double>(0,3) = (double)camera2[4];
    H2.at<double>(1,3) = (double)camera2[5];
    H2.at<double>(2,3) = (double)camera2[6];

    H2.at<double>(3,3) = 1;

    Mat H2_to_H1 = H1*H2.inv();

    double size_x = 900;
    double size_y = 600;

    Mat f1(Mat::zeros((int)size_y, (int)size_x,CV_64F));
    Mat f2(Mat::zeros((int)size_y, (int)size_x,CV_64F));

    getDepthImage(points, E, f1); 
    getDepthImage(other_points, H2_to_H1, f2);
    double score1, score2;
    double residuals0 = getScore(f1, f2, &score1); //exp(1-pow(getScore(f1, f2),2))-1; //sum of     
    Mat f3(Mat::zeros((int)size_y, (int)size_x,CV_64F));
    Mat f4(Mat::zeros((int)size_y, (int)size_x,CV_64F));

    getDepthImage(other_points, E, f3);
    getDepthImage(points, H2_to_H1.inv(), f4);


    double residuals1 = getScore(f3, f4, &score2); // exp(1-pow(getScore(f3, f4),2))-1;
    residuals[0] = max(residuals0, residuals1);
    residuals[1] = max(score1, score2);
    
    std::cout << "residuals at " << i << " " << j << " are: -----" << residuals[0] << " " << residuals[1] << std::endl;

    return true;
  }

  Mat points, other_points;
  int i,j;
};



LUA_EXTERNC DLL_EXPORT void minimize_pairwise_alignment_error(Mat *obs1, Mat *obs2, 
  double *params, int nparams){

  cout << "nparams: " << nparams << endl;

  double *mutable_cameras = params;

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
  int i=0;
  ceres::CostFunction* cost_function =
  new ceres::NumericDiffCostFunction<PairwiseError, ceres::CENTRAL,2,7,7>(new PairwiseError(*obs1, *obs2, i, i+1));

  problem.AddResidualBlock(cost_function,
                           NULL /* squared loss */,
   mutable_cameras + i * 7,
   mutable_cameras + (i+1) * 7);


  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.linear_solver_type = ceres::DENSE_SCHUR;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.FullReport() << "\n";

}


LUA_EXTERNC DLL_EXPORT  void minimize_global_alignment_error(Mat *points, int *start_index, int *end_index,
  double *params, int ncams, Mat* scores) {

  double *mutable_cameras = params;

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < ncams; ++i) {
    for (int j=i+1; j<ncams; ++j) {
      if(scores->at<int>(i,j) == 1) {
        cout << "good " << i << " good " << j << endl;
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
        ceres::CostFunction* cost_function =
        new ceres::NumericDiffCostFunction<PairwiseError, ceres::CENTRAL,2,7,7>(
          new PairwiseError(points->rowRange(start_index[i],end_index[i]+1), 
            points->rowRange(start_index[j],end_index[j]+1), i, j));

        problem.AddResidualBlock(cost_function,
                               NULL /* squared loss */,
         mutable_cameras + i * 7,
         mutable_cameras + j* 7);
      }
    }

  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.FullReport() << "\n";

}


struct RampError {
  RampError(double observed_x, double observed_y, double observed_z, 
      double observed_nx, double observed_ny, double observed_nz,

    int ncam)
  : observed_x(observed_x), observed_y(observed_y), observed_z(observed_z), 
    observed_nx(observed_nx), observed_ny(observed_ny), observed_nz(observed_nz), ncam(ncam) {}
  template <typename T>
  bool operator()(const T* const plane,
    T* residuals) const {
    // The error is the difference between the predicted and observed distance.
    T minSoFar = (T)10000000000;
    T valSoFar = (T)10000000000;
    T valSoFarAngle = (T)10000000000;
    T multiplier = (T)0;//.5*360/(2*M_PI);
    T multiplier_error = (T)1;

    int bestI =10;
    for(int i=0; i<ncam; i++) {
      T norm = sqrt(plane[4*i]*plane[4*i] + plane[4*i+1]*plane[4*i+1]+plane[4*i+2]*plane[4*i+2]);
      T error_i = ((T)observed_x*plane[4*i]+(T)observed_y*plane[4*i+1]+(T)observed_z*plane[4*i+2])/norm+(T)plane[4*i+3];
      
      T cosTheta = ((T)observed_nx*plane[4*i]+(T)observed_ny*plane[4*i+1]+(T)observed_nz*plane[4*i+2])/norm;
      T angle_between = acos(cosTheta);

      T error_combo = abs(error_i); //(abs(error_i)-10) > 0 ? (abs(error_i)-10) : 0;
      //to do weigh these
      if(abs(error_combo) < minSoFar) { //*multiplier_error+abs(angle_between)*multiplier < minSoFar) {
        minSoFar = error_combo; //*multiplier_error+abs(angle_between)*multiplier ;
        valSoFar = error_i;
        valSoFarAngle = abs(angle_between)>M_PI/6 ? abs(angle_between)-M_PI/6 : 0;
        bestI = i;
      }
    }
    //cout << bestI << " ";
    valSoFar = abs(valSoFar) > 10 ? abs(valSoFar)-10 : 0;
    residuals[0]=abs(valSoFar); //*abs(valSoFar); //*(abs(valSoFar)-10)*; residuals[1] = 0;
    residuals[1]=abs(valSoFarAngle*180/M_PI*.1); //*(abs(valSoFar)-10)*; residuals[1] = 0;
    return true;
  }

  double observed_x;
  double observed_y;
  double observed_z;
  double observed_nx;
  double observed_ny;
  double observed_nz;
  
  int ncam;
};

LUA_EXTERNC DLL_EXPORT void ramp_opt(double* points, double* normals, int npoints, double* planes, int ncam) {

  double *mutable_planes = planes;

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < npoints; ++i) {
    ceres::CostFunction* cost_function =
    new ceres::NumericDiffCostFunction<RampError, ceres::CENTRAL,2,20>(
      new RampError(points[3 * i + 0], points[3 * i + 1], points[3 * i + 2], 
          normals[3 * i + 0], normals[3 * i + 1], normals[3 * i + 2], 
          ncam));

    problem.AddResidualBlock(cost_function,
                               NULL /* squared loss */,
     mutable_planes);

  }

    // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.FullReport() << "\n";


  for(int j=0; j<ncam; j++) {
    double norm = sqrt(mutable_planes[4*j]*mutable_planes[4*j] + mutable_planes[4*j+1]*mutable_planes[4*j+1]+mutable_planes[4*j+2]*mutable_planes[4*j+2]);
    mutable_planes[4*j]=      mutable_planes[4*j]/norm;
    mutable_planes[4*j+1]=      mutable_planes[4*j+1]/norm;
    mutable_planes[4*j+2]=      mutable_planes[4*j+2]/norm;
  }
}
