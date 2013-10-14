/* extern "C" makes the functions callable from C */
extern "C"
{
#include "TH.h"
}

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

void printParams( Algorithm* algorithm ) {
  vector<string> parameters;
  string param, helpText, typeText;
  int type;

  algorithm->getParams(parameters);
  cout << "Found " << parameters.size() << " parameters" << endl;
  for (int i = 0; i < (int) parameters.size(); i++) {
    param    = parameters[i];
    type     = algorithm->paramType(param);

    cout << " + '" << param << "' ";
    switch (type) {
      case Param::BOOLEAN:
      typeText = "bool";
      cout << "(" << typeText << ") " << algorithm->get<bool>(param);
      break;
      case Param::INT:
      typeText = "int";
      cout << "(" << typeText << ") " << algorithm->get<int>(param);
      break;
      case Param::REAL:
      typeText = "double";
      cout << "(" << typeText << ") " << algorithm->get<double>(param);
      break;
      case Param::STRING:
      typeText = "string";
      cout << "(" << typeText << ") " << algorithm->get<string>(param);
      break;
      case Param::MAT:
      typeText = "Mat";
      cout << "(" << typeText << ") " << algorithm->get<Mat>(param);
      break;
      case Param::ALGORITHM:
      typeText = "Algorithm";
      cout << "(" << typeText << ") " << algorithm->get<Algorithm>(param);
      break;
      case Param::MAT_VECTOR:
      typeText = "Mat vector";
      cout << "(" << typeText << ") " ; // << algorithm->get<vector<Mat> >(param);
      break;
      case Param::FLOAT:
      typeText = "float";
      cout << "(" << typeText << ") " << algorithm->getDouble(param);
      break;
      case Param::UNSIGNED_INT:
      typeText = "unsigned int";
      cout << "(" << typeText << ") " << algorithm->getInt(param);
      break;
      case Param::UINT64:
      typeText = "uint64";
      cout << "(" << typeText << ") " << algorithm->getInt(param);
      break;
      case Param::UCHAR:
      typeText = "uchar";
      cout << "(" << typeText << ") " << algorithm->getInt(param);
      break;
    }
    // string paramHelp = algorithm->paramHelp(param);
    // cout << paramHelp;
    cout << endl;
  }
}

extern "C"
{
  // return a torch style THLongStorage for size of mat
  void Mat_size(Mat* mat, THLongStorage* size)
  {
    int i;
    int dims     = mat->dims;
    int channels = mat->channels();
    if (channels == 1) {
      THLongStorage_resize(size,dims);
    } else {
      THLongStorage_resize(size,dims+1);
      THLongStorage_set(size,dims,channels);
    }
    for (i=0;i<dims;i++){
      THLongStorage_set(size,i,mat->size[i]);
    }
  }

  int  Mat_size0(Mat *mat) {
    return mat->size[0];
  }

  int  Mat_size1(Mat *mat) {
    return mat->size[1];
  }

  int  Mat_size2(Mat *mat) {
    return mat->size[2];
  }
  // return a torch style THLongStorage for stride of mat
  void Mat_stride(Mat* mat, THLongStorage* stride)
  {
    int i;
    int dims     = mat->dims;
    int channels = mat->channels();
    int elemsize = mat->elemSize1();

    if (channels > 1) {
      THLongStorage_resize(stride,dims+1);
      THLongStorage_set(stride,dims,1);
    } else {
      THLongStorage_resize(stride,dims);
    }
    for(i=0;i<mat->dims;i++){
      THLongStorage_set(stride,i,mat->step[i]/elemsize);
    }
  }
}

// toTensor and fromTensor code
#include "generic/opencv.cpp"
#include "THGenerateAllTypes.h"

extern "C" {
  Mat* Mat_create (int height, int width, int type)
  {
    return new Mat (width, height, type);
  }

  Mat* Mat_clone (Mat* mat)
  {
    // need to make a copy like mat->clone() but dynamic memory allocation.
    Mat * clone = new Mat(mat->dims,mat->size[0],mat->type());
    mat->copyTo(*clone);
    return clone;
  }
  // TODO move to image processing
  void Mat_convert(Mat* input, Mat* output, int cvt_type)
  {
    cvtColor(*input, *output, cvt_type);
  }

  // convert from int to enum in string form for enum < 31
  string enum_strings[] = {
    "CV_8UC1", "CV_8SC1", "CV_16UC1", "CV_16SC1", "CV_32SC1", "CV_32FC1", "CV_64FC1", "CV_USRTYPE1",
    "CV_8UC2", "CV_8SC2", "CV_16UC2", "CV_16SC2", "CV_32SC2", "CV_32FC2", "CV_64FC2", "CV_USRTYPE2",
    "CV_8UC3", "CV_8SC3", "CV_16UC3", "CV_16SC3", "CV_32SC3", "CV_32FC3", "CV_64FC3", "CV_USRTYPE3",
    "CV_8UC4", "CV_8SC4", "CV_16UC4", "CV_16SC4", "CV_32SC4", "CV_32FC4", "CV_64FC4", "CV_USRTYPE4"};

    int Mat_depth(Mat* mat)
    {
      return mat->depth();
    }

    void Mat_info(Mat* mat)
    {
      int i;
      cout << "dims:        " << mat->dims       << endl;
      cout << "channels:    " << mat->channels() << endl;
      cout << "depth:       " << enum_strings[mat->depth()] << endl;
      cout << "type:        " << enum_strings[mat->type()]  << endl;
      cout << "elemsize:    " << mat->elemSize()  << endl;
      cout << "elemsize1:   " << mat->elemSize1() << endl;
      cout << "total:       " << mat->total()     << endl;
      cout << "contiguous?: " << mat->isContinuous() << endl;
      if (mat->dims == 2) {
        cout << "rows:      " << mat->rows       << endl;
        cout << "cols:      " << mat->cols       << endl;
      }
      for (i=0;i<mat->dims;i++){
        cout << "size["<<i<<"]:   " << mat->size[i]   << endl;
      }
      for (i=0;i<mat->dims;i++){
        cout << "step["<<i<<"]:   " << mat->step[i]   << endl;
      }

    }

    int Mat_type(Mat* mat)
    {
      return mat->type();
    }
    Mat* Mat_loadImage(const char* fname)
    {
    // make sure that the pointer is dynamically allocated
      return new Mat(imread(fname));
    }

  // this is primarily for debugging as it interferes w/ luvit
    void Mat_showImage(Mat* mat, const char* wname)
    {
    // Create a window for display.
      namedWindow(wname, CV_WINDOW_AUTOSIZE );
      imshow(wname, *mat);
    }

    void Mat_destroy(Mat* mat)
    {
      delete(mat);
    }


  // -----------------------------
  // FeatureDetector
  // -----------------------------

  // function to sort the KeyPoints returned in DetectorExtractor
    struct KeyPointCompare {
      bool operator ()(const KeyPoint& a, const KeyPoint& b)
      const {return a.response>b.response;}
    };

    FeatureDetector* FeatureDetector_create(const char* detector_type)
    {
      Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
    detector.addref(); // make sure the Ptr stays around TODO: check memleak
    return detector;
  }

  void FeatureDetector_parameters(FeatureDetector* detector)
  {
    printParams((Algorithm*)detector);
  }

  int FeatureDetector_detect(FeatureDetector* detector,
   const Mat*  img,
   const Mat*  mask,
   KeyPoint*   keypointsC,
   int npts)
  {

    vector <KeyPoint> keypoints ;
    detector->detect( *img, keypoints, *mask);

    // Sort the keypoints by response.
    sort(keypoints.begin(), keypoints.end(), KeyPointCompare());

    if (npts > keypoints.size()) {
      npts = keypoints.size();
    }

    // memory at &keypoints[0] is guaranteed contiguous.
    memcpy(keypointsC,keypoints.data(),npts * sizeof(KeyPoint));

    return npts;
  }

  //return matrix representing the harris response at each point in src
  //note that FeatureDetector_detect with harris only returns the points, not the response!
  Mat* detectCornerHarris(Mat* src, int blockSize, int ksize, int k)
  {
    Mat returnMat; // = new Mat(src->size[0], src->size[1], src->type() );
    cornerHarris(*src, returnMat, blockSize, ksize, k);
    normalize( returnMat, returnMat, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

    return new Mat(returnMat);
  }

  //return a matrix instead of a keypoint list, representing x,y,response
  Mat* getMatFromKeypoints(const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);

    vector<Point3d> points;

    for(int i=0; i < keypoints.size(); i++){
      points.push_back(Point3d(keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].response));
    }
    return new Mat(points);
  }

  void FeatureDetector_destroy(FeatureDetector* detector)
  {
    // calling release() on the Ptr (rather than deleting it) calls the correct destructor for the object.
    ((Ptr<FeatureDetector>)detector).release();
  }

  void dump_keypoints(const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);

    for(int i=0; i < keypoints.size(); i++){
      printf("[%d] (%d, %d) %f\n",i,
       (int)keypoints[i].pt.x, (int)keypoints[i].pt.y,
       keypoints[i].response);
    }
  }

  void draw_keypoints(Mat* img, const KeyPoint* keyptr, int npts)
  {
    vector<KeyPoint> keypoints (keyptr, keyptr + npts);
    drawKeypoints( *img, keypoints, *img, Scalar::all(-1),
     DrawMatchesFlags::DEFAULT );
  }

  DescriptorExtractor* DescriptorExtractor_create(const char* feature_type)
  {
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(feature_type);
    extractor.addref(); // make sure the Ptr stays around TODO: check memleak
    return extractor;
  }

  void DescriptorExtractor_compute(DescriptorExtractor* extractor,
   const Mat*  img,
   Mat*  descriptor,
   KeyPoint*   keypointsC,
   int npts)
  {

    vector <KeyPoint> keypoints(npts);
    memcpy(keypoints.data(),keypointsC,npts * sizeof(KeyPoint));

    extractor->compute(*img, keypoints, *descriptor);

  }

  void DescriptorExtractor_destroy(DescriptorExtractor* extractor)
  {
    delete(extractor);
  }

  // -----------------------------
  // Edge detection
  // -----------------------------
  Mat* CannyDetectEdges(Mat* src, double threshold1, double threshold2)
  {
    Mat* dest = new Mat(0,0,src->type());
    Canny(*src, *dest, threshold1, threshold2);
    return dest;
  }

  // -----------------------------
  // Hough transform
  // -----------------------------

  Mat* HoughLinesRegular(Mat* image, double rho, double theta, int threshold, double srn, double stn)
  {
    Mat* lines = new Mat(0,0,image->type());;
    HoughLines(*image, *lines, rho, theta, threshold, srn, stn);
    return lines;
  }

  Mat* HoughLinesProbabilistic(Mat* image,double rho, double theta, int threshold, double minLineLength, double maxLineGap) 
  {
    Mat* lines = new Mat(0,0,image->type());
    HoughLinesP(*image, *lines, rho, theta, threshold, minLineLength, maxLineGap);
    return lines;
  }

  // -----------------------------
  // Erosion and dilation
  // -----------------------------

  Mat* getStructuringElement(int type, int size_x, int size_y, int center_x, int center_y)
  {
    return new Mat(getStructuringElement(type,
     Size(size_x, size_y),
     Point(center_x, center_y) ));
  }
  void dilate(Mat*  src, Mat* structuringElement)
  {
    dilate(*src, *src, *structuringElement);
  }

  void erode(Mat*  src, Mat* structuringElement)
  {
    erode(*src, *src, *structuringElement);
  }


  // -----------------------------
  // Matcher
  // -----------------------------
  DescriptorMatcher* DescriptorMatcher_create(const char* feature_type)
  {
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(feature_type);
    matcher.addref(); // make sure the Ptr stays around TODO: check memleak
    return matcher;
  }

  int DescriptorMatcher_match(DescriptorMatcher* matcher,
    const Mat*  descriptors_src,
    const Mat*  descriptors_dest,
    DMatch*   matchesC,
    int npts)
  {
    vector<DMatch> matches;

    matcher->match(*descriptors_src, *descriptors_dest, matches);

    int nmatches = matches.size();

    memcpy(matchesC, matches.data(),nmatches * sizeof(DMatch));

    return nmatches;
  }

  void DescriptorMatcher_destroy(DescriptorMatcher* matcher)
  {
    delete(matcher);
  }

  int DescriptorMatcher_reduceMatches(DMatch*   matchesptr, int nmatches, DMatch*   matchesReducedC)
  {
    vector<DMatch> matches (matchesptr, matchesptr + nmatches);
    double max_dist = 0, min_dist = 100000000;
    for( int i = 0; i < nmatches; i++ )
    {
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

    //select only good matches
    vector< DMatch > good_matches;

    for( int i = 0; i < nmatches; i++ )
    {
      if( matches[i].distance < 3*min_dist )
      {
        good_matches.push_back( matches[i]);
      }
    }

    memcpy(matchesReducedC,good_matches.data(),good_matches.size() * sizeof(DMatch));

    return good_matches.size();

  }

  // -----------------------------
  // ImageProcessing
  // -----------------------------
  Mat* getHomography(const KeyPoint* keyptr_src, int npts_src,
   const KeyPoint* keyptr_dest, int npts_dest,
   const DMatch* matchptr, int npts_match)
  {
    vector<KeyPoint> keypoints_src (keyptr_src, keyptr_src + npts_src);
    vector<KeyPoint> keypoints_dest (keyptr_dest, keyptr_dest + npts_dest);

    vector<DMatch> matches (matchptr, matchptr + npts_match);

    //-- Localize the object
    vector<Point2d> src;
    vector<Point2d> dest;

    for( int i = 0; i < npts_match; i++ )
    {
        //-- Get the keypoints from the good matches
      src.push_back( keypoints_src[ matches[i].queryIdx ].pt );
      dest.push_back( keypoints_dest[ matches[i].trainIdx ].pt );
    }
    Mat H = findHomography( src, dest, CV_RANSAC );
    H.convertTo(H,CV_32FC1,1,0);

    return new Mat(H);
  }

  Mat* warpImage(const Mat* src, const Mat* transform, int size_x, int size_y)
  {
    Mat* warpedSrc= new Mat(0,0,src->type());
    warpPerspective(*src, *warpedSrc, *transform, Size(size_x, size_y));
    return warpedSrc;
  }

  /*
  get the pairwise distance between all pairs of points in A and B
  if A is mxp and B is nxp then the result will be mxn 
  */
    void getPairwiseDistances(const Mat* A, const Mat* B, Mat* result)
  {
     Mat AdA;
     reduce((*A).mul(*A), AdA, 1, CV_REDUCE_SUM, -1); //nx1

     Mat A_new = AdA * Mat::ones(1,B->rows, A->type()); //mxn
     Mat BdB;
     reduce((*B).mul(*B), BdB, 1, CV_REDUCE_SUM, -1); //mx1

     Mat B_new = Mat::ones(A->rows,1, A->type()) * BdB.t(); //mxn

     Mat dis_square = A_new + B_new - 2*(*A)*((*B).t()); //mxn
     sqrt(dis_square, *result);
     
   }

  bool isSameTransformation(Mat* H1, Mat *H2, int min_x, int max_x, int min_y, int max_y, double threshold) {
    Mat corners = (Mat_<double>(3,4) << min_x, min_x, max_x, max_x, min_y, max_y, min_y, max_y, 1, 1, 1, 1);

    Mat transformedCorners1 = (*H1)*corners;
    Mat transformedCorners2 = (*H2)*corners;
    Mat C = (transformedCorners2-transformedCorners1);
    Mat mult = C.mul(C);
    Mat dis;
    sqrt(mult.rowRange(0,1)+mult.rowRange(1,2), dis);
    int totalCloseCorners = sum(dis<=threshold)[0]/255;
    return totalCloseCorners==4;
  }

  void getInverseMatrixFromSource(Point2d* src_pt1, Point2d* src_pt2, Mat* A_inv) {
    Mat A = (Mat_<double>(4, 4) <<src_pt1->x, -src_pt1->y, 1, 0,src_pt1->y, src_pt1->x, 0, 1,src_pt2->x,-src_pt2->y, 1,0, src_pt2->y, src_pt2->x, 0, 1);
    *A_inv = A.inv();
  }

  void getMatrixFromDestination(const Point2d* p1, const Point2d* p2, Mat* result) {
    *result = (Mat_<double>(4, 1) << p1->x, p1->y, p2->x, p2->y);
    
  }

  void getHomographyFromAb(const Mat* A_inv, const Mat* b, Mat* homography)
  {
    Mat result = (*A_inv)*(*b);
    double b1 = result.at<double>(0,0);
    double b2 = result.at<double>(1,0);
    double b3 = result.at<double>(2,0);
    double b4 = result.at<double>(3,0);
    
    *homography= (Mat_<double>(3,3) << b1, -b2, b3, b2, b1, b4, 0, 0, 1);
  }

 Mat* findBestTransformation(const Mat* goodLocationsX_src, const Mat* goodLocationsY_src, const Mat* scores_src,  const Mat* pairwise_dis_src,
  const Mat* goodLocationsX_dest, const Mat* goodLocationsY_dest, const Mat* scores_dest, const Mat* pairwise_dis_dest, 
  double corr_thresh, int minInliers, int numInliersMax, double cornerComparisonThreshold, double minx, double maxx, double miny, double maxy)
 {
  Mat* bestHMatrix = new Mat(Mat::zeros(numInliersMax,10,CV_64F));
  Mat A_inv, b, H, temp, concatWithOnes, transformed_src, reshapedTransformation, reshaped, subSelected, compareResult;
  Point2d src_pt1, src_pt2, dest_pt1, dest_pt2;
  Point minLoc; Point maxLoc;
  int numTransfSoFar = 0;
  for (int i_src = 0; i_src < goodLocationsX_src->rows; i_src++)
  {
    int pt1_src = (*goodLocationsX_src).at<double>(i_src,0)-1;
    int pt2_src = (*goodLocationsY_src).at<double>(i_src,0)-1;
    src_pt1= Point2d((*scores_src).at<double>(pt1_src,0), (*scores_src).at<double>(pt1_src,1));
    src_pt2= Point2d((*scores_src).at<double>(pt2_src,0), (*scores_src).at<double>(pt2_src,1));

    double d_src = (*pairwise_dis_src).at<double>(pt1_src, pt2_src);
      //calculate transformation matrix and it's inverse
    getInverseMatrixFromSource(&src_pt1, &src_pt2, &A_inv);

    for (int i_dest = 0; i_dest < goodLocationsX_dest->rows; i_dest++) 
    {
      int pt1_dest = (*goodLocationsX_dest).at<double>(i_dest,0)-1;
      int pt2_dest = (*goodLocationsY_dest).at<double>(i_dest,0)-1;
      dest_pt1= Point2d((*scores_dest).at<double>(pt1_dest,0), (*scores_dest).at<double>(pt1_dest,1));
      dest_pt2= Point2d((*scores_dest).at<double>(pt2_dest,0), (*scores_dest).at<double>(pt2_dest,1));

      double d_dest = (*pairwise_dis_dest).at<double>(pt1_dest, pt2_dest);
      
      if (abs(d_dest-d_src)<corr_thresh) 
      {
        getMatrixFromDestination(&dest_pt1, &dest_pt2, &b);
        getHomographyFromAb(&A_inv, &b, &H);
        //ignore if scaled!
        double scale = H.at<double>(0,0)* H.at<double>(0,0)+H.at<double>(0,1)*H.at<double>(0,1);
        if(scale>.98 && scale<=1.00001) 
        {
          vconcat((*scores_src).t(),Mat::ones(1,scores_src->rows, scores_src->type()), concatWithOnes);
          transformed_src = H*concatWithOnes;
          transformed_src = (transformed_src.rowRange(0,2)).t();
            //When the comparison result is true, the corresponding element of output array is set to 255.
          getPairwiseDistances(&transformed_src, scores_dest, &temp);
          compareResult = temp <= corr_thresh;
          long num_inliers = sum(compareResult)[0]/255;

          /*
            bool onetoone = (temp.at<double>(pt1_src, pt1_dest) < corr_thresh/2) && (temp.at<double>(pt2_src, pt2_dest) < corr_thresh/2);
            double num_inliers = 0;
            for(int i = 0; i < temp.rows; i++) {
                for(int j=0; j < temp.cols; j++) {
                    if (temp.at<double>(i,j) < corr_thresh) {
                        double d_dest1 = (*pairwise_dis_dest).at<double>(pt1_dest, j);
                        double d_dest2 = (*pairwise_dis_dest).at<double>(pt2_dest, j);
                        double d_src1 = (*pairwise_dis_src).at<double>(pt1_src, i);
                        double d_src2 = (*pairwise_dis_src).at<double>(pt2_src, i);
                        
                        if (onetoone && (abs(d_dest1-d_src1)<corr_thresh) && (abs(d_dest2-d_src2)<corr_thresh)) {
                            num_inliers++;
                        } else if ((~onetoone) && (abs(d_dest1-d_src2)<corr_thresh) && (abs(d_dest2-d_src1)<corr_thresh)) {
                            num_inliers++;
                        }
                    }
                }
            }
    */
          double minVal; double maxVal; 
          minMaxLoc( bestHMatrix->colRange(0,1), &minVal, &maxVal, &minLoc, &maxLoc);
         
          if(num_inliers > minVal && num_inliers >= minInliers) {
            bool shouldUse = true;
            reshaped = H.reshape(1,1);

            for (int tn = 0; tn < numTransfSoFar; tn++) 
            {
              reshapedTransformation = (*bestHMatrix)(Range(tn,tn+1), Range(1,10)).reshape(1,3);
              if (isSameTransformation(&reshapedTransformation, &H, minx, maxx, miny, maxy, cornerComparisonThreshold))
              {
                if(num_inliers > bestHMatrix->at<double>(tn,0)) 
                {
                  (*bestHMatrix)(Range(tn,tn+1), Range(1,10)) = reshaped;
                  bestHMatrix->at<double>(tn,0) = num_inliers;

                }
                shouldUse = false;
                break;
              }
            }

            if(shouldUse) {
              subSelected = (*bestHMatrix)(Range(minLoc.y,minLoc.y+1), Range(1,10));

               reshaped.copyTo(subSelected);
               bestHMatrix->at<double>(minLoc.y,0) = num_inliers;
               numTransfSoFar++;
               numTransfSoFar = std::min(numTransfSoFar,bestHMatrix->rows);

           }
         }
         
       }

      }

    }
 }

 return new Mat(bestHMatrix->rowRange(0,numTransfSoFar));
}

void flann_knn(Mat* m_object, Mat* m_destinations, int knn, Mat* m_indices, Mat* m_dists) {
    // find nearest neighbors using FLANN
    //cv::Mat m_indices(m_object->rows, 1, CV_32S);
    //cv::Mat m_dists(m_object->rows, 1, CV_32F);
 
    Mat dest_32f; (*m_destinations).convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; (*m_object).convertTo(obj_32f,CV_32FC2);
 
    assert(dest_32f.type() == CV_32F);
 
    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.knnSearch(obj_32f, *m_indices, *m_dists, knn, cv::flann::SearchParams(64) ); 
 
  }

void flann_radius(Mat* m_object, Mat* m_destinations, double radius, int maxresults, Mat* m_indices, Mat* m_dists) {
    // find nearest neighbors using FLANN
    //cv::Mat m_indices(m_object->rows, 1, CV_32S);
    //cv::Mat m_dists(m_object->rows, 1, CV_32F);
 
    Mat dest_32f; (*m_destinations).convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; (*m_object).convertTo(obj_32f,CV_32FC2);
 
    assert(dest_32f.type() == CV_32F);
 
    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.radiusSearch(obj_32f, *m_indices, *m_dists, radius, maxresults, cv::flann::SearchParams(64) ); 
 
}


void get_orientation(Mat* src, int ksize, Mat* mag, Mat* orientaiton)
{

    Mat Sx;
    Sobel(*src, Sx, src->type(), 1, 0, ksize);

    Mat Sy;
    Sobel(*src, Sy, src->type(), 0, 1, ksize);

    magnitude(Sx, Sy, *mag);
    phase(Sx, Sy, *orientaiton, true);

}

Mat* phaseCorrelate(Mat *src, Mat *dst) 
{
  double score;
  Point2d result= phaseCorrelateRes(*src, *dst, noArray(), &score);
  Mat* ret = new Mat(3,1, src->type());
  ret->at<double>(0,0) = result.x;
  ret->at<double>(1,0) = result.y;
  ret->at<double>(2,0) = score;
  
  return ret;
}

} // extern "C"
