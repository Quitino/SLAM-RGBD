#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <iomanip>

using namespace std;
using namespace cv;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;//Eigen需要开辟内训
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

  // BA by gauss-newton
void bundleAdjustmentGaussNewton(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose
);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);


int main(int argc, char **argv) 
{
  //读取图像
  Mat img_1 = imread("1.png", CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread("2.png", CV_LOAD_IMAGE_COLOR);
  assert(img_1.data && img_2.data && "Can not load images!");

  //特征匹配
  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << "一共找到了" << matches.size() << "组匹配点" << endl;
  
  //内参矩阵
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  
  //建立3D点
  Mat d1 = imread("1_depth.png", CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
  vector<Point3f> pts_3d;
  vector<Point2f> pts_2d;
  
  // vector<Point2f> pt;
  // vector<int> idx;
  // KeyPoint::convert(keypoints_1,pt,idx);
  // cout<<"keypoints_1:"<<keypoints_1<<endl;

  for (DMatch m:matches) {
    // Point2f pt=keypoints_1[m.queryIdx];//一个点一个点的提取
    Point2f pt=keypoints_1[m.queryIdx].pt;//一个点一个点的提取

    // Point2f pt;
    // KeyPoint::convert(keypoints_1[m.queryIdx],pt,1);
    ushort d = d1.ptr<unsigned short>(int(pt.y))[int(pt.x)];//特征点的深度[第y行，第x列]
    if (d == 0)   // bad depth
      continue;
    // float dd = d / 5000.0;
    float dd = d / 1000.0;//尺度转换
    Point2d p1 = pixel2cam(pt, K);//像素坐标转相机归一化坐标
    
    pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));//归一化坐标转相机坐标
    pts_2d.push_back(keypoints_2[m.trainIdx].pt);
  }

  cout << "3d-2d pairs: " << pts_3d.size() << endl;

  Mat r, t;
  //求解PnP
  solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); //调用opencv库函数
  cout<<"solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false):"<<endl<<"r= "<<endl<<r<<endl<<"t= "<<t<<endl;
  Mat R;
  cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
  cout<<"cv::Rodrigues(r, R):"<<endl;
  cout << "R=" << endl << R << endl;
  cout << "t=" << endl << t << endl;


  VecVector3d pts_3d_eigen;
  VecVector2d pts_2d_eigen;
  for (size_t i = 0; i < pts_3d.size(); ++i) {
    pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
    pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
  }
  
  cout << "使用高斯牛顿法进行bundle adjustment" << endl;
  Sophus::SE3d pose_gn;
  bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
  
  return 0;
}


void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);
  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);
  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> match;
  // BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);
  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;
  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);
  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}


void bundleAdjustmentGaussNewton(
		    const VecVector3d &points_3d,
		    const VecVector2d &points_2d,
		    const Mat &K,
		    Sophus::SE3d &pose)
{
  
  const int iterations = 10;
  double cost = 0, lastCost = 0;

  //相机内参数
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < iterations; iter++) 
  {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Vector6d b = Vector6d::Zero();
    cost = 0;
    //所有特征点的代价计算
    for (int i = 0; i < points_3d.size(); i++) 
    {
      //计算P'
      Eigen::Vector3d pc = pose * points_3d[i];
      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      //计算重投影的像素坐标
      Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
      //单个点的重投影误差
      Eigen::Vector2d e = points_2d[i] - proj;
      cost += e.squaredNorm();//+=平方和
      
      //计算雅可比矩阵
      Eigen::Matrix<double, 2, 6> J;
      J << 
      -fx * inv_z,			0,					fx * pc[0] * inv_z2,
      fx * pc[0] * pc[1] * inv_z2,	-fx - fx * pc[0] * pc[0] * inv_z2,	fx * pc[1] * inv_z,
      0,				-fy * inv_z,				fy * pc[1] * inv_z2,
      fy + fy * pc[1] * pc[1] * inv_z2,-fy * pc[0] * pc[1] * inv_z2,-fy * pc[0] * inv_z;

      //高斯牛顿方程为：H*dx=b，H=J^2，b=J*e
      H += J.transpose() * J;
      b += -J.transpose() * e;
    }

    //解方程：H*dx=b，其中dx是李代数的增量
    Vector6d dx;
    dx = H.ldlt().solve(b);

    if (isnan(dx[0])) {
        cout << "result is nan!" << endl;
        break;
      }

    //超过极小点时，结束迭代
    if (iter > 0 && cost >= lastCost) {
        cout << "cost: " << cost << ", last cost: " << lastCost << endl;
        break;
    }

    //更新姿态
    pose = Sophus::SE3d::exp(dx) * pose;
    lastCost = cost;

    cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
    //迭代已收敛
    if (dx.norm() < 1e-6) {//norm()表示绝对值
      break;
    }
  }

  cout << "pose by g-n: \n" << pose.matrix() << endl;
}
