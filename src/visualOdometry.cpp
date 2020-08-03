/*************************************************************************
    > File Name: rgbd-slam-tutorial-gx/part V/src/visualOdometry.cpp
    > Author: xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Created Time: 2015年08月01日 星期六 15时35分42秒
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
#include <unistd.h> 

#include "slamBase.h"

// 给定index，读取一帧数据
FRAME readFrame( int index, ParameterReader& pd );
FRAME readFrame(string vstrImageFilenamesRGB, string vstrImageFilenamesD, ParameterReader& pd );
// 度量运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main( int argc, char** argv )
{
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );


    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = ( pd.getData("path_to_association").c_str() );
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Read image and depthmap from file
    // cv::Mat imRGB, imD;
    // imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
    // imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
    // double tframe = vTimestamps[ni];

    // if(imRGB.empty())
    // {
    //     cerr << endl << "Failed to load image at: "
    //             << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
    //     return 1;
    // }

    // cout<<"vTimestamps:  "<<vTimestamps.size()<<endl;
    // cout<<"vstrImageFilenamesD:  "<<vstrImageFilenamesD.size()<<endl;
    //cv::waitKey(0);
    // initialize
    cout<<"Initializing ..."<<endl;
    //int currIndex = startIndex = vTimestamps.size(); // 当前索引为currIndex
    // FRAME lastFrame = readFrame( currIndex, pd ); // 上一帧数据
    FRAME lastFrame = readFrame(vstrImageFilenamesRGB[0], vstrImageFilenamesD[0], pd );
    // 我们总是在比较currFrame和lastFrame
    // string detector = pd.getData( "detector" );
    // string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    // computeKeyPointsAndDesp( lastFrame, detector, descriptor );
    computeKeyPointsAndDesp( lastFrame);

    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );

    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );
    //getchar();
    // for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
// FRAME readFrame(string vstrImageFilenamesRGB, string vstrImageFilenamesD, ParameterReader& pd );

    for (int i=1; i<vTimestamps.size()-2000; i++)
    {
        // cout<<"Reading files "<<currIndex<<endl;
        // FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        FRAME currFrame = readFrame( vstrImageFilenamesRGB[i],vstrImageFilenamesD[i],pd ); // 读取currFrame
        // computeKeyPointsAndDesp( currFrame, detector, descriptor );
        computeKeyPointsAndDesp( currFrame);
        // 比较currFrame 和 lastFrame
        bool isMinDisEquZero = false;
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera,isMinDisEquZero);
        //if ( result.inliers < min_inliers && !isMinDisEquZero) //inliers不够，放弃该帧
        if ( result.inliers < min_inliers) //inliers不够，放弃该帧
        {
            isMinDisEquZero = false;
            continue;
        }

        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
        {
            cout<<"end with i: "<<i<<endl;
            continue;
        }
        cout<<"current i: "<<i<<endl;
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;

        //cloud = joinPointCloud( cloud, currFrame, T.inverse(), camera );
        cloud = joinPointCloud( cloud, currFrame, T, camera );

        if ( visualize == true )
            viewer.showCloud( cloud );

        lastFrame = currFrame;
        // sleep(5);//延迟5秒 

    }

    pcl::io::savePCDFile( "data/result.pcd", *cloud );
    return 0;
}

FRAME readFrame( int index, ParameterReader& pd )
{
    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");

    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );
    cv::imshow("test.png",f.rgb);

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    return f;
}


FRAME readFrame(string vstrImageFilenamesRGB, string vstrImageFilenamesD, ParameterReader& pd )
{
    FRAME f;
    string path_to_sequence   =   pd.getData("path_to_sequence");
    
    // f.rgb = cv::imread(path_to_sequence+"/"+vstrImageFilenamesRGB,CV_LOAD_IMAGE_UNCHANGED);
    // f.depth  = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD,CV_LOAD_IMAGE_UNCHANGED);    
    f.rgb = cv::imread(path_to_sequence+"/"+vstrImageFilenamesRGB);
    f.depth  = cv::imread(path_to_sequence+"/"+vstrImageFilenamesD,-1);
    // double tframe = vTimestamps[ni];

    if(f.rgb.empty())
    {
        cerr << endl << "Failed to load image at: "
                << string(path_to_sequence + "/" + vstrImageFilenamesRGB) << endl;
        
    }
    // f.depth = cv::imread( filename, -1 );
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}


//参考博客： https://www.twblogs.net/a/5c542fdcbd9eee06ee218f6c?lang=zh-cn
//参考博客：  http://www.jeepxie.net/article/924885.html