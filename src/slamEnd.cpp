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

    cout<<"Initializing ..."<<endl;

    FRAME lastFrame = readFrame(vstrImageFilenamesRGB[0], vstrImageFilenamesD[0], pd );
    // 我们总是在比较currFrame和lastFrame
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp( lastFrame);

    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );

    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );

    /*******************************
    // 新增:有关g2o的初始化
    *******************************/
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());// 线性方程求解器
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm ( solver );
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    // The parameterization for the increments constructed is a 6d vector (x,y,z,qx,qy,qz)
    g2o::VertexSE3* v = new g2o::VertexSE3();//顶点：相机位姿+ID
    v->setId( 0 );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    //pnp保存结果
    ofstream outfile;
    outfile.setf(std::ios::fixed, std::ios::floatfield);
    outfile.open("./data/resultOfpnp.txt");
    vector<Eigen::Isometry3d> beforeOptimizeOfT;
    for (int i=1; i<vTimestamps.size()-2000; i++)
    {
        FRAME currFrame = readFrame( vstrImageFilenamesRGB[i],vstrImageFilenamesD[i],pd ); // 读取currFrame
        computeKeyPointsAndDesp( currFrame);
        // 比较currFrame 和 lastFrame
        bool isMinDisEquZero = false;
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera,isMinDisEquZero);
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

        //保存结果
        // Eigen::Quaterniond q(T.block(0,0,3,3));
        // Eigen::Vector3d t = T.block(0,3,1,3);
        beforeOptimizeOfT.push_back(T);
        outfile <<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "<<T(0,3)<<endl
                <<T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "<<T(1,3)<<endl
                <<T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "<<T(2,3)<<endl
                <<T(3,0)<<" "<<T(3,1)<<" "<<T(3,2)<<" "<<T(3,3)<<endl<<endl;
        //cloud = joinPointCloud( cloud, currFrame, T, camera );
        // if ( visualize == true )
        //     viewer.showCloud( cloud );
        // lastFrame = currFrame;
        // // sleep(5);//延迟5秒 

        // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分
        // 顶点只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( i );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);
        // 边部分
        //edge====继承自===>BaseBinaryEdge<6, Isometry3, VertexSE3, VertexSE3>
        //D：测量值的维度；E：测量值的数据类型；VertexSE3：顶点类型
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( i-1 );
        edge->vertices() [1] = globalOptimizer.vertex( i );
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        edge->setMeasurement( T );
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;

    }
    outfile.close();
    
    // pcl::io::savePCDFile( "data/result.pcd", *cloud );
    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    vector<Eigen::Isometry3d> afterOptimizeOfT;
    ofstream outfile1;
    outfile1.setf(std::ios::fixed, std::ios::floatfield);
    outfile1.open("./data/resultOfoptimize.txt");
    //打印优化后的相机姿态
    for(int i = 0;i<globalOptimizer.vertices().size() -300;i++){
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(i));
        Eigen::Isometry3d T = vertex->estimate(); //该帧优化后的位姿
        //Eigen::Isometry3d T = pose.matrix();
        cout << "current is "<<i<<" T=\n" << T.matrix() << endl;

        // Eigen::Quaterniond q(T.block(0,0,3,3));
        // Eigen::Vector3d t = T.block(0,3,1,3);
        afterOptimizeOfT.push_back(T);
        outfile1 <<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "<<T(0,3)<<endl
                <<T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "<<T(1,3)<<endl
                <<T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "<<T(2,3)<<endl
                <<T(3,0)<<" "<<T(3,1)<<" "<<T(3,2)<<" "<<T(3,3)<<endl<<endl;

    }
    outfile1.close();
    cout<<"total vertex num: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.clear();


    //比较优化前后
    ofstream outfile2;
    outfile2.setf(std::ios::fixed, std::ios::floatfield);
    outfile2.open("./data/compareOfoptimize.txt");
    for(int i = 1;i<afterOptimizeOfT.size();i++)
    {
        Eigen::Isometry3d T = beforeOptimizeOfT[i-1]*beforeOptimizeOfT[i];
        outfile2 <<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "<<T(0,3)<<endl
                <<T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "<<T(1,3)<<endl
                <<T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "<<T(2,3)<<endl
                <<T(3,0)<<" "<<T(3,1)<<" "<<T(3,2)<<" "<<T(3,3)<<endl<<endl;
    }
    outfile2.close();

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


//参考博客：  https://www.twblogs.net/a/5c542fdcbd9eee06ee218f6c?lang=zh-cn
//参考博客：  http://www.jeepxie.net/article/924885.html
//参考博客：  https://www.cnblogs.com/xueyuanaichiyu/p/7921382.html
//参考博客：  https://blog.csdn.net/robinhjwy/article/details/78084210


// /home/fb/Learn/pnp/data/result_before.g2o
// /home/fb/Learn/pnp/data/result_after.g2o

// g2o_viewer: error while loading shared libraries: libg2o_viewer.so: cannot open shared object file: No such file or directory
//参考链接： https://zhuanlan.zhihu.com/p/65574135