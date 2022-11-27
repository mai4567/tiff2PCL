#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/filters/statistical_outlier_removal.h>
#include <string.h>
#include <pcl/visualization/pcl_visualizer.h>


//点云转tiff无序版
cv::Mat cloud2Tiff2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int width,int height){
    cv::Mat result = cv::Mat::zeros(cv::Size(width,height) ,CV_32FC3);
    int i=0;
        for(int j = 0;j<result.rows;j++){
            for (int k=0;k<result.cols;k++){
                if (i<cloud->points.size()){
                    result.ptr<float>(j)[3*k+2] = cloud->points[i].x;
                    result.ptr<float>(j)[3*k+1] = cloud->points[i].y;
                    result.ptr<float>(j)[3*k] = cloud->points[i].z;
                    i+=1;
                }
                else{
                    return result;
                }
            }
        }
    return result;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr tiff2Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_,cv::Mat tiff_){
    cloud_->points.clear();
    if (tiff_.type() == CV_32FC3){
        for (int i = 0; i < tiff_.rows; i++){
            const float *p_src = tiff_.ptr<float>(i);
            for (int j = 0; j < tiff_.cols; j++){
                if(p_src[3*j]!=0){
                    pcl::PointXYZ point;
                    point.x = p_src[3*j+2];
                    point.y = p_src[3*j+1];
                    point.z = p_src[3*j];
                    cloud_->push_back(point);
                }
            }
        }
        std::cout<<"debug_size:"<<cloud_->points.size()<<std::endl;
        return cloud_;
    }
    else{
        return cloud_;
    }
}


//点云可视化
void showCloud(std::string windowname,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (windowname));
    viewer->setBackgroundColor (0.5, 0.5, 0.5);  //设置背景
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");  //显示点云
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");  //设置点尺寸
    viewer->addCoordinateSystem (100.0);  //设置坐标轴尺寸
//    while (!viewer->wasStopped ())
//    {
//      viewer->spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
    cout<<"Point couting in "<<windowname<<": "<<cloud->size()<<endl;
}

cv::Mat showMatInDV(cv::Mat img){
    cv::Mat src;
    img.convertTo(src,CV_8UC3);
    return src;
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    cv::Mat img = cv::imread(".\\model123.tiff",cv::IMREAD_UNCHANGED|cv::IMREAD_ANYCOLOR|cv::IMREAD_ANYDEPTH);
    std::cout<<img.rows<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = tiff2Cloud(cloud,img);
    
    std::cout<<"size:"<<cloud->points.size()<<std::endl;
    showCloud("cloud",cloud);
    return a.exec();
}

