#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <chrono>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;

int main()
{
    string source_file1="../cloud_002.pcd";
    string source_file2="../cloud_003.pcd";
    string target_file="../cloud_output.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_mid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_mid(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    if((pcl::io::loadPCDFile<pcl::PointXYZ>(source_file1,*cloud1))==-1||(pcl::io::loadPCDFile<pcl::PointXYZ>(source_file2,*cloud2))==-1)
    {
        cout<<"READ PCD ERROR"<<endl;
        exit(-1);
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud1);
    sor.setMeanK(50);
    sor.setStddevMulThresh(10.0);
    sor.filter(*cloud1_mid);
    sor.setInputCloud(cloud2);
    sor.filter(*cloud2_mid);

    pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud1_icp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud2_icp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;

    grid.setLeafSize (0.2, 0.2, 0.2);
    grid.setInputCloud (cloud1_mid);
    grid.filter (*cloud1_icp);
    grid.setInputCloud (cloud2_mid);
    grid.filter (*cloud2_icp);




    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;

    icp.setInputSource(cloud1_icp);
    icp.setInputTarget(cloud2_icp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    chrono::steady_clock::time_point start= chrono::steady_clock::now();
    icp.align(*out);
    chrono::steady_clock::time_point end= chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( end-start )*1000;
    cout<<"solve time cost = "<<time_used.count()<<" ms. "<<endl;

    Eigen::Matrix4f T = icp.getFinalTransformation();
    cout<<"translation_matrix:"<<endl<<T<<endl;

    pcl::transformPointCloud(*cloud1,*output,T);
    *output+=*cloud2;
    pcl::io::savePCDFile(target_file,*output);
    return 0;
}
