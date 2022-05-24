#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid_label.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointXYZINormal PointXYZINormal;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> pclCloudXYZI;
typedef pcl::PointCloud<pcl::PointXYZ> pclCloudXYZ;
using namespace std;
template <typename T>
void Publish(ros::Publisher publish, T laserCloudIn)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(laserCloudIn, msg);
    msg.header.frame_id = "map";
    publish.publish(msg);
};
void coverToCloud(Eigen::MatrixXf heat, pclCloudXYZI &result, float xx_res, float yy_res, float minx, float miny)
{
    for (int i = 0; i < heat.rows(); i++)
    {
        for (int j = 0; j < heat.cols(); j++)
        {
            PointXYZI point;
            point.x = i * xx_res + minx;
            point.y = j * yy_res + miny;
            point.z = heat(i, j);
            result.push_back(point);
        }
    }
}
struct xy
{
    unsigned int x;
    unsigned int y;
};
int X_SCAN_2D = 200, Y_SCAN_2D = 200;
int main(int argc, char **argv)
{
    pclCloudXYZI laserCloudIn, laserCloudForTree, result1, result2;
    pcl::io::loadPCDFile("pcl_onlymeidui.pcd", laserCloudIn);

    PointXYZI minPt, maxPt;
    pcl::getMinMax3D(laserCloudIn, minPt, maxPt);

    float xx_res = (maxPt.x - minPt.x) / X_SCAN_2D;
    float yy_res = (maxPt.y - minPt.y) / Y_SCAN_2D;

    Eigen::MatrixXf heatmap;
    heatmap.resize(X_SCAN_2D, Y_SCAN_2D);
    heatmap.setZero();

    // TODO 首先完成 边界确定 然后再去填充
    //* 这个完成了投影功能,并且得到 KdCloud 所有击中点的二维坐标  kdCloud保存所有击中点(二维)  fullCloud 顺便加入以row col为坐标的点云
    std::vector<int> CopyCloudIndex;
    CopyCloudIndex.resize(laserCloudIn.size());
    std::fill(CopyCloudIndex.begin(), CopyCloudIndex.end(), -1);

    for (int i = 0; i < laserCloudIn.size(); i++)
    {
        PointXYZI point = laserCloudIn.points.at(i);

        //基于雷达地图坐标系
        int local_x = round((point.x - minPt.x) / xx_res);
        int local_y = round((point.y - minPt.y) / yy_res);

        if (local_y >= (Y_SCAN_2D) || local_x >= (X_SCAN_2D) || local_x < 0 || local_y < 0)
            continue;

        if (heatmap(local_x, local_y) != 0) //覆盖
            heatmap(local_x, local_y) = ((point.z - minPt.z) > heatmap(local_x, local_y)) ? heatmap(local_x, local_y) : (point.z - minPt.z);
        else //更新
            heatmap(local_x, local_y) = point.z - minPt.z;
    }

    // std::ofstream out;
    // out.open("data.txt");
    // for (int i = 0; i < heatmap.rows(); i++)
    //     for (int j = 0; j < heatmap.cols(); j++)
    //     {
    //         if (heatmap(i, j) != 0)
    //         {
    //             out << i << " " << j << " " << heatmap(i, j) << std::endl;
    //         }
    //     }
    // out.close();

    coverToCloud(heatmap, result1, xx_res, yy_res, minPt.x, minPt.y);
    //

    pcl::PointCloud<PointXYZINormal> cloud_with_normals, cloud_with_empty;

    for (int i = 0; i < heatmap.rows(); i++)
        for (int j = 0; j < heatmap.cols(); j++)
        {
            if (heatmap(i, j) != 0)
            {
                PointXYZI point;
                point.x = i;
                point.y = j;
                point.intensity = heatmap(i, j);
                laserCloudForTree.push_back(point);
                PointXYZINormal pointNormal;
                pointNormal.x = i;
                pointNormal.y = j;
                pointNormal.intensity = heatmap(i, j);
                pointNormal.normal_z = 1;
                pointNormal.normal_x = 0;
                pointNormal.normal_y = 0;
                cloud_with_normals.push_back(pointNormal);
            }
            else
            {
                PointXYZINormal pointNormal;
                pointNormal.x = i;
                pointNormal.y = j;
                cloud_with_empty.push_back(pointNormal);
            }
        }
    if (0)
    {
        pcl::KdTreeFLANN<PointXYZI>::Ptr kdtreelaserCloudOut;
        kdtreelaserCloudOut.reset(new pcl::KdTreeFLANN<PointXYZI>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreelaserCloudOut->setInputCloud(laserCloudForTree.makeShared());
        for (int i = 0; i < heatmap.rows(); i++)
            for (int j = 0; j < heatmap.cols(); j++)
            {
                if (heatmap(i, j) == 0)
                {
                    PointXYZI point;
                    point.x = i;
                    point.y = j;
                    kdtreelaserCloudOut->nearestKSearch(point, 50, pointSearchInd, pointSearchSqDis);

                    float heightSum = 0, disSum = 0;
                    float minAngle = 999, maxAngle = -999;
                    for (auto index : pointSearchInd)
                    {
                        PointXYZI pointCur = laserCloudForTree.points[index];

                        float dis = sqrt(pointCur.x * pointCur.x + pointCur.y + pointCur.y);
                        heightSum += pointCur.intensity / dis;
                        disSum += 1 / dis;

                        float angle = atan2(pointCur.y - point.y, pointCur.x - point.x);
                        minAngle = (minAngle < angle) ? minAngle : angle;
                        maxAngle = (maxAngle < angle) ? angle : maxAngle;
                    }
                    // if(maxAngle - minAngle)
                    heightSum /= disSum;

                    heatmap(i, j) = heightSum;
                }
            }
        coverToCloud(heatmap, result2, xx_res, yy_res, minPt.x, minPt.y);
    }
    cout << "1" << endl;

    pcl::VoxelGrid<PointXYZINormal> voxel;
    voxel.setLeafSize(5, 5, 5);
    voxel.setInputCloud(cloud_with_normals.makeShared());
    voxel.filter(cloud_with_normals);
    voxel.setInputCloud(cloud_with_empty.makeShared());
    voxel.filter(cloud_with_empty);

    // Initialize objects
    pcl::search::KdTree<PointXYZINormal>::Ptr tree2(new pcl::search::KdTree<PointXYZINormal>);

    tree2->setInputCloud(cloud_with_normals.makeShared());
    pcl::GreedyProjectionTriangulation<PointXYZINormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(100);

    // Set typical values for the parameters
    gp3.setMu(100);
    gp3.setMaximumNearestNeighbors(100000);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals.makeShared());
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // pcl::io::savePLYFile("tri.ply", triangles);
    // pcl::io::savePCDFile("cloud_with_empty.pcd", cloud_with_empty);
    // pcl::io::savePCDFile("cloud_with_normals.pcd", cloud_with_normals);

    // cout << cloud_with_normals.size() << endl;

    // cout << triangles.polygons.size() << endl;

    map<int, vector<xy>> map;
    for (auto iter : triangles.polygons)
    {
        for (int i = 0; i < iter.vertices.size(); i++)
        {
            int plus = (i + 1 == iter.vertices.size()) ? 0 : i + 1;
            int sub = (i - 1 == -1) ? iter.vertices.size() - 1 : i - 1;
            map[iter.vertices[i]].push_back(xy{iter.vertices[sub], iter.vertices[plus]});
        }
    }

    //
    pcl::KdTreeFLANN<PointXYZINormal>::Ptr kdtreelaserCloudOut;
    kdtreelaserCloudOut.reset(new pcl::KdTreeFLANN<PointXYZINormal>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreelaserCloudOut->setInputCloud(cloud_with_normals.makeShared());

    for (auto &point : cloud_with_normals)
        point.z = point.intensity;

    int count = 0;
    for (auto point : cloud_with_empty)
    {
        kdtreelaserCloudOut->nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);

        int index = pointSearchInd[0];
        if (map[index].empty())
            continue;

        for (auto iter : map[index])
        {
            Eigen::Vector2d ab, ac, ap;
            PointXYZINormal a = cloud_with_normals[index], b = cloud_with_normals[iter.x], c = cloud_with_normals[iter.y], p = point;

            ap << p.x - a.x, p.y - a.y;

            // ap = x * ab + y * ac;
            Eigen::Matrix2d A;
            A << b.x - a.x, c.x - a.x, b.y - a.y, c.y - a.y;

            auto x = A.colPivHouseholderQr().solve(ap); //求解Ax=b
            if (x[0] + x[1] > 0 && x[0] + x[1] < 1)
            {
                point.z = (a.intensity / sqrt(pow(a.x - point.x, 2) + pow(a.y - point.y, 2)) + b.intensity / sqrt(pow(b.x - point.x, 2) + pow(b.y - point.y, 2)) + c.intensity / sqrt(pow(c.x - point.x, 2) + pow(c.y - point.y, 2))) / (1 / sqrt(pow(a.x - point.x, 2) + pow(a.y - point.y, 2)) + 1 / sqrt(pow(b.x - point.x, 2) + pow(b.y - point.y, 2)) + 1 / sqrt(pow(c.x - point.x, 2) + pow(c.y - point.y, 2)));
                cloud_with_normals.push_back(point);
                break;
            }
        }
    }

    for (auto &point : cloud_with_normals)
    {
        point.x = point.x * xx_res + minPt.x;
        point.y = point.y * yy_res + minPt.y;
    }

    cout << count << " " << cloud_with_empty.size() << endl;

    ros::init(argc, argv, "interpolation");
    ros::NodeHandle n;
    ros::Publisher pubresult1 = n.advertise<sensor_msgs::PointCloud2>("result1", 10, true);
    ros::Publisher pubresult2 = n.advertise<sensor_msgs::PointCloud2>("result2", 10, true);

    ros::Rate rate(10);
    while (ros::ok())
    {
        Publish(pubresult1, cloud_with_normals);
        Publish(pubresult2, cloud_with_empty);
        rate.sleep();
    }
}