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
using namespace cv;

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
    float x;
    float y;
    bool operator<(const xy temp) const
    {
        if (x == temp.x)
            return y < temp.y;
        return x < temp.x;
    }
};
static void draw_delaunay(Mat &img, Subdiv2D &subdiv, Scalar delaunay_color)
{

    vector<Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);
    vector<Point> pt(3);
    Size size = img.size();
    Rect rect(0, 0, size.width, size.height);

    for (size_t i = 0; i < triangleList.size(); i++)
    {
        Vec6f t = triangleList[i];
        pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = Point(cvRound(t[4]), cvRound(t[5]));

        // Draw rectangles completely inside the image.
        if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
            line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
            line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
        }
    }
}
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

    pcl::PointCloud<PointXYZI> cloud_with_having, cloud_with_empty;

    for (int i = 0; i < heatmap.rows(); i++)
        for (int j = 0; j < heatmap.cols(); j++)
        {
            if (heatmap(i, j) != 0)
            {
                PointXYZI point;
                point.x = i;
                point.y = j;
                point.intensity = heatmap(i, j);
                cloud_with_having.push_back(point);
            }
            else
            {
                PointXYZI point;
                point.x = i;
                point.y = j;
                cloud_with_empty.push_back(point);
            }
        }

    pcl::VoxelGrid<PointXYZI> voxel;
    voxel.setLeafSize(5, 5, 5);
    voxel.setInputCloud(cloud_with_having.makeShared());
    voxel.filter(cloud_with_having);
    voxel.setInputCloud(cloud_with_empty.makeShared());
    voxel.filter(cloud_with_empty);

    vector<Point2f> points;

    Rect rect(0, 0, 200, 200);
    Subdiv2D subdiv(rect);
    map<xy, float> PointToHeight;

    for (auto point : cloud_with_having)
    {
        points.push_back(Point2f(point.x, point.y));
        subdiv.insert(Point2f(point.x, point.y));
        PointToHeight[xy{point.x, point.y}] = point.intensity; // xy 和 z 一一对应
    }

    vector<Point2f> HullResult;
    cv::convexHull(points, HullResult, false, true);

    Mat img = Mat(200, 200, CV_8UC3, Scalar(255, 255, 255));

    for (int i = 1; i < HullResult.size(); i++)
    {
        line(img, HullResult[i - 1], HullResult[i], Scalar(0, 0, 0), 1, CV_AA, 0);
    }
    line(img, HullResult[HullResult.size() - 1], HullResult[0], Scalar(0, 0, 0), 1, CV_AA, 0);

    draw_delaunay(img, subdiv, Scalar(0, 0, 0));

    vector<Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);

    map<xy, vector<xy>> StoreXYMap;

    for (auto iter : triangleList)
    {
        StoreXYMap[xy{iter[0], iter[1]}].push_back(xy{iter[2], iter[3]});
        StoreXYMap[xy{iter[0], iter[1]}].push_back(xy{iter[4], iter[5]});
        StoreXYMap[xy{iter[2], iter[3]}].push_back(xy{iter[0], iter[1]});
        StoreXYMap[xy{iter[2], iter[3]}].push_back(xy{iter[4], iter[5]});
        StoreXYMap[xy{iter[4], iter[5]}].push_back(xy{iter[0], iter[1]});
        StoreXYMap[xy{iter[4], iter[5]}].push_back(xy{iter[2], iter[3]});
    }

    pcl::KdTreeFLANN<PointXYZI> kdtreelaserCloudOut;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreelaserCloudOut.setInputCloud(cloud_with_having.makeShared());
    for (auto &point : cloud_with_having)
        point.z = point.intensity;
    for (auto &point : cloud_with_empty)
    {
        kdtreelaserCloudOut.nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);
        PointXYZI getpoint = cloud_with_having[pointSearchInd[0]];
        auto getVec = StoreXYMap[xy{getpoint.x, getpoint.y}];
        if (getVec.empty())
        {
            cout << "w" << endl;
            continue;
        }
        //每次都是存放两个

        int index = 0;
        for (int i = 0; i < getVec.size(); i++)
        {
            index++;
            if (index == 2)
            {
                index = 0;

                Eigen::Vector2d ap(point.x - getpoint.x, point.y - getpoint.y);

                // ap = x * ab + y * ac;
                Eigen::Matrix2d A;
                A << getVec[i - 1].x - getpoint.x, getVec[i].x - getpoint.x, getVec[i - 1].y - getpoint.y, getVec[i].y - getpoint.y;

                // getpoint = A    point = P  getVec[i-1] = B  getVec[i] = C
                auto x = A.colPivHouseholderQr().solve(ap); //求解Ax=b
                if (x[0] + x[1] > 0 && x[0] + x[1] < 1)
                {
                    float _ap = 1 / sqrt(pow(getpoint.x - point.x, 2) + pow(getpoint.y - point.y, 2));
                    float _bp = 1 / sqrt(pow(getVec[i - 1].x - point.x, 2) + pow(getVec[i - 1].y - point.y, 2));
                    float _cp = 1 / sqrt(pow(getVec[i].x - point.x, 2) + pow(getVec[i].y - point.y, 2));

                    point.z = (PointToHeight[getVec[i - 1]] * _bp + PointToHeight[getVec[i]] * _cp + PointToHeight[xy{getpoint.x, getpoint.y}] * _ap) / (_ap + _bp + _cp);
                    cloud_with_having.push_back(point);
                    break;
                }
            }
        }
    }

    pcl::MovingLeastSquares<PointXYZI, PointXYZI> mls;

    mls.setNumberOfThreads(12);
    mls.setPolynomialOrder(2); //一般2-5
    mls.setSearchRadius(12);   //越大的话平滑力度越大
    mls.setComputeNormals(false);
    pcl::search::KdTree<PointXYZI>::Ptr search(new pcl::search::KdTree<PointXYZI>);
    pclCloudXYZI resultForMLS;
    search->setInputCloud(cloud_with_having.makeShared());
    mls.setSearchMethod(search);
    mls.setInputCloud(cloud_with_having.makeShared());
    mls.process(resultForMLS);

    //再次投影 然后近邻搜寻
    heatmap.setZero();
    for (auto point : resultForMLS)
    {
        if (point.x < 0 || point.y < 0 || point.x >= 200 || point.y >= 200)
            continue;
        heatmap((int)point.x, (int)point.y) = point.z;
    }

    kdtreelaserCloudOut.setInputCloud(resultForMLS.makeShared());

    for (int i = 0; i < heatmap.rows(); i++)
        for (int j = 0; j < heatmap.cols(); j++)
        {
            if (heatmap(i, j) == 0)
            {
                PointXYZI point;
                point.x = i;
                point.y = j;
                kdtreelaserCloudOut.nearestKSearch(point, 20, pointSearchInd, pointSearchSqDis);
                float dis = 0, avg = 0;
                for (int index = 0; index < pointSearchInd.size(); index++)
                {
                    dis += resultForMLS[pointSearchInd[index]].z / pointSearchSqDis[index];
                    avg += 1 / pointSearchSqDis[index];
                }
                point.z = dis / avg;
                resultForMLS.push_back(point);
            }
        }

    //判断点是否在内部  也就是找轮廓   方法就是上面的方法

    pclCloudXYZI finalResultCloud;
    for (auto &point : resultForMLS)
    {
        if (pointPolygonTest(HullResult, Point2f(point.x, point.y), false) == -1)
            continue;

        point.x = point.x * xx_res + minPt.x;
        point.y = point.y * yy_res + minPt.y;

        finalResultCloud.push_back(point);
    }
    cout << finalResultCloud.size() << " " << resultForMLS.size() << endl;
    // imshow("ww", img);
    // // imwrite("draw_delaunay.jpg", img);
    // waitKey(0);

    pcl::io::savePCDFile("inter.pcd",finalResultCloud);


    ros::init(argc, argv, "interpolation");
    ros::NodeHandle n;
    ros::Publisher pubresult1 = n.advertise<sensor_msgs::PointCloud2>("result1", 10, true);
    ros::Publisher pubresult2 = n.advertise<sensor_msgs::PointCloud2>("result2", 10, true);

    ros::Rate rate(10);
    while (ros::ok())
    {
        Publish(pubresult1, cloud_with_having);
        Publish(pubresult2, finalResultCloud);
        rate.sleep();
    }
}