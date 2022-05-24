#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    //按时间设置随机种子
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 生成1000个随机点
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    float resolution = 128.0f;
    //八叉树搜索对象
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    //进行八叉树结构划分
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    std::cout<<octree.getLeafCount()<<std::endl;

    pcl::PointXYZ searchPoint;

    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // 搜索searchPoint所在体素内其他点

    std::vector<int> pointIdxVec;

    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;

        for (std::size_t i = 0; i < pointIdxVec.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxVec[i]].x
                      << " " << (*cloud)[pointIdxVec[i]].y
                      << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
    }

    //搜索K邻近点

    int K = 10;
    //邻近点索引数组
    std::vector<int> pointIdxNKNSearch;
    //邻近点与测点的平方距离数组
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x
                      << " " << (*cloud)[pointIdxNKNSearch[i]].y
                      << " " << (*cloud)[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    //测点半径内搜索

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    Eigen::Vector3f direction(1, 1, 0);
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> OctreeSearchResult;

    std::cout << "dsd" << octree.getIntersectedVoxelCenters(Eigen::Vector3f(0.0f, 0.0f, 0.0f), direction, OctreeSearchResult, 100) << std::endl;

    for (auto point : OctreeSearchResult)
    {
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    }
}
