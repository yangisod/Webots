#include "finalmapping.h"

template <typename T>
void Publish(ros::Publisher publish, T laserCloudIn)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(laserCloudIn, msg);
    msg.header.frame_id = "map";
    publish.publish(msg);
};
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
std::mutex mtx;
// For Hok
class interpolation
{
private:
    pcl::VoxelGrid<PointXYZI> voxel;                  //下采样
    Eigen::MatrixXf heatmap;                          //投影高度矩阵
    pclCloudXYZI cloud_with_having, cloud_with_empty; //投影高度依据点云 和 需要插值点云
    vector<Point2f> HullResult;                       //外边界轮廓结果 和 轮廓判断输入
    PointXYZI minPt, maxPt;                           //点云最大值 最小值
    int X_SCAN_2D = 200, Y_SCAN_2D = 200;
    map<xy, vector<xy>> StoreXYMap; //存储 所有 点 和其他 三角点 的对应map
    map<xy, float> PointToHeight;   // xy和z对应
    pcl::KdTreeFLANN<PointXYZI> kdtreelaserCloudOut;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pcl::MovingLeastSquares<PointXYZI, PointXYZI> mls;
    float xx_res = 0;
    float yy_res = 0;
    pclCloudXYZI laserCloudIn;
    vector<Vec6f> triangleList; //保存所有 三角化结果

    ros::Publisher pubInter_result;

    stack<pclCloudXYZI> storePclcloud;
    std::future<void> process_modbus;

public:
    interpolation() // : laserCloudIn(cloudIn)
    {
        voxel.setLeafSize(5, 5, 5);
        heatmap.resize(X_SCAN_2D, Y_SCAN_2D);

        mls.setNumberOfThreads(12);
        mls.setPolynomialOrder(2); //一般2-5
        mls.setSearchRadius(12);   //越大的话平滑力度越大
        mls.setComputeNormals(false);

        ros::NodeHandle n_;
        pubInter_result = n_.advertise<sensor_msgs::PointCloud2>("pubInter_result", 10, true);

        process_modbus = std::async(std::launch::async, &interpolation::laserCloudHandle, this);
    }
    void laserCloudHandle()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            pclCloudXYZI laserCloudIn;
            mtx.lock();
            if (storePclcloud.empty())
            {
                mtx.unlock();
                continue;
            }
            pcl::copyPointCloud(storePclcloud.top(), laserCloudIn);
            while (!storePclcloud.empty())
                storePclcloud.pop();
            mtx.unlock();

            double time1 = clock();
            cout << laserCloudIn.size() << endl;
            if (laserCloudIn.empty())
                return;
            project(laserCloudIn);
            double time2 = clock();
            cout << "project: " << (time2 - time1) / CLOCKS_PER_SEC << endl;
            preHandle();
            double time3 = clock();
            cout << "preHandle: " << (time3 - time2) / CLOCKS_PER_SEC << endl;
            TriInter();
            double time4 = clock();
            cout << "TriInter: " << (time4 - time3) / CLOCKS_PER_SEC << endl;
            pclCloudXYZI result = inflation();
            double time5 = clock();
            cout << "inflation: " << (time5 - time4) / CLOCKS_PER_SEC << endl;
            Publish(pubInter_result, result);
            resetParams();
            cout << "总结：" << (clock() - time1) / CLOCKS_PER_SEC << endl;

            rate.sleep();
        }
    }
    pclCloudXYZI filter()
    {
        double time1 = clock();
        cout << laserCloudIn.size() << endl;
        if (laserCloudIn.empty())
            return laserCloudIn;
        project(laserCloudIn);
        double time2 = clock();
        cout << "project: " << (time2 - time1) / CLOCKS_PER_SEC << endl;
        preHandle();
        double time3 = clock();
        cout << "preHandle: " << (time3 - time2) / CLOCKS_PER_SEC << endl;
        TriInter();
        double time4 = clock();
        cout << "TriInter: " << (time4 - time3) / CLOCKS_PER_SEC << endl;
        pclCloudXYZI result = inflation();
        double time5 = clock();
        cout << "inflation: " << (time5 - time4) / CLOCKS_PER_SEC << endl;
        Publish(pubInter_result, result);
        resetParams();
        cout << "总结：" << (clock() - time1) / CLOCKS_PER_SEC << endl;
        return result;
    }
    void addInputCloud(pclCloudXYZI cloudIn)
    {
        mtx.lock();
        storePclcloud.push(cloudIn);
        mtx.unlock();
    }
    void project(pclCloudXYZI &laserCloudIn)
    {
        pcl::getMinMax3D(laserCloudIn, minPt, maxPt);
        xx_res = (maxPt.x - minPt.x) / X_SCAN_2D;
        yy_res = (maxPt.y - minPt.y) / Y_SCAN_2D;
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
    }
    void preHandle()
    {
        voxel.setInputCloud(cloud_with_having.makeShared());
        voxel.filter(cloud_with_having);
        voxel.setInputCloud(cloud_with_empty.makeShared());
        voxel.filter(cloud_with_empty);

        vector<Point2f> pointsForHull;

        Rect rect(0, 0, 200, 200);
        Subdiv2D subdiv(rect);

        for (auto point : cloud_with_having)
        {
            pointsForHull.push_back(Point2f(point.x, point.y));
            subdiv.insert(Point2f(point.x, point.y));
            PointToHeight[xy{point.x, point.y}] = point.intensity; // xy 和 z 一一对应
        }
        HullResult.clear();
        cv::convexHull(pointsForHull, HullResult, false, true);

        subdiv.getTriangleList(triangleList);

        for (auto iter : triangleList)
        {
            StoreXYMap[xy{iter[0], iter[1]}].push_back(xy{iter[2], iter[3]});
            StoreXYMap[xy{iter[0], iter[1]}].push_back(xy{iter[4], iter[5]});
            StoreXYMap[xy{iter[2], iter[3]}].push_back(xy{iter[0], iter[1]});
            StoreXYMap[xy{iter[2], iter[3]}].push_back(xy{iter[4], iter[5]});
            StoreXYMap[xy{iter[4], iter[5]}].push_back(xy{iter[0], iter[1]});
            StoreXYMap[xy{iter[4], iter[5]}].push_back(xy{iter[2], iter[3]});
        }
    }
    void TriInter()
    {
        kdtreelaserCloudOut.setInputCloud(cloud_with_having.makeShared());
        for (auto &point : cloud_with_having)
            point.z = point.intensity;       //这里恢复z
        for (auto &point : cloud_with_empty) //对每一个空点进行插值
        {
            kdtreelaserCloudOut.nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);
            PointXYZI getpoint = cloud_with_having[pointSearchInd[0]]; // a点
            auto getVec = StoreXYMap[xy{getpoint.x, getpoint.y}];      //取出这个点连接所有三角的两个顶点
            if (getVec.empty())                                        //如果没有  不会出现这种情况，因为每一个顶点都有对应的三角 肯定有
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
                        point.intensity = point.z;
                        cloud_with_having.push_back(point); //这个空点插值完毕后,就存入 having 中,之前having中的点全部恢复正常z了
                        break;
                    }
                }
            }
        }
    }
    pclCloudXYZI inflation()
    {
        //由于上面 三角化 时间处理很长，所以进行了5x5x5的下采样，导致点很稀疏，但是点分布均匀，剩下的就是根据这些点开始膨胀
        //膨胀原本采用cv的方法，但是还是利用已有的 heatmap 继续插值即可

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
            int x_index = int(round(point.x)), y_index = int(round(point.y));
            if (x_index < 0 || y_index < 0 || x_index >= 200 || y_index >= 200)
                continue;
            heatmap(x_index, y_index) = point.z;
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
                    heatmap(i, j) = 1;
                }
            }
        int count = 0;
        for (int i = 0; i < heatmap.rows(); i++)
            for (int j = 0; j < heatmap.cols(); j++)
            {
                if (heatmap(i, j) == 0)
                    count++;
            }
        // cout << count << endl;

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

        return finalResultCloud;
    }
    void resetParams(bool flag = true)
    {
        heatmap.setZero();
        cloud_with_having.clear();
        cloud_with_empty.clear();

        StoreXYMap.clear();
        PointToHeight.clear();

        xx_res = 0;
        yy_res = 0;
        if (flag)
            laserCloudIn.clear();

        // HullResult.clear();
        // triangleList.clear(); //这个就不清除，可以在showimage中展示
    }

    void showImage(bool SaveOrWrite)
    {
        Mat img = Mat(200, 200, CV_8UC3, Scalar(255, 255, 255));
        for (int i = 1; i < HullResult.size(); i++)
        {
            line(img, HullResult[i - 1], HullResult[i], delaunay_color, 1, CV_AA, 0);
        }
        line(img, HullResult[HullResult.size() - 1], HullResult[0], delaunay_color, 1, CV_AA, 0);

        vector<Point> pt(3);
        Size size = img.size();
        Rect rect(0, 0, size.width, size.height);

        for (size_t i = 0; i < triangleList.size(); i++)
        {
            Vec6f t = triangleList[i];
            pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
            pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
            pt[2] = Point(cvRound(t[4]), cvRound(t[5]));

            if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
            {
                line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
                line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
                line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
            }
        }
        if (SaveOrWrite)
            imwrite("triangleList.jpg", img);
        else
        {
            imshow("triangleList", img);
            waitKey(0);
        }
    }
};
// For Hok
class segmentation
{
private:
    ros::Subscriber lidarSub;
    ros::Publisher pubSeg_result, pubSeg_result2;
    vector<int> indexSearch{-3, -2, -1, 1, 2, 3};
    pcl::KdTreeFLANN<PointXYZ> kdtree;
    vector<int> kdtreeIndice;
    vector<float> kdtreeDistance;
    pcl::VoxelGrid<PointXYZI> voxel;
    pclCloudXYZI laserCloudMap;
    interpolation &iter;

public:
    segmentation(interpolation &iter_) : iter(iter_)
    {
        // iter = iter_;
        ros::NodeHandle n_;
        voxel.setLeafSize(0.1, 0.1, 0.1);
        lidarSub = n_.subscribe<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_Register", 1, &segmentation::laserCloudHandle, this);
        pubSeg_result = n_.advertise<sensor_msgs::PointCloud2>("pubSeg_result", 1);
        pubSeg_result2 = n_.advertise<sensor_msgs::PointCloud2>("pubSeg_result2", 1);
    }
    void laserCloudHandle(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        double time1 = clock();
        pclCloudXYZ temp;
        pclCloudXYZI laserCloudIn, laserCloudMapCopy, laserCloudMeidui;
        pcl::fromROSMsg(*msg, temp);

        pcl::copyPointCloud(temp, laserCloudIn);
        laserCloudMap += laserCloudIn;
        voxel.setInputCloud(laserCloudMap.makeShared());
        voxel.filter(laserCloudMap);
        Publish(pubSeg_result2, laserCloudMap);

        pcl::copyPointCloud(laserCloudMap, laserCloudMapCopy);
        if (laserCloudMapCopy.size() <= 10)
            return;

        float dis = 0.2;        // 0.2
        float z_dis = 0.025;    // 0.05
        float vector_dis = 0.5; // 0.8
        int surround_sum = 8;   // 7

        pcl::KdTreeFLANN<PointXYZI> kdtree;
        kdtree.setInputCloud(laserCloudMapCopy.makeShared());
        vector<int> indices;
        vector<float> distance;

        queue<PointXYZI> seedQue;
        PointXYZI seed;
        seedQue.push(seed);

        while (!seedQue.empty())
        {
            auto pointfront = seedQue.front();
            seedQue.pop();

            kdtree.nearestKSearch(pointfront, 20, indices, distance);

            Eigen::Vector3d pointfrontXYZ(pointfront.x, pointfront.y, pointfront.z);
            Eigen::Vector2d pointfrontXY(pointfront.x, pointfront.y);

            int NotSeedNum = 0;

            for (auto indice : indices)
            {
                PointXYZI &point = laserCloudMapCopy[indice];

                if (point.intensity == 20 || point.intensity == 30)
                    continue;

                double len = (pointfrontXYZ - Eigen::Vector3d(point.x, point.y, point.z)).norm();
                double angle = atan2(abs(point.z - pointfront.z), (pointfrontXY - Eigen::Vector2d(point.x, point.y)).norm()) * 180 * M_1_PI;

                if (len < dis && abs(point.z - pointfront.z) < z_dis && angle < 20)
                {
                    seedQue.push(point);
                    point.intensity = 20;
                }
                else
                    NotSeedNum++; //一旦周围数目不满足 > 8 就进入再次判断  判断曲率 不够竖直 当前点重新赋值30
            }
            if (0) // NotSeedNum > surround_sum
            {
                // resultCloud2.push_back(pointfront);

                Eigen::Vector4d centroid;
                pcl::compute3DCentroid(pcl::PointCloud<PointXYZI>(laserCloudMapCopy, indices), centroid);

                //计算协方差矩阵
                double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
                for (auto indice : indices)
                {
                    xx += (laserCloudMapCopy[indice].x - centroid.x()) * (laserCloudMapCopy[indice].x - centroid.x());
                    xy += (laserCloudMapCopy[indice].x - centroid.x()) * (laserCloudMapCopy[indice].y - centroid.y());
                    xz += (laserCloudMapCopy[indice].x - centroid.x()) * (laserCloudMapCopy[indice].z - centroid.z());
                    yy += (laserCloudMapCopy[indice].y - centroid.y()) * (laserCloudMapCopy[indice].y - centroid.y());
                    yz += (laserCloudMapCopy[indice].y - centroid.y()) * (laserCloudMapCopy[indice].z - centroid.z());
                    zz += (laserCloudMapCopy[indice].z - centroid.z()) * (laserCloudMapCopy[indice].z - centroid.z());
                }

                //大小为3*3的协方差矩阵
                Eigen::Matrix3f covMat(3, 3);
                covMat(0, 0) = xx / indices.size();
                covMat(0, 1) = covMat(1, 0) = xy / indices.size();
                covMat(0, 2) = covMat(2, 0) = xz / indices.size();
                covMat(1, 1) = yy / indices.size();
                covMat(1, 2) = covMat(2, 1) = yz / indices.size();
                covMat(2, 2) = zz / indices.size();

                Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
                Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
                Eigen::Matrix3f vec = es.pseudoEigenvectors();

                //找到最小特征值t1
                double t1 = val(0, 0);
                int ii = 0;
                if (t1 > val(1, 1))
                {
                    ii = 1;
                    t1 = val(1, 1);
                }
                if (t1 > val(2, 2))
                {
                    ii = 2;
                    t1 = val(2, 2);
                }

                //最小特征值对应的特征向量v_n
                Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii)); //最小特征值就是这个面的法向量 最大的是平面的方向向量
                //特征向量单位化
                v /= v.norm();

                if (abs(v(2)) < vector_dis) //不够竖直  v2就是z轴分量
                {
                    for (auto indice : indices)
                    {
                        PointXYZI &point = laserCloudMapCopy[indice];
                        point.intensity = 30;
                    }
                }
            }
        }

        pclCloudXYZI result;
        for (auto point : laserCloudMapCopy)
            if (point.intensity == 20)
                result.push_back(point);

        Publish(pubSeg_result, result);
        // cout << (clock() - time1) / CLOCKS_PER_SEC << endl;

        iter.addInputCloud(result);
    }
};
// For Vel
pclCloudXYZ laserCloudForOctreeFirst;
// 目前想法是 有一个初始地图，然后每一帧在这个地图上匹配，但是如何创建这个初始地图呢，首先就是通过octomap来维护这个地图，每次跟这个map来匹配
// 维护什么 维护地图，假设当前为Cur帧，那么 Cur - 3 之前的帧 维护， 然后Cur跟这个地图匹配
// 如果是这种做法，其实根本不需要这样匹配， 只需要这个栅格地图 反馈到 点云，然后当前帧 对目前帧 作距离约束， 取最大距离点云去除。
// 那么反馈 怎么做，首先提取所有 占据栅格 ，得到中心点坐标，然后建立map，所有在栅格的点 全部提取出来 即可。
// 那能不能不需要距离约束，其实应该可以 （主要解决能不能 第一次击中点 不能算占据点，必须第二次击中才算）
class octoGridmap
{
private:
    ros::Subscriber laserSub, updateSub;
    ros::Publisher pubGrid_result1, pubGrid_result2, pubGrid_Octomap, pubUpdate;
    pcl::VoxelGrid<PointXYZI> voxel;
    octomap::OcTree Octree{0.1};
    ros::NodeHandle n_;
    int index = 0;

    queue<pclCloudXYZI> queUpdate;

public:
    octoGridmap()
    {
        Octree.setResolution(0.1);
        // Octree.setOccupancyThres(0.7);
        // Octree.setProbHit(0.7); // 0.6
        // Octree.setProbMiss(0.5);
        voxel.setLeafSize(0.1, 0.1, 0.1);
        laserSub = n_.subscribe<sensor_msgs::PointCloud2>("Velodyne_PointCloud_Register", 1, &octoGridmap::laserCloudHandle, this);

        pubGrid_result1 = n_.advertise<sensor_msgs::PointCloud2>("pubGrid_result1", 1, true);
        pubGrid_result2 = n_.advertise<sensor_msgs::PointCloud2>("pubGrid_result2", 1, true);
        pubGrid_Octomap = n_.advertise<octomap_msgs::Octomap>("pubGrid_Octomap", 5, true);

        pubUpdate = n_.advertise<sensor_msgs::PointCloud2>("updateGridMapLocalCircle", 1, true);
        updateSub = n_.subscribe<sensor_msgs::PointCloud2>("updateGridMapLocalCircle", 1, &octoGridmap::updateGridmap, this);

        // sensor_msgs::PointCloud2 msg;
        // pcl::toROSMsg(laserCloudForOctreeFirst, msg);
        // octomap::Pointcloud octo_cloud;
        // octomap::pointCloud2ToOctomap(msg, octo_cloud);

        // Octree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0));
        // Octree.updateInnerOccupancy();

        // octomap_msgs::Octomap octomap_msg;
        // octomap_msg.header.frame_id = "map";
        // octomap_msgs::fullMapToMsg(Octree, octomap_msg); //这一步耗时很高
        // pubGrid_Octomap.publish(octomap_msg);
    }

    void laserCloudHandle(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pclCloudXYZ temp;
        pclCloudXYZI laserCloudIn, laserCloudResult;

        pcl::fromROSMsg(*msg, temp);

        vector<int> NANIndices;
        pcl::removeNaNFromPointCloud(temp, temp, NANIndices);

        pcl::copyPointCloud(temp, laserCloudIn);

        double time1 = clock();

        for (auto &point : laserCloudIn)
        {
            octomap::point3d pointCur(point.x, point.y, point.z);
            octomap::OcTreeNode *node = Octree.search(pointCur, 0);

            if (node)
            {
                if (Octree.isNodeOccupied(node))
                {

                    point.intensity = 20;
                }
                else
                {
                    PointXYZI pointResult(20);
                    pointResult.x = point.x;
                    pointResult.y = point.y;
                    pointResult.z = point.z;
                    laserCloudResult.push_back(pointResult);
                    point.intensity = 80;
                }
            }
        }

        // cout << (clock() - time1) / CLOCKS_PER_SEC << endl;

        // sensor_msgs::PointCloud2 laserCloudMsg;
        // pcl::toROSMsg(temp, laserCloudMsg);
        // double time2 = clock();
        // octomap::Pointcloud octo_cloud;
        // octomap::pointCloud2ToOctomap(*msg, octo_cloud);

        // double time3 = clock();

        // Octree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0));
        // Octree.updateInnerOccupancy();

        // cout << (time3 - time2) / CLOCKS_PER_SEC << " " << (clock() - time3) / CLOCKS_PER_SEC << endl;

        // octomap_msgs::Octomap octomap_msg;
        // octomap_msg.header.frame_id = "map";
        // octomap_msgs::fullMapToMsg(Octree, octomap_msg); //这一步耗时很高
        // pubGrid_Octomap.publish(octomap_msg);

        Publish(pubGrid_result1, laserCloudIn);
        Publish(pubGrid_result2, laserCloudResult);

        Publish(pubUpdate, laserCloudIn);

        //这个时候有一个问题就是 当 octomap的占据格子 > 激光帧的格子，遍历搜寻octomap是不对的
        //应该是激光帧 投影到栅格，然后遍历 找格子是否被占据，如果是 即加入普通点
        //这个投影过程 应该用
    }

    void updateGridmap(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        octomap::Pointcloud octo_cloud;
        octomap::pointCloud2ToOctomap(*msg, octo_cloud);
        Octree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0));
        Octree.updateInnerOccupancy();

        octomap_msgs::Octomap octomap_msg;
        octomap_msg.header.frame_id = "map";
        octomap_msgs::fullMapToMsg(Octree, octomap_msg); //这一步耗时很高
        pubGrid_Octomap.publish(octomap_msg);
    }
};
// For Vel
/*
输入参数： 订阅 当前帧 和上一帧的odom,但是这里odom可以不用，因为我们用GPSIMU先一步矫正了，算是一个odom
输出参数： 发布 当前帧动态过滤后的点云
主要作用： 完成了 点云输入，经过 odom 矫正后(仿真环境下用GPS和IMU来矫正)，通过 getPointIndicesFromNewVoxels 获取动态点云，
         过滤后，重新投影到 camera 坐标系下，然后传入到aloam定位建图。
*/
class octreeGrid
{
private:
    int switchBuffersFlag = 0, switchBuffersIndex = 0;
    ros::Publisher pubOctree_Out, pubOctree_Map, pubOctree_Reference, pubOctree_Comparison, pubOctree_Freespace, pub_image_Map, pub_image_Cur, pubOctree_ComparisonMove, pubOctree_ComparisonStatic, pubOctree_DBSCAN;
    ros::Subscriber laserSub;

    pcl::ExtractIndices<PointXYZI> extract{true};

    pclCloudXYZI laserCloudMap;
    int indexFrame = 0;
    pcl::VoxelGrid<PointXYZI> voxel;
    pcl::CropBox<PointXYZ> roof;
    PointXYZI MapMinPt, MapMaxPt;
    int map_size_x = 80, map_size_y = 50;
    Eigen::MatrixXf HeatForMap;
    pcl::KdTreeFLANN<PointXYZI> kdtree;
    vector<int> kdtreeIndice;
    vector<float> kdtreeDistance;
    queue<pclCloudXYZI> CloudQue;

    struct ReferenceFrame
    {
        pcl::octree::OctreePointCloudChangeDetector<PointXYZI> octree{0.1};
        pcl::KdTreeFLANN<PointXYZI> kdtree;
        pcl::octree::OctreePointCloudSearch<PointXYZI> octreePre{0.1};
    };

    ReferenceFrame StoreFrame[5];

public:
    octreeGrid()
    {
        roof.setNegative(true);
        roof.setMin(Eigen::Vector4f(-10, -10, -0.5, 1));
        roof.setMax(Eigen::Vector4f(10, 10, 0.5, 1));
        voxel.setLeafSize(0.05, 0.05, 0.05);
        ros::NodeHandle n;
        pubOctree_Out = n.advertise<sensor_msgs::PointCloud2>("laserCloudOut", 1, true);
        pubOctree_Map = n.advertise<sensor_msgs::PointCloud2>("laserCloudMap", 1, true);
        pubOctree_Reference = n.advertise<sensor_msgs::PointCloud2>("laserCloudReference", 1, true);
        pubOctree_Freespace = n.advertise<sensor_msgs::PointCloud2>("laserCloudFreespace", 1, true);
        pubOctree_ComparisonMove = n.advertise<sensor_msgs::PointCloud2>("laserCloudComparisonMove", 1, true);
        pubOctree_ComparisonStatic = n.advertise<sensor_msgs::PointCloud2>("laserCloudComparisonStatic", 1, true);
        pubOctree_DBSCAN = n.advertise<sensor_msgs::PointCloud2>("laserCloudDBSCANResult", 1, true);

        pub_image_Map = n.advertise<sensor_msgs::Image>("/pub_image_Map", 1);
        pub_image_Cur = n.advertise<sensor_msgs::Image>("/pub_image_Cur", 1);

        laserSub = n.subscribe<sensor_msgs::PointCloud2>("/Velodyne_PointCloud_Register", 1, &octreeGrid::laserCloudHandle3, this);
    }
    void laserCloudHandle(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        // pclCloudXYZ temp;
        // pclCloudXYZI laserCloudIn;
        // pcl::fromROSMsg(*msg, temp);
        // pcl::copyPointCloud(temp, laserCloudIn);

        // octree.setInputCloud(laserCloudIn.makeShared());
        // octree.addPointsFromInputCloud();
        // if (switchBuffersFlag)
        // {
        //     octree.switchBuffers();
        //     switchBuffersFlag = false;
        //     pcl::copyPointCloud(laserCloudIn, laserCloudLast);
        //     return;
        // }
        // std::vector<int> newPointIdxVector;
        // // 从八叉树体素中获取点索引向量，这在之前的缓冲区中不存在
        // octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        // pclCloudXYZI laserCloudMoveResult, laserCloudReResult;

        // for (auto indice : newPointIdxVector)
        // {
        //     // laserCloudIn[indice].intensity = 20; //动态点 是 20
        //     laserCloudMoveResult.push_back(laserCloudIn[indice]);
        //     laserCloudMoveResult.back().intensity = indice;
        // }
        // newPointIdxVector.clear();

        // pcl::RadiusOutlierRemoval<PointXYZI> sor;             // 设置滤波器对象
        // sor.setInputCloud(laserCloudMoveResult.makeShared()); // 设置输入点云
        // sor.setRadiusSearch(0.1);                             // 设置在1.5的半径内找临近点
        // sor.setMinNeighborsInRadius(10);                      // 设置邻近点个数小于2的点均删除
        // sor.filter(laserCloudMoveResult);                     // 执行过滤，保存结果

        // //接下来根据 laserCloudResult2 的点 在 laserCloudIn 中聚类, 将小车聚类出来 视作动态物体

        // for (auto point : laserCloudMoveResult)
        //     if (point.intensity != 0 && point.intensity > 0 && point.intensity < laserCloudIn.size())
        //     {
        //         laserCloudIn[point.intensity].intensity = -1; //这是动态物体
        //         newPointIdxVector.push_back(point.intensity);
        //     }

        // kdtree.setInputCloud(laserCloudLast.makeShared()); //将所有 怀疑点 近邻搜索， 然后 计算
        // for (auto point : laserCloudMoveResult)
        // {
        //     kdtree.nearestKSearch(point, 20, kdtreeIndice, kdtreeDistance);
        //     std::accumulate(kdtreeDistance.begin(), kdtreeDistance.end(), 0.0);
        // }

        // for (auto point : laserCloudIn)
        //     if (point.intensity != -1)
        //         laserCloudReResult.push_back(point);

        // cout << laserCloudIn.size() << " " << laserCloudReResult.size() << endl;
        // Publish(pubOctree_Comparison, laserCloudReResult);

        // octree.switchBuffers();
        // pcl::copyPointCloud(laserCloudIn, laserCloudLast);
    }

    void laserCloudHandle2(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        // pclCloudXYZ laserCloudIn;
        // pcl::fromROSMsg(*msg, laserCloudIn);

        // vector<int> NANIndices;
        // pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, NANIndices);

        // Publish(pubOctree_Out, laserCloudIn);

        // if (switchBuffersIndex < 5)
        // {
        //     CloudQue.push(laserCloudIn);
        //     switchBuffersIndex++;
        //     return;
        // }

        // pclCloudXYZ quefront = CloudQue.front();
        // CloudQue.pop();
        // CloudQue.push(laserCloudIn);

        // pcl::KdTreeFLANN<PointXYZ> kdtree;
        // vector<int> kdtreeIndex;
        // vector<float> kdtreeDistance;
        // kdtree.setInputCloud(quefront.makeShared());
        // map<float, int, greater<float>> DistanceAndIndex;
        // for (int i = 0; i < laserCloudIn.size(); i++)
        // {
        //     PointXYZ point = laserCloudIn[i];
        //     kdtree.nearestKSearch(point, 5, kdtreeIndex, kdtreeDistance);
        //     float dis = accumulate(kdtreeDistance.begin(), kdtreeDistance.end(), 0.0) / kdtreeDistance.size();
        //     DistanceAndIndex.insert(make_pair(dis, i));
        // }
        // int index = 0.3 * DistanceAndIndex.size();
        // pclCloudXYZ laserCloudComparison, laserCloudFreespace;
        // for (auto iter : DistanceAndIndex)
        // {
        //     if (index-- <= 0 || sqrt(iter.first) < 0.05)
        //         break;
        //     laserCloudComparison.push_back(laserCloudIn[iter.second]);
        // }
        // Publish(pubOctree_Comparison, laserCloudComparison);
        // Publish(pubOctree_Reference, quefront);

        // pcl::octree::OctreePointCloudSearch<PointXYZ> octreePre{0.1};
        // voxel.setInputCloud(quefront.makeShared());
        // // voxel.filter(quefront);
        // octreePre.setInputCloud(laserCloudIn.makeShared());
        // octreePre.addPointsFromInputCloud();
        // //开启射线 模式！
        // std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> OctreeSearchResult;
        // for (auto point : quefront)
        // {
        //     Eigen::Vector3f direction(point.x, point.y, point.z);
        //     octreePre.getIntersectedVoxelCenters(Eigen::Vector3f(0.0f, 0.0f, 1.0f), direction, OctreeSearchResult, 100);
        //     if (OctreeSearchResult.empty())
        //         continue;
        //     PointXYZ pointFar = OctreeSearchResult.back();
        //     float referenceDistance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        //     float queryframeDistance = sqrt(pointFar.x * pointFar.x + pointFar.y * pointFar.y + pointFar.z * pointFar.z);
        //     if (abs(referenceDistance - queryframeDistance) < 0.1)
        //     {

        //     } //在平面上 视作静态物体
        //     else if (referenceDistance > queryframeDistance)
        //     {
        //         for (auto iter : OctreeSearchResult)
        //             laserCloudFreespace.push_back(iter);
        //     }    //参考点 击穿了 查询点  即为动态点
        //     else //这里就是 未击中 还没到，论文中是 暂时没确定
        //     {
        //     }
        // }

        // Publish(pubOctree_Freespace, laserCloudFreespace);
    }

    void returnEigen(pclCloudXYZI laserCloudIn, PointXYZI minPt, PointXYZI maxPt, Eigen::MatrixXf &heatmap)
    {

        heatmap.resize(map_size_x, map_size_y); // rows  cols
        for (int i = 0; i < heatmap.rows(); i++)
            for (int j = 0; j < heatmap.cols(); j++)
                heatmap(i, j) = 9999;

        for (auto point : laserCloudIn)
        {
            //基于雷达地图坐标系
            int local_x = int((point.x - minPt.x) * map_size_x / (maxPt.x - minPt.x));
            int local_y = int((point.y - minPt.y) * map_size_y / (maxPt.y - minPt.y));

            if (local_y >= (map_size_y) || local_x >= (map_size_x) || local_x < 0 || local_y < 0)
                continue;

            if (heatmap(local_x, local_y) != 0) //覆盖
            {
                heatmap(local_x, local_y) = (heatmap(local_x, local_y) < point.z) ? (heatmap(local_x, local_y)) : point.z;
            }
            else //更新
            {
                heatmap(local_x, local_y) = point.z;
            }
        }
    }
    void returnEigenForCur(pclCloudXYZI laserCloudIn, PointXYZI minPt, PointXYZI maxPt, Eigen::MatrixXf &heatmap, map<int, vector<int>> &StoreAllPoints)
    {
        heatmap.resize(map_size_x, map_size_y); // rows  cols
        for (int i = 0; i < heatmap.rows(); i++)
            for (int j = 0; j < heatmap.cols(); j++)
                heatmap(i, j) = 9999;

        for (int i = 0; i < laserCloudIn.size(); i++)
        {
            PointXYZI point = laserCloudIn[i];
            //基于雷达地图坐标系
            int local_x = int((point.x - minPt.x) * map_size_x / (maxPt.x - minPt.x));
            int local_y = int((point.y - minPt.y) * map_size_y / (maxPt.y - minPt.y));

            if (local_y >= (map_size_y) || local_x >= (map_size_x) || local_x < 0 || local_y < 0)
                continue;

            if (point.z < heatmap(local_x, local_y))
            {
                heatmap(local_x, local_y) = point.z;
                StoreAllPoints[local_x + local_y * map_size_x].push_back(i);
            }
        }
    }
    void EigenToImage(Eigen::MatrixXf heatmap, int flag)
    {
        cv::Mat image = cv::Mat(heatmap.rows(), heatmap.cols(), CV_8UC1, cv::Scalar(0));
        for (int i = 0; i < heatmap.rows(); i++)
        {
            for (int j = 0; j < heatmap.cols(); j++)
            {
                if (heatmap(i, j) == 9999)
                    continue;
                image.at<uchar>(i, j) = int(heatmap(i, j) / 6 * 255);
            }
        }

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        out_msg.image = image;
        if (flag)
            pub_image_Map.publish(out_msg.toImageMsg());
        else
            pub_image_Cur.publish(out_msg.toImageMsg());
    }
    void laserCloudHandle3(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pclCloudXYZ temp;
        pcl::fromROSMsg(*msg, temp);
        pclCloudXYZI laserCloudIn;
        pcl::copyPointCloud(temp, laserCloudIn);

        vector<int> NANIndices;
        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, NANIndices);

        Publish(pubOctree_Out, laserCloudIn);
        if (indexFrame < 600)
            laserCloudMap += laserCloudIn;

        voxel.setInputCloud(laserCloudMap.makeShared());
        voxel.filter(laserCloudMap);
        Publish(pubOctree_Map, laserCloudMap);

        if (indexFrame > 400) //开始匹配
        {
            if (indexFrame == 401)
            {
                pcl::getMinMax3D(laserCloudMap, MapMinPt, MapMaxPt);

                cout << MapMinPt.x << " " << MapMinPt.y << " " << MapMaxPt.x << " " << MapMaxPt.y << endl;

                returnEigen(laserCloudMap, MapMinPt, MapMaxPt, HeatForMap);
            }
            EigenToImage(HeatForMap, 1);
            Eigen::MatrixXf HeatForCur;

            map<int, vector<int>> StoreAllPoints;
            returnEigenForCur(laserCloudIn, MapMinPt, MapMaxPt, HeatForCur, StoreAllPoints);
            EigenToImage(HeatForCur, 0);

            map<float, int, greater<float>> heightAndIndex; //存着所有 距离 和 下标

            int index = 0;
            for (int i = 0; i < HeatForCur.rows(); i++)
                for (int j = 0; j < HeatForCur.cols(); j++)
                {
                    if (HeatForCur(i, j) == 9999 || HeatForMap(i, j) == 9999)
                        continue;
                    index++;
                    heightAndIndex.insert(make_pair(abs(HeatForCur(i, j) - HeatForMap(i, j)), i + j * HeatForCur.rows()));
                }

            index = heightAndIndex.size() * 0.3;

            pclCloudXYZI laserCloudComparisonForMove, laserCloudComparisonForStatic, laserCloudComparison;

            vector<int> laserCloudComparisonIndices;

            for (auto iter : heightAndIndex)
            {
                if (index-- <= 0 || iter.first < 0.06)
                    break;

                PointXYZ point;
                point.x = iter.second % HeatForCur.rows();
                point.y = iter.second / HeatForCur.rows();

                if (point.x < 4 || point.x > HeatForCur.rows() - 4 || point.y < 4 || point.y > HeatForCur.cols() - 4)
                    continue;

                // point x y 代表下标，在heatmap中的下标  而这个下标可以在 storeAllPoints中检索到 所有在 laserCloudIn 中的下标

                for (auto iter : StoreAllPoints[iter.second])
                    laserCloudComparisonIndices.push_back(iter);

                // point.z = HeatForCur((int)point.x, (int)point.y);
                // point.x = point.x * (MapMaxPt.x - MapMinPt.x) / map_size_x + MapMinPt.x;
                // point.y = point.y * (MapMaxPt.y - MapMinPt.y) / map_size_y + MapMinPt.y;
                // laserCloudComparison.push_back(point);
            }

            if (laserCloudComparisonIndices.empty())
            {
                pcl::copyPointCloud(laserCloudIn, laserCloudComparisonForStatic);
            }
            else
            {
                extract.setInputCloud(laserCloudIn.makeShared());
                extract.setIndices(pcl::IndicesPtr(new vector<int>(laserCloudComparisonIndices)));
                extract.setNegative(false);
                extract.filter(laserCloudComparisonForMove);
                extract.setNegative(true);
                extract.filter(laserCloudComparisonForStatic);
            }

            Publish(pubOctree_ComparisonMove, laserCloudComparisonForMove);
            Publish(pubOctree_ComparisonStatic, laserCloudComparisonForStatic);

            //开始聚类  首先投影xy平面，周围格子都视为 潜在动态点   然后这些潜在动态点 在用
            kdtree.setInputCloud(laserCloudIn.makeShared());
            queue<PointXYZI> QuaryPoints;
            pclCloudXYZI laserCloudDBSCANResult;
            for (auto point : laserCloudComparisonForMove)
            {
                point.intensity = 20; //标记动态点 20
                QuaryPoints.push(point);
            }

            while (!QuaryPoints.empty())
            {
                PointXYZI point = QuaryPoints.front();
                QuaryPoints.pop();

                laserCloudDBSCANResult.push_back(point);
                if (laserCloudDBSCANResult.size() > 300)
                    break;

                kdtree.nearestKSearch(point, 5, kdtreeIndice, kdtreeDistance);

                for (int i = 0; i < kdtreeIndice.size(); i++)
                {
                    if (kdtreeDistance[i] < 0.05 * 0.05) //距离过进 基本上就是相同聚类了
                    {
                        if (laserCloudIn[kdtreeIndice[i]].intensity == 20)
                            continue;
                        point.intensity = 20;
                        QuaryPoints.push(point);
                    }
                    // else if (1) //考虑角度
                    // {
                    // }
                }
            }
            Publish(pubOctree_DBSCAN, laserCloudDBSCANResult);

            cout << laserCloudComparisonForMove.size() << " " << laserCloudDBSCANResult.size() << endl;
        }

        indexFrame++;
    }
};

/*
输入参数：订阅 两个雷达、两个IMU、两个GPS
输出参数：发布 匹配好的雷达两个激光帧
主要作用：两个模组的匹配 发布  发布出来的激光帧首先 进入到 octreeGrid 进行预处理，然后在进入到ALOAM中定位
        然后定位的激光镇
*/
class mapping
{
private:
    ros::Subscriber subLaserCloudVel, subImuVel, subGpsVel, subLaserCloudHok, subImuHok, subGpsHok;
    ros::Publisher pubMotorLidarVel, pubMotorLidarHok;
    Eigen::Quaterniond InertialValuesVel, InertialValuesHok;
    Eigen::Vector3d GPSValuesVel, GPSValuesHok;

    //直通滤波
    pcl::PassThrough<PointXYZ> pass_z, pass_frame; //设置滤波器对象
    pcl::CropBox<PointXYZ> roof;

    ros::NodeHandle n;
    int rateDecreseVel = -1, rateDecreseHok = -1;
    int initFlag = 0;

public:
    mapping()
    {
        n.param<int>("initFlag", initFlag, 0);
        if (initFlag == 1)
            init();

        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-10, 5);
        roof.setNegative(true);
        roof.setMin(Eigen::Vector4f(-10, -10, -0.5, 1));
        roof.setMax(Eigen::Vector4f(10, 10, 0.5, 1));

        subLaserCloudVel = n.subscribe<sensor_msgs::PointCloud>("/pioneer3at/Velodyne_VLP_16/point_cloud", 10, &mapping::laserCloudVelHandle, this);
        subImuVel = n.subscribe<sensor_msgs::Imu>("/pioneer3at/imu2/quaternion", 100, &mapping::ImuVelHandler, this);
        subGpsVel = n.subscribe<geometry_msgs::PointStamped>("/pioneer3at/gps2/values", 100, &mapping::GPSVelHandler, this);
        pubMotorLidarVel = n.advertise<sensor_msgs::PointCloud2>("/Velodyne_PointCloud_Register", 10);

        subLaserCloudHok = n.subscribe<sensor_msgs::PointCloud>("/pioneer3at/Hokuyo_UTM_30LX/point_cloud", 10, &mapping::laserCloudHokHandle, this);
        subImuHok = n.subscribe<sensor_msgs::Imu>("/pioneer3at/imu/quaternion", 100, &mapping::ImuHokHandler, this);
        subGpsHok = n.subscribe<geometry_msgs::PointStamped>("/pioneer3at/gps/values", 100, &mapping::GPSHokHandler, this);
        pubMotorLidarHok = n.advertise<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_Register", 10);
    }
    void init()
    {
        ros::ServiceClient SensorService;
        webots_ros::set_int initSrv;
        initSrv.request.value = 1;
        SensorService = n.serviceClient<webots_ros::set_int>("/pioneer3at/imu2/enable");
        if (SensorService.call(initSrv) && initSrv.response.success)
            ROS_INFO("IMU2 enabled.");
        else
        {
            ROS_ERROR("Failed to enable IMU2.");
            return;
        }
        SensorService = n.serviceClient<webots_ros::set_int>("/pioneer3at/gps2/enable");
        if (SensorService.call(initSrv) && initSrv.response.success)
            ROS_INFO("GPS2 enabled.");
        else
        {
            ROS_ERROR("Failed to enable GPS2.");
            return;
        }

        // enable lidar
        ros::ServiceClient set_Vel_enable, set_Hok_enable;
        webots_ros::set_int Vel_srv, Hok_srv;

        Vel_srv.request.value = 1;
        Hok_srv.request.value = 1;

        set_Vel_enable = n.serviceClient<webots_ros::set_int>("/pioneer3at/Velodyne_VLP_16/enable");
        set_Hok_enable = n.serviceClient<webots_ros::set_int>("/pioneer3at/Hokuyo_UTM_30LX/enable");
        if (set_Vel_enable.call(Vel_srv) && Vel_srv.response.success)
            ROS_INFO("Vel  enabled.");
        else
        {
            ROS_ERROR("Failed to enable Vel.");
            return;
        }
        webots_ros::set_float Vel_Hz, Hok_Hz;
        Vel_Hz.request.value = 10.0;
        Hok_Hz.request.value = 10;
        set_Vel_enable = n.serviceClient<webots_ros::set_float>("/pioneer3at/Velodyne_VLP_16/set_frequency");
        // set_Hok_enable = n.serviceClient<webots_ros::set_float>("/pioneer3at/Hokuyo_UTM_30LX/enable");
        if (set_Vel_enable.call(Vel_Hz) && Vel_Hz.response.success)
            ROS_INFO("Vel HZ 10 enabled.");
        else
        {
            ROS_ERROR("Failed to enable Vel HZ.");
            return;
        }

        webots_ros::set_bool Vel_cloud_srv, Hok_cloud_srv;
        Vel_cloud_srv.request.value = true;
        Hok_cloud_srv.request.value = true;

        set_Vel_enable = n.serviceClient<webots_ros::set_bool>("/pioneer3at/Velodyne_VLP_16/enable_point_cloud");
        set_Hok_enable = n.serviceClient<webots_ros::set_bool>("/pioneer3at/Hokuyo_UTM_30LX/enable_point_cloud");

        if (set_Vel_enable.call(Vel_cloud_srv) && Vel_cloud_srv.response.success)
            ROS_INFO("Vel Hok enabled Cloud.");
        else
        {
            ROS_ERROR("Failed to enable Cloud Vel Hok.");
            return;
        }
    }
    void laserCloudVelHandle(const sensor_msgs::PointCloudConstPtr &msg)
    {
        rateDecreseVel = (++rateDecreseVel) % 3;
        if (rateDecreseVel != 0)
            return;

        pclCloudXYZ laserCloudIn;
        sensor_msgs::PointCloud2 pointcloud2;
        sensor_msgs::convertPointCloudToPointCloud2(*msg, pointcloud2);
        pcl::fromROSMsg(pointcloud2, laserCloudIn);

        Eigen::Matrix4d transform_;
        transform_.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix() * InertialValuesVel.matrix();
        transform_.block<3, 1>(0, 3) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix() * GPSValuesVel;
        pcl::transformPointCloud(laserCloudIn, laserCloudIn, transform_);

        Publish(pubMotorLidarVel, laserCloudIn);
    }
    void ImuVelHandler(const sensor_msgs::ImuConstPtr &imumsg)
    {
        ROS_INFO_ONCE("IMUVEl GET");
        InertialValuesVel = Eigen::Quaterniond(imumsg->orientation.w, imumsg->orientation.x, imumsg->orientation.y, imumsg->orientation.z);
    }
    void GPSVelHandler(const geometry_msgs::PointStampedConstPtr &gpsmsg)
    {
        ROS_INFO_ONCE("GPSVEL GET");
        GPSValuesVel[0] = gpsmsg->point.x;
        GPSValuesVel[1] = gpsmsg->point.y;
        GPSValuesVel[2] = gpsmsg->point.z;
    }

    void laserCloudHokHandle(const sensor_msgs::PointCloudConstPtr &msg)
    {
        pclCloudXYZ laserCloudIn;
        sensor_msgs::PointCloud2 pointcloud2;
        sensor_msgs::convertPointCloudToPointCloud2(*msg, pointcloud2);
        pcl::fromROSMsg(pointcloud2, laserCloudIn);

        // pass_frame.setInputCloud(laserCloudIn.makeShared());
        // pass_frame.filter(laserCloudIn);

        roof.setInputCloud(laserCloudIn.makeShared());
        roof.filter(laserCloudIn); //去车

        Eigen::Matrix4d transform_;

        transform_.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix() * InertialValuesHok.matrix() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()).matrix();
        transform_.block<3, 1>(0, 3) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix() * GPSValuesHok;
        pcl::transformPointCloud(laserCloudIn, laserCloudIn, transform_);

        pass_z.setInputCloud(laserCloudIn.makeShared());
        pass_z.filter(laserCloudIn);

        Publish(pubMotorLidarHok, laserCloudIn);
    }
    void ImuHokHandler(const sensor_msgs::ImuConstPtr &imumsg)
    {
        InertialValuesHok = Eigen::Quaterniond(imumsg->orientation.w, imumsg->orientation.x, imumsg->orientation.y, imumsg->orientation.z);
    }
    void GPSHokHandler(const geometry_msgs::PointStampedConstPtr &gpsmsg)
    {
        GPSValuesHok[0] = gpsmsg->point.x;
        GPSValuesHok[1] = gpsmsg->point.y;
        GPSValuesHok[2] = gpsmsg->point.z;
    }
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "finalmapping");

    pcl::io::loadPCDFile("HokForOctree.pcd", laserCloudForOctreeFirst);

    // interpolation iter;
    // segmentation seg(iter); //
    // octoGridmap octo;
    octreeGrid octree;


    // mapping map;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}
