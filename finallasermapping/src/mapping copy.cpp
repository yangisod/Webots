#include "Motor.h"
using namespace std;

pcl::PointCloud<PointType> laserCloudVelMap, laserCloudHokMap, laserCloudHokMapRegister;
// pcl::PointCloud<PointType> laserCloudVel, laserCloudHok;
ros::Publisher pubMotorLidarVelMap, pubMotorLidarVel, pubMotorLidarVelALoam;
ros::Publisher pubMotorLidarHokMap, pubMotorLidarHok, pubMotorLidarHokmeiduiCloud, pubMotorLidarHokmeiduiCloudKriging, pubMotorLidarHokMapRegister;
ros::Publisher pubMotorLidarCornerPoints, pubMotorLidarCarLocation;
ros::ServiceClient set_position_motor, set_velocity_motor, cloudKriging;
pcl::VoxelGrid<PointType> voxelVel, voxelHok, voxelHokForCeres;

Eigen::Quaterniond InertialValuesHok;
void ImuHandler(const sensor_msgs::ImuConstPtr &imumsg)
{
    ROS_INFO_ONCE("IMU Enabled");
    InertialValuesHok = Eigen::Quaterniond(imumsg->orientation.w, imumsg->orientation.x, imumsg->orientation.y, imumsg->orientation.z);
}
Eigen::Vector3d GPSValuesHok;
void GPSHandler(const geometry_msgs::PointStampedConstPtr &gpsmsg)
{
    ROS_INFO_ONCE("GPS Enabled");
    GPSValuesHok[0] = gpsmsg->point.x;
    GPSValuesHok[1] = gpsmsg->point.y;
    GPSValuesHok[2] = gpsmsg->point.z;
}
Eigen::Quaterniond InertialValuesVel;
void Imu2Handler(const sensor_msgs::ImuConstPtr &imumsg)
{
    ROS_INFO_ONCE("IMU Enabled");
    InertialValuesVel = Eigen::Quaterniond(imumsg->orientation.w, imumsg->orientation.x, imumsg->orientation.y, imumsg->orientation.z);
}
Eigen::Vector3d GPSValuesVel;
pcl::PointCloud<PointType> carlocation;
PointType carLocal;
void GPS2Handler(const geometry_msgs::PointStampedConstPtr &gpsmsg)
{
    ROS_INFO_ONCE("GPS Enabled");
    GPSValuesVel[0] = gpsmsg->point.x;
    GPSValuesVel[1] = gpsmsg->point.y;
    GPSValuesVel[2] = gpsmsg->point.z;
    PointType point;

    point.x = gpsmsg->point.x;
    point.y = -gpsmsg->point.z;
    point.z = 0;
    if (sqrt(pow(point.x - carLocal.x, 2) + pow(point.y - carLocal.y, 2)) > 0.3)
    {
        carlocation.push_back(point);
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(carlocation, msg);
        msg.header.frame_id = "map";
        pubMotorLidarCarLocation.publish(msg);
        carLocal = point;
    }
}
std::mutex mtxVel, mtxHok;
auto Publish = [](ros::Publisher &pub,
                  const pclCloud &cloud)
{
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(cloud, laserCloudOutMsg);
    laserCloudOutMsg.header.frame_id = "map";
    pub.publish(laserCloudOutMsg);
};
int rateDecrese = -1;
void laserCloudHandleVel(const sensor_msgs::PointCloudConstPtr &laserCloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    sensor_msgs::PointCloud2 pointcloud2;
    sensor_msgs::convertPointCloudToPointCloud2(*laserCloudMsg, pointcloud2);
    pcl::fromROSMsg(pointcloud2, laserCloudIn);

    Eigen::Matrix4d transform_;

    pcl::PointCloud<pcl::PointXYZ> AloamCloud;
    transform_.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix();
    pcl::transformPointCloud(laserCloudIn, AloamCloud, transform_);

    transform_.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix() * InertialValuesVel.matrix();
    transform_.block<3, 1>(0, 3) = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).matrix() * GPSValuesVel;
    pcl::transformPointCloud(laserCloudIn, laserCloudIn, transform_);

    Publish(pubMotorLidarVel, laserCloudIn);

    rateDecrese = (++rateDecrese) % 3;
    if (rateDecrese != 0)
        return;
    Publish(pubMotorLidarVelALoam, laserCloudIn);
}
//直通滤波
pcl::PassThrough<PointType> pass_z, pass_frame; //设置滤波器对象
pcl::CropBox<PointType> roof;
void laserCloudHandleHok(const sensor_msgs::PointCloudConstPtr &laserCloudMsg)
{
    ROS_INFO_ONCE("HOK Started");
    pcl::PointCloud<PointType> laserCloudIn;
    sensor_msgs::PointCloud2 pointcloud2;
    sensor_msgs::convertPointCloudToPointCloud2(*laserCloudMsg, pointcloud2);
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

void laserCloudHandleVelRecall(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    laserCloudVelMap += laserCloudIn;

    voxelVel.setInputCloud(laserCloudVelMap.makeShared());
    voxelVel.filter(laserCloudVelMap);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(laserCloudVelMap, msg);
    msg.header.frame_id = "map";
    pubMotorLidarVelMap.publish(msg);
}
pcl::KdTreeFLANN<PointType>::Ptr kdtreelaserCloudOut(new pcl::KdTreeFLANN<PointType>());
std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;
pclCloud pclmap;
void CeresForMap(pclCloud &laserCloudCorner, double parameters[7])
{
    if (pclmap.empty())
        return;

    // ceres优化
    for (int iterCount = 0; iterCount < 2; iterCount++)
    {
        // ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters, 4, q_parameterization);
        problem.AddParameterBlock(parameters + 4, 3);

        int corner_num = 0;

        //点到线的ICP匹配 ！！！！！！！！！！！！！！！！！！！！！！
        for (int i = 0; i < laserCloudCorner.size(); i++)
        {
            PointType pointOri = laserCloudCorner[i];
            // 在 submap 的 corner 特征点(target)中，寻找距离当前帧 corner 特征点(source)最近的5个点
            kdtreelaserCloudOut->nearestKSearch(pointOri, 5, pointSearchInd, pointSearchSqDis);

            //距离小于1
            if (pointSearchSqDis[4] < 1.0)
            {
                std::vector<Eigen::Vector3d> nearCorners;
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(pclmap[pointSearchInd[j]].x,
                                        pclmap[pointSearchInd[j]].y,
                                        pclmap[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                // 计算这个5个最近邻点的中心
                center = center / 5.0;

                // 协方差矩阵
                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }

                // 计算协方差矩阵的特征值和特征向量，用于判断这5个点是不是呈线状分布，此为PCA的原理
                //用来判断
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                // if is indeed line feature
                // note Eigen library sort eigenvalues in increasing order
                // 如果5个点呈线状分布，最大的特征值对应的特征向量就是该线的方向向量
                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                // 如果最大的特征值 >> 其他特征值，则5个点确实呈线状分布，否则认为直线“不够直”
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                {
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    // 从中心点沿着方向向量向两端移动0.1m，构造线上的两个点
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;

                    // 然后残差函数的形式就跟Odometry一样了，残差距离即点到线的距离，到介绍lidarFactor.cpp时再说明具体计算方法
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                    corner_num++;
                }
            }
        }

        // printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
        // printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

        // printf("mapping data assosiation time %f ms \n", t_data.toc());

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // printf("mapping solver time %f ms \n", t_solver.toc());

        // printf("time %f \n", timeLaserOdometry);
        // printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
        // printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
        //        parameters[4], parameters[5], parameters[6]);
    }

    // return parameters;
}
bool isFirstHok = true;
int HokRecallFrameId = 0;
void laserCloudHandleHokRecall(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    if (isFirstHok)
    {
        isFirstHok = false;
        return;
    }

    double time1 = clock();
    pclCloud laserCloudRegister;

    if (HokRecallFrameId < 240)
        laserCloudHokMap += laserCloudIn;
    else if (HokRecallFrameId == 240)
    {
        pcl::copyPointCloud(laserCloudHokMap, pclmap);
        kdtreelaserCloudOut->setInputCloud(pclmap.makeShared());
        laserCloudHokMap.clear();
    }
    else if (HokRecallFrameId > 240)
    {
        double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        voxelHokForCeres.setInputCloud(laserCloudIn.makeShared());
        voxelHokForCeres.filter(laserCloudIn); // 0.2

        CeresForMap(laserCloudIn, parameters); // laserCloudCorner 用ForNew数据包 基本完成了就是有一点点延时 因为是30hz太快了

        // printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
        //        parameters[4], parameters[5], parameters[6]);

        Eigen::Quaterniond qua(parameters);
        Eigen::Vector3d vec(parameters + 4);
        for (auto point : laserCloudIn)
        {
            Eigen::Vector3d point_w = qua * Eigen::Vector3d(point.x, point.y, point.z) + vec;
            point.x = point_w.x();
            point.y = point_w.y();
            point.z = point_w.z();
            laserCloudRegister.push_back(point);
        }
        laserCloudHokMap += laserCloudIn;
        laserCloudHokMapRegister += laserCloudRegister;

        voxelHok.setInputCloud(laserCloudHokMapRegister.makeShared());
        voxelHok.filter(laserCloudHokMapRegister);
        voxelHok.setInputCloud(laserCloudHokMap.makeShared());
        voxelHok.filter(laserCloudHokMap);
    }

    Publish(pubMotorLidarHokMap, laserCloudHokMap);
    Publish(pubMotorLidarHokMapRegister, laserCloudHokMapRegister);

    // cout << (clock() - time1) / CLOCKS_PER_SEC << endl;
    HokRecallFrameId++;
}

int displaycount = 0;
PointType2 minPt, maxPt;
void display_()
{
    ros::Rate rate(1);
    sensor_msgs::PointCloud2 msg;
    while (ros::ok())
    {
        // laserCloudHokMap
        rate.sleep();
    }
}

void INIT(ros::NodeHandle n)
{
    // enable imu
    ros::ServiceClient SensorService;
    webots_ros::set_int initSrv;
    initSrv.request.value = 1;
    SensorService = n.serviceClient<webots_ros::set_int>("/pioneer3at/imu/enable");
    if (SensorService.call(initSrv) && initSrv.response.success)
        ROS_INFO("IMU enabled.");
    else
    {
        ROS_ERROR("Failed to enable IMU.");
        return;
    }
    SensorService = n.serviceClient<webots_ros::set_int>("/pioneer3at/imu2/enable");
    if (SensorService.call(initSrv) && initSrv.response.success)
        ROS_INFO("IMU2 enabled.");
    else
    {
        ROS_ERROR("Failed to enable IMU2.");
        return;
    }

    // enable gps
    SensorService = n.serviceClient<webots_ros::set_int>("/pioneer3at/gps/enable");
    if (SensorService.call(initSrv) && initSrv.response.success)
        ROS_INFO("GPS enabled.");
    else
    {
        ROS_ERROR("Failed to enable GPS.");
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
    if (set_Vel_enable.call(Vel_srv) && set_Hok_enable.call(Hok_srv) && Vel_srv.response.success && Hok_srv.response.success)
        ROS_INFO("Vel Hok enabled.");
    else
    {
        ROS_ERROR("Failed to enable Vel Hok.");
        return;
    }
    webots_ros::set_bool Vel_cloud_srv, Hok_cloud_srv;
    Vel_cloud_srv.request.value = true;
    Hok_cloud_srv.request.value = true;

    set_Vel_enable = n.serviceClient<webots_ros::set_bool>("/pioneer3at/Velodyne_VLP_16/enable_point_cloud");
    set_Hok_enable = n.serviceClient<webots_ros::set_bool>("/pioneer3at/Hokuyo_UTM_30LX/enable_point_cloud");

    if (set_Vel_enable.call(Vel_cloud_srv) && set_Hok_enable.call(Hok_cloud_srv) && Vel_cloud_srv.response.success && Hok_cloud_srv.response.success)
        ROS_INFO("Vel Hok enabled Cloud.");
    else
    {
        ROS_ERROR("Failed to enable Cloud Vel Hok.");
        return;
    }
}
void quit(int sig)
{
    std::cout << "QUIT" << std::endl;
    webots_ros::set_float set_velocity_srv;

    set_velocity_srv.request.value = 10.0;
    if (set_velocity_motor.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
        ROS_INFO("Velocity set to 10.0 for 电机.");
    else
        ROS_ERROR("Failed to call service set_velocity on 电机.");

    // position
    webots_ros::set_float set_position_srv;

    set_position_srv.request.value = 0; // Motor_local
    if (set_position_motor.call(set_position_srv) && set_position_srv.response.success)
        ROS_INFO("Position set to 0 for 电机.");
    else
        ROS_ERROR("Failed to call service set_position on 电机.");

    ros::shutdown();
}
int main(int argc, char **argv)
{
    voxelVel.setLeafSize(0.1, 0.1, 0.1);
    voxelHok.setLeafSize(0.1, 0.1, 0.1);
    voxelHokForCeres.setLeafSize(0.2, 0.2, 0.2);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-10, 5);

    roof.setNegative(true);
    roof.setMin(Eigen::Vector4f(-10, -10, -0.5, 1));
    roof.setMax(Eigen::Vector4f(10, 10, 0.5, 1));
    pass_frame.setFilterFieldName("z");
    pass_frame.setFilterLimits(-10000, 0);

    ros::init(argc, argv, "mapping", ros::init_options::AnonymousName); //, ros::init_options::NoSigintHandler
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::NodeHandle n;
    signal(SIGINT, quit);

    int initFlag = 0;
    n.param<int>("initFlag", initFlag, 0);

    cout<<initFlag<<endl;

    if (initFlag == 1)
    {
        ROS_INFO("初始化传感器");
        INIT(n);
    }
    // INIT(n);

    ros::Subscriber subLaserCloudVel = n.subscribe<sensor_msgs::PointCloud>("/pioneer3at/Velodyne_VLP_16/point_cloud", 1, laserCloudHandleVel);   // /HokCloud
    ros::Subscriber subLaserCloudHok = n.subscribe<sensor_msgs::PointCloud>("/pioneer3at/Hokuyo_UTM_30LX/point_cloud", 100, laserCloudHandleHok); // /HokCloud

    ros::Subscriber subImu = n.subscribe<sensor_msgs::Imu>("/pioneer3at/imu/quaternion", 100, ImuHandler);
    ros::Subscriber subGps = n.subscribe<geometry_msgs::PointStamped>("/pioneer3at/gps/values", 100, GPSHandler);
    ros::Subscriber subImu2 = n.subscribe<sensor_msgs::Imu>("/pioneer3at/imu2/quaternion", 100, Imu2Handler);
    ros::Subscriber subGps2 = n.subscribe<geometry_msgs::PointStamped>("/pioneer3at/gps2/values", 100, GPS2Handler);

    pubMotorLidarVelALoam = n.advertise<sensor_msgs::PointCloud2>("/Velodyne_PointCloud_Loam", 10);
    pubMotorLidarVel = n.advertise<sensor_msgs::PointCloud2>("/Velodyne_PointCloud_Register", 1);
    pubMotorLidarVelMap = n.advertise<sensor_msgs::PointCloud2>("/Velodyne_PointCloud_Map", 10);
    pubMotorLidarHok = n.advertise<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_Register", 1);
    pubMotorLidarHokMap = n.advertise<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_Map", 10, true);
    pubMotorLidarHokMapRegister = n.advertise<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_MapRegister", 10, true);
    pubMotorLidarHokmeiduiCloud = n.advertise<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_meiduiCloud", 10);
    pubMotorLidarCarLocation = n.advertise<sensor_msgs::PointCloud2>("/CarLocation", 10);

    pubMotorLidarHokmeiduiCloudKriging = n.advertise<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_meiduiCloud_Kriging", 10);
    pubMotorLidarCornerPoints = n.advertise<sensor_msgs::PointCloud2>("/CornerPoints", 10);

    ros::Subscriber subLaserCloudVelRecall = n.subscribe<sensor_msgs::PointCloud2>("/Velodyne_PointCloud_Register", 1, laserCloudHandleVelRecall); // /HokCloud
    ros::Subscriber subLaserCloudHokRecall = n.subscribe<sensor_msgs::PointCloud2>("/Hokuyo_PointCloud_Register", 1, laserCloudHandleHokRecall);   // /HokCloud

    // cloudKriging = n.serviceClient<lidar_remove::CloudSrv>("kriging");

    // std::future<void> process_modbus = std::async(std::launch::async, display_);

    webots_ros::set_float set_velocity_srv;
    set_velocity_motor = n.serviceClient<webots_ros::set_float>(std::string("pioneer3at/dianji/set_velocity"));

    set_velocity_srv.request.value = 1.0;
    if (set_velocity_motor.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
        ROS_INFO("Velocity set to 1.0 for 电机.");
    else
        ROS_ERROR("Failed to call service set_velocity on 电机.");

    // position
    webots_ros::set_float set_position_srv;
    set_position_motor = n.serviceClient<webots_ros::set_float>(std::string("pioneer3at/dianji/set_position"));

    set_position_srv.request.value = 1000000000; // Motor_local
    if (set_position_motor.call(set_position_srv) && set_position_srv.response.success)
        ROS_INFO("Position set to 2 M_PI for 电机.");
    else
        ROS_ERROR("Failed to call service set_position on 电机.");

    ros::spin();
    return 0;
}