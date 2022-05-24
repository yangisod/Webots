#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/Int32Stamped.h>
#define TIME_STEP 32
#define NMOTORS 4
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 1 // 0.9

ros::NodeHandle *n;

static std::vector<float> lidarValues;

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};

static double GPSValues[2] = {0, 0};
static double inertialUnitValues[4] = {0, 0, 0, 0};

static int lms291Resolution = 0;
static int halfResolution = 0;
static double maxRange = 0.0;
static double rangeThreshold = 0.0;
static std::vector<double> braitenbergCoefficients;
static bool areBraitenbergCoefficientsinitialized = false;

static double lposition = 0;
static double rposition = 0;

ros::Time keyTime;
int firstkey = -1;
int coutinueflag = 0;
double speeds[NMOTORS];

ros::Publisher VelPub, HokPub;

void quit(int sig)
{
  ROS_INFO("User stopped the 'pioneer3at' node.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value)
{
  firstkey = 0;

  keyTime = ros::Time::now();
  int key = value->data;
  int send = 0, ifdir = -1;
  // double speeds[NMOTORS];
  switch (key)
  {
  case 314:
    // speeds[0] = MAX_SPEED / 2;
    // speeds[1] = MAX_SPEED;

    speeds[1] = speeds[1] + 1.;
    speeds[0] = (speeds[1] > 0) ? speeds[1] / 2 : speeds[1] - 1.;

    speeds[2] = BACK_SLOWDOWN * speeds[0];
    speeds[3] = BACK_SLOWDOWN * speeds[1];
    send = 1;
    break;
  case 316:
    // speeds[0] = MAX_SPEED;
    // speeds[1] = MAX_SPEED / 2;
    speeds[0] = speeds[0] + 1.;
    speeds[1] = (speeds[0] > 0) ? speeds[0] / 2 : speeds[0] - 1.; //

    speeds[2] = BACK_SLOWDOWN * speeds[0]; // BACK_SLOWDOWN *
    speeds[3] = BACK_SLOWDOWN * speeds[1]; //
    // std::cout << "旋转 " << speeds[0] << " " << speeds[1] << " " << speeds[2] << " " << speeds[3] << std::endl;
    send = 1;
    break;
  case 315:
    if (send == 2)
    {
      speeds[0] = 0.5;
      speeds[1] = 0.5;
    }
    speeds[0] = speeds[0] + 1.;
    speeds[1] = speeds[1] + 1.;

    speeds[2] = speeds[0];
    speeds[3] = speeds[1];

    // std::cout << "直行 " << speeds[0] << " " << speeds[1] << " " << speeds[2] << " " << speeds[3] << std::endl;

    send = 1;
    break;
  case 317:
    if (send == 2)
    {
      speeds[0] = -0.3;
      speeds[1] = -0.3;
    }
    speeds[0] = speeds[0] - 1.;
    speeds[1] = speeds[1] - 1.;

    speeds[2] = speeds[0];
    speeds[3] = speeds[1];
    // std::cout << "后退 " << speeds[0] << " " << speeds[1] << " " << speeds[2] << " " << speeds[3] << std::endl;
    send = 1;
    break;
  case 312:
    ROS_INFO("END.");
    send = 0;
    quit(-1);
    break;
  case 81:
    send = 2;
    break;
  default:
    send = 0;
    break;
  }
  if (send == 1)
  {
    // set speeds
    for (int i = 0; i < NMOTORS; ++i)
    {
      ros::ServiceClient set_velocity_client;
      webots_ros::set_float set_velocity_srv;
      set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                    std::string("/set_velocity")); // set_position
      if (speeds[i] > MAX_SPEED)
        speeds[i] = MAX_SPEED;
      else if (speeds[i] < -MAX_SPEED)
        speeds[i] = -MAX_SPEED;
      // std::cout << speeds[i] << " ";
      set_velocity_srv.request.value = speeds[i];
      set_velocity_client.call(set_velocity_srv);
    }
    // std::cout << std::endl;
  }
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name)
{
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

int main(int argc, char **argv)
{
  std::string controllerName;
  // create a node named 'pioneer3at' on ROS network
  ros::init(argc, argv, "pioneer3at", ros::init_options::AnonymousName);
  n = new ros::NodeHandle;

  // VelPub = n->advertise<sensor_msgs::PointCloud2>("VelCloud", 1);
  // HokPub = n->advertise<sensor_msgs::PointCloud2>("HokCloud", 1);

  keyTime = ros::Time::now();

  setlocale(LC_CTYPE, "zh_CN.utf8");
  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers())
  {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  timeStepClient = n->serviceClient<webots_ros::set_int>("pioneer3at/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  // if there is more than one controller available, it let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else
  {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else
    {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  // leave topic once it is not necessary anymore
  nameSub.shutdown();

  // init motors
  for (int i = 0; i < NMOTORS; ++i)
  {
    // position
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
  }

  ros::ServiceClient enableKeyboardClient = n->serviceClient<webots_ros::set_int>(controllerName + "/keyboard/enable");
  webots_ros::set_int enableKeyboardSrv;
  enableKeyboardSrv.request.value = TIME_STEP;
  ros::Subscriber sub_keyboard;
  if (enableKeyboardClient.call(enableKeyboardSrv) && enableKeyboardSrv.response.success)
  {
    sub_keyboard = n->subscribe(controllerName + "/keyboard/key", 1, keyboardCallback);
    ros::Rate rate(10);
    while (sub_keyboard.getNumPublishers() == 0)
    {
      ROS_WARN("没有连接到keyboard");
      rate.sleep();
    }
    ROS_INFO("连接到keyboard");
  }
  else
  {
    ROS_ERROR("Failed to enable key.");
    return 1;
  }

  for (int i = 0; i < NMOTORS; ++i)
  {
    // position
    ros::ServiceClient MovePos = n->serviceClient<webots_ros::set_float>(std::string("MoveCar/") + std::string(motorNames[i]) +
                                                                         std::string("/set_position"));
    webots_ros::set_float PosSrv;
    PosSrv.request.value = 80; // INFINITY
    if (MovePos.call(PosSrv) && PosSrv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("MoveCar/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 4.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
  }

  while (ros::ok())
  {

    ros::Duration during = ros::Time::now() - keyTime;
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
    {
      ROS_ERROR("Failed to call service time_step for next step.");
      break;
    }
    if (firstkey != -1 && during.toSec() > 1)
    {
      firstkey++;

      for (int i = 0; i < NMOTORS; ++i)
      {
        ros::ServiceClient set_velocity_client;
        webots_ros::set_float set_velocity_srv; // pioneer3at
        set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                      std::string("/set_velocity")); // set_position
        set_velocity_srv.request.value = int(speeds[i] / firstkey);
        set_velocity_client.call(set_velocity_srv);
      }
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}
