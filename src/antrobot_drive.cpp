#include "antrobot_drive.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
//
//#include "std_msgs/String.h"
//#include "sensor_msgs/Range.h"
using namespace std;

//#define COM_PORT_NUMBER 17 // /dev/ttyUSB0
#define COM_PORT_NUMBER 17 // /dev/ttyUSB1
//#define COM_PORT_NUMBER 22 // /dev/ttyAMA0

bool is_antrobot = false;
bool pos_with_imu = false;
bool command_sent = false;
int linear_distance = 3;
int angular_distance = 3;
double robot_pos_x = 0;
double robot_pos_y = 0;
double robot_pos_theta = 0;
double imu_last_update;
double imu_current_update;
geometry_msgs::Vector3 last_accel, current_accel;
geometry_msgs::Quaternion last_orien, current_orien;
double x_dist = 0, x_dist_last = 0;
double x_vel = 0, x_vel_last = 0;
ros::Publisher drv_dbg;
ros::Subscriber sub_cmd_vel;
ros::Subscriber sub_imu_data;

#define NANO_EXP 1000000000
double dt;

char forward[6], backward[6], t_left[6], t_right[6];
string move_forward, move_backward, turn_left, turn_right;

void controllerCallback(const geometry_msgs::TwistConstPtr &msg) {
   if (!command_sent) {
      float dx = msg->linear.x; // ruch do przodu
      float dy = msg->linear.y;
      float dr = msg->angular.z;  // obrot
      //std::cout << "Received message" << std::endl;
      //std::vector<double> response(3, 0);
      if (fabs(dx) > fabs(dr)) {
         if (dx > 0) {
            send_command(move_forward);
         } else if (dx < 0) {
            send_command(move_backward);
         }
      } else if (fabs(dx) < fabs(dr)) {
         if (dr > 0) {
            send_command(turn_right);
         } else if (dr < 0) {
            send_command(turn_left);
         }
      } else {
         send_command("!NN0");
      }
   }
   //std::cout << std::endl;
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg) {

   //cout << "Received imu message" << endl;
   //string frame = msg->header.frame_id;
   imu_last_update = imu_current_update;
   imu_current_update = (msg->header.stamp.sec / NANO_EXP) + msg->header.stamp.nsec;
   // linear acceleration
   last_accel = current_accel;
   current_accel = msg->linear_acceleration;
   // quaternion
   last_orien = current_orien;
   current_orien = msg->orientation;

   //dt = imu_current_update - imu_last_update;


   x_vel_last = x_vel;
   x_vel = x_vel + ((current_accel.x + last_accel.x) / 2);
   x_dist = x_dist + ((x_vel + x_vel_last) / 2);
   cout << "Acc: " << current_accel.x << ", " << last_accel.x << ", vel: " << x_vel << ", dist: " <<
   x_dist << endl;
   //std::string message;
   std_msgs::Float64 text;
   text.data = x_dist;
   drv_dbg.publish(text);
}

int main(int argc, char **argv) {
   if (const char *env_antrobot = std::getenv("DEVICE_WITH_ANTROBOT_CONTROLLER")) {
      int i = strcmp(env_antrobot, "true");
      if (i == 0) {
         std::cout << "Program is running on antrobot.\n";
         is_antrobot = true;
      } else {
         std::cout << "Program is not running on robot. Switching to debug mode.\n";
         is_antrobot = false;
      }
   } else {
      std::cout <<
      "Can not find DEVICE_WITH_ANTROBOT_CONTROLLER variable.\nSwitching to debug mode.\n";
      is_antrobot = false;
   }

   if (is_antrobot) {
      init_comport(COM_PORT_NUMBER);
   }

   //std::vector<double> response(3, 0);
   send_command("!RST");

   ros::init(argc, argv, "antrobot_drive_node");
   ros::NodeHandle n("~");

   if (n.getParam("lin_dist", linear_distance)) {
      if (linear_distance > 99) {
         linear_distance = 99;
      }
      if (linear_distance < 0) {
         linear_distance = 0;
      }
   }
   printf("Running with linear drive step %d centimeters\n", linear_distance);
   sprintf(&forward[0], "!NN%d", linear_distance);
   move_forward = string(forward);
   sprintf(&backward[0], "!BB%d", linear_distance);
   move_backward = string(backward);


   if (n.getParam("ang_dist", angular_distance)) {
      if (angular_distance > 99) {
         angular_distance = 99;
      }
      if (angular_distance < 0) {
         angular_distance = 0;
      }
   }
   printf("Running with angular drive step %d degrees\n", angular_distance);
   sprintf(&t_left[0], "!--%d", angular_distance);
   turn_left = string(t_left);
   sprintf(&t_right[0], "!++%d", angular_distance);
   turn_right = string(t_right);

   if (n.getParam("use_imu", pos_with_imu)) {
      if (pos_with_imu) {
         printf("Will use imu to approximate position during movement.\n");
      } else {
         printf("Will NOT use imu.\n");
      }
   }

   cout << "Drive node running" << endl;
   // Subscribing to velocity topic
   sub_cmd_vel = n.subscribe("/cmd_vel", 1, controllerCallback);
   std::cout << "Subscribed to topic '/cmd_vel'" << std::endl;

   if (pos_with_imu) {
      //Subscribing to imu topic
      sub_imu_data = n.subscribe("/imu/data", 1, imuCallback);
      std::cout << "Subscribed to topic '/imu/data'" << std::endl;
   }

   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
   std::cout << "Advertising to topic '/odom'" << std::endl;
   //tf::TransformBroadcaster odom_broadcaster;
   ros::Time current_time, last_time;
   current_time = ros::Time::now();
   last_time = ros::Time::now();

   drv_dbg = n.advertise<std_msgs::Float64>("/driver_debug", 50);
   std::cout << "Advertising to topic '/driver_debug'" << std::endl;

   ros::Rate loop_rate(10); // 10Hz taka sama nazwa "loop_rate" pożniej ma być uzyta do .sleep
   //ros::spinOnce();
   while (ros::ok()) {
      //std::cout << "Sending transform." << std::endl;
      current_time = ros::Time::now();
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pos_theta);
      /*geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "world";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = robot_pos_x;
      odom_trans.transform.translation.y = robot_pos_y;

      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster.sendTransform(odom_trans);*/

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "world";

      odom.pose.pose.position.x = robot_pos_x;
      odom.pose.pose.position.y = robot_pos_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "robot_base";
      //odom.twist.twist.linear.x = vx;
      //odom.twist.twist.linear.y = vy;
      //odom.twist.twist.angular.z = vth;

      //publish the message
      //std::cout << "Publishing message." << std::endl;
      odom_pub.publish(odom);

      last_time = current_time;

      if (command_sent) {
         std::vector<double> response(3, 0);
         check_for_response(&response);
         robot_pos_x = response[1];
         robot_pos_y = response[2];
         robot_pos_theta = response[0];
      }

      //std::cout << "Spin once." << std::endl;
      ros::spinOnce();
      //std::cout << "sleep." << std::endl;
      //loop_rate.sleep();
   }

   if (is_antrobot) {
      close_comport(COM_PORT_NUMBER);
   }

   return (0);
}


int send_command(string command) { // theta, x, y
   int ret_val = 0;
   //cout << "Sending command: " << command << endl;
   unsigned char one_sign[1];
   //std::vector<double> pos = *position;
   if (!is_antrobot) { //debugging
      //std::cout << "Command to send: " << command << " Skip waiting for response\n";
      //pos[0] = 10;
      //pos[1] = 20;
      //pos[2] = 30;
      ///*position = pos;
      ret_val = -1;
   } else {
      // sending command
      int s_len = command.length();
      for (int i = 0; i < s_len; i++) {
         RS232_SendByte(COM_PORT_NUMBER, command[i]);
      }
      RS232_SendByte(COM_PORT_NUMBER, 13);
      command_sent = true;
      ret_val = 0;
   }
   return ret_val;
}

void check_for_response(std::vector<double> *position) {
   // reading response
   unsigned char one_sign[1];
   std::vector<double> pos = *position;

   int status = RS232_PollComport(COM_PORT_NUMBER, one_sign, 1);
   if (status == 1) {
      if (one_sign[0] != '!') {
         while (status == 1 && one_sign[0] != '!') {
            // read all unnecesary signs
            status = RS232_PollComport(COM_PORT_NUMBER, one_sign, 1);
         }
         if (one_sign[0] == '!') {
            //read line
         } else {
            return;
         }
      } else {
         // read line
      }
   } else {
      return;
   }
   int n = 0;
   string digit;
   for (int i = 0; i < 7; i++) {
      digit = "";
      while (one_sign[0] != 9 && one_sign[0] != 13) {
         digit += one_sign[0];
         while (n == 0) {  // waiting for new sign
            n = RS232_PollComport(COM_PORT_NUMBER, one_sign, 1);
         }
         n = 0;
      }
      if (i == 4) { //theta
         pos[0] = (((double) atoi(digit.c_str())) * M_PI) / 180.0;
      }
      if (i == 5) {
         pos[1] = ((double) atoi(digit.c_str())) / 100.0;
      }
      if (i == 6) {
         pos[2] = ((double) atoi(digit.c_str())) / 100.0;
      }
      while (n == 0) {  // waiting for new sign
         n = RS232_PollComport(COM_PORT_NUMBER, one_sign, 1);
      }
      command_sent = false;
      n = 0;
   }
   *position = pos;
   // print robot current position
   //std::cout << "Robot is at position: Theta=" << pos[0] << " X=" << pos[1] << " Y=" << pos[2] <<
   // std::endl;
}

int init_comport(int port_number) {
   int bdrate = 115200;       /* 115200 baud */
   char mode[] = {'8', 'N', '1', 0};
   if (RS232_OpenComport(port_number, bdrate, mode)) {
      printf("Can not open comport\n");
      return (0);
   }
}

int close_comport(int port_number) {
   RS232_CloseComport(port_number);
}