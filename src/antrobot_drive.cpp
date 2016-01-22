#include "antrobot_drive.h"
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
//#include "std_msgs/String.h"
//#include "sensor_msgs/Range.h"
using namespace std;

bool is_antrobot = false;
double robot_pos_x = 0;
double robot_pos_y = 0;
double robot_pos_theta = 0;

void controllerCallback(const geometry_msgs::TwistConstPtr &msg) {
   float dx = msg->linear.x; // ruch do przodu
   float dy = msg->linear.y;
   float dr = msg->angular.z;  // obrot
   std::cout << "Received message" << std::endl;
   std::vector<double> response(3, 0);
   if (fabs(dx) > fabs(dr)) {
      if (dx > 0) {
         send_command("!NN1", &response);
      } else if (dx < 0) {
         send_command("!BB1", &response);
      }
   } else if (fabs(dx) < fabs(dr)) {
      if (dr > 0) {
         send_command("!++3", &response);
      } else if (dr < 0) {
         send_command("!--3", &response);
      }
   } else {
      send_command("!NN0", &response);
   }
   // print robot current position
   std::cout << "Robot is at position: Theta=" << response[0] << " X=" << response[1] << " Y=" << response[2] <<
   std::endl;
   robot_pos_x = response[1];
   robot_pos_y = response[2];
   robot_pos_theta = response[0];

   std::cout << std::endl;
}

int main(int argc, char **argv) {
   if (const char *env_raspi_led = std::getenv("DEVICE_SELF_ANTROBOT")) {
      int i = strcmp(env_raspi_led, "true");
      if (i == 0) {
         std::cout << "Program is running on antrobot.\n";
         is_antrobot = true;
      } else {
         std::cout << "Program is not running on robot. Switching to debug mode.\n";
         is_antrobot = false;
      }
   } else {
      std::cout << "Can not find DEVICE_SELF_ANTROBOT variable.\nSwitching to debug mode.\n";
      is_antrobot = false;
   }


   std::vector<double> response(3, 0);
   send_command("!RST", &response);
   //const string directions[] = {"WW", "NW", "NN", "NE", "EE", "SE", "SS", "SW"};
   //const int direction_id[] = {6, 7, 8, 9, 10, 11, 12, 13};
   //string values[22];

   ros::init(argc, argv, "antrobot_drive_node");
   ros::NodeHandle n;
   cout << "Drive node running" << endl;
   //ros::Rate loop_rate(3);
   //int count = 0;

   //std::vector<int> response(3, 0);
   //send_command("!NN0", &response);

   //std::cout << "Robot is at position: Theta=" << response[0] << " X=" << response[1] << " Y=" << response[2] << std::endl;


   ros::Subscriber sub = n.subscribe("cmd_vel", 1, controllerCallback);

   std::cout << "Subscribed to topic 'cmd_vel'" << std::endl;

   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
   std::cout << "Advertising to topic 'odom'" << std::endl;
   tf::TransformBroadcaster odom_broadcaster;

   ros::Time current_time, last_time;
   current_time = ros::Time::now();
   last_time = ros::Time::now();

   ros::Rate loop_rate(10); // 10Hz taka sama nazwa "loop_rate" pożniej ma być uzyta do .sleep
   ros::spinOnce();
   while (ros::ok()) {
      current_time = ros::Time::now();
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "world";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = robot_pos_x;
      odom_trans.transform.translation.y = robot_pos_y;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pos_theta);
      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster.sendTransform(odom_trans);

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "world";

      odom.pose.pose.position.x = robot_pos_x;
      odom.pose.pose.position.y = robot_pos_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      //odom.twist.twist.linear.x = vx;
      //odom.twist.twist.linear.y = vy;
      //odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;


      ros::spinOnce();
      loop_rate.sleep();
   }

   return (0);
}


int send_command(string command, std::vector<double> *position) { // theta, x, y
   int cport_nr = 17; //dev/ttyUSB0
   int ret_val = 0;
   unsigned char one_sign[1];
   std::vector<double> pos = *position;
   if (!is_antrobot) { //debugging
      std::cout << "Command to send: " << command << " Skip waiting for response\n";
      pos[0] = 10;
      pos[1] = 20;
      pos[2] = 30;
      //*position = pos;
      ret_val = -1;
   } else {
      // sending command
      init_comport(cport_nr);
      int s_len = command.length();
      for (int i = 0; i < s_len; i++) {
         RS232_SendByte(cport_nr, command[i]);
      }
      RS232_SendByte(cport_nr, 13);

      // reading response
      RS232_PollComport(cport_nr, one_sign, 1);
      while (one_sign[0] != '!') {
         RS232_PollComport(cport_nr, one_sign, 1);
      }

      int n = 0;
      string digit;
      for (int i = 0; i < 7; i++) {
         digit = "";
         while (one_sign[0] != 9 && one_sign[0] != 13) {
            digit += one_sign[0];
            while (n == 0) {  // waiting for new sign
               n = RS232_PollComport(cport_nr, one_sign, 1);
            }
            n = 0;
         }
         std::cout << "Command to send: " << command << " Skip waiting for response\n";
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
            n = RS232_PollComport(cport_nr, one_sign, 1);
         }

         n = 0;
      }

      close_comport(cport_nr);\
    ret_val = 0;
   }
   *position = pos;
   return ret_val;
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