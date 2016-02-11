#include "ros/ros.h"
#include "cola2_control/Setpoints.h"
#include <vector>

double Thrusters[] = {1,2,3};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cola2_control::Setpoints>("cola2_control/thrusters_data", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cola2_control::Setpoints thrusters_msg;
    

    for (int i = 0; i < 3; ++i)
    {
      thrusters_msg.setpoints.push_back(Thrusters[i]);
    }


    pub.publish(thrusters_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}