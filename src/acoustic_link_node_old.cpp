#include "ros/ros.h"
#include "cola2_control/Setpoints.h"
#include "evologics_driver/AcousticModemPayload.h"




std::vector<double> thrusters;



void thrusters_dataCallback(const cola2_control::Setpoints& thrusters_msg)
{
  ROS_INFO("RECIVED THRUSTERS");
  thrusters = thrusters_msg.setpoints;

  for (int i = 0; i < 3; ++i)
  {
    ROS_INFO("Thruster [%i]: [%f]",i, thrusters.at(i));
  }
  ROS_INFO("=================");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acoustic_link_node");
  
  ros::NodeHandle n;
  //Subscribers
  ros::Subscriber sub = n.subscribe("cola2_control/thrusters_data", 1000, thrusters_dataCallback);

  //Publishers
  ros::Publisher pub = n.advertise<evologics_driver::AcousticModemPayload>("im/out", 1000);

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    evologics_driver::AcousticModemPayload ac_thrusters_msg;

    //Publish IM/OUT-->Put it in a separate function
    ac_thrusters_msg.address = 1;
    ac_thrusters_msg.payload = 'e';
    pub.publish(ac_thrusters_msg);
    //

    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}