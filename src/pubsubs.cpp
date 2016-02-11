#include <ros/ros.h>
#include "cola2_control/Setpoints.h"
#include "evologics_driver/AcousticModemPayload.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<evologics_driver::AcousticModemPayload>("im/out", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("cola2_control/thrusters_data", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const cola2_control::Setpoints& input)
  {
    evologics_driver::AcousticModemPayload output;
    // Use de input......................
    output.address = 1;
    output.payload = 'e';

    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}