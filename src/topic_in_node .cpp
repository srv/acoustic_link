#include "ros/ros.h"
#include "cola2_control/Setpoints.h"
#include "evologics_driver/AcousticModemPayload.h"

/*
Node to transform a topic "cola2_control/thrusters_data" to "im/out" in an AcousticModemPayload.msg
TODO: Pending to build (it is a copy of topic_out_node.cpp)
*/

std::vector<double> thrusters;

class topic_link
{
public:
  topic_link()
  {
    //TOPIC
    sub_topic = n.subscribe("cola2_control/thrusters_data", 1, &topic_link::thrusters_dataCallback, this);
    pub_topic = n.advertise<evologics_driver::AcousticModemPayload>("im/out", 1);

    //SERVICE
    //ser_service = n.advertiseService("/lights_on", lights_on_publisher);
  }

  void thrusters_dataCallback(const cola2_control::Setpoints& thrusters_msg)
  {
    std::vector<double> thrusters = thrusters_msg.setpoints;
    std::string payload;
    // Save message in a vector and print
    ROS_INFO("RECIVED THRUSTERS");
    for (int i = 0; i < 3; ++i)
    {
      ROS_INFO("Thruster [%i]: [%f]",i, thrusters.at(i));
    }
    ROS_INFO("=================");

    //Convert and publish//
    evologics_driver::AcousticModemPayload ac_thrusters_msg;
    //Adress
    ac_thrusters_msg.address = 1;
    //Payload
    int type = 0;
    string Number;
    ostringstream convert;
    convert << type;
    Result = convert.str();

    payload.push_back(Result);
    /*
    for (int i = 0; i < 3; ++i)
    {
      payload.push_back((char)thrusters.at(i));//Repair the change of format, it is not well converted
    }
    */
    ac_thrusters_msg.payload = payload;
    pub_topic.publish(ac_thrusters_msg);
  }

  void lights_on_publisher()
  {

  }

private:
  ros::NodeHandle n; 
  ros::Subscriber sub_topic;
  ros::Publisher pub_topic;
  //ros::ServiceServer ser_service;
  //ros::Publisher pub_service;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "acoustic_link_node");

  topic_link thrusters_link;



  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}