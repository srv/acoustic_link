#include "ros/ros.h"
#include "cola2_control/Setpoints.h"
#include "evologics_driver/AcousticModemPayload.h"
#include <boost/lexical_cast.hpp>
#include <vector>


/*
Node to transform a topic "cola2_control/thrusters_data" to "im/out" in an AcousticModemPayload.msg
TODO: Format of the payload to contain the data string
*/


std::vector<double> thrusters;

class topic_link
{
public:
  topic_link()
  {
    sub_topic = n.subscribe("cola2_control/thrusters_data", 1, &topic_link::thrusters_dataCallback, this);
    pub_topic = n.advertise<evologics_driver::AcousticModemPayload>("im/in", 1);
  }

  void thrusters_dataCallback(const cola2_control::Setpoints& thrusters_msg)
  {
    std::vector<double> thrusters = thrusters_msg.setpoints;
    std::string payload;

    // Print
    /*
    ROS_INFO("RECIVED THRUSTERS");
    for (int i = 0; i < 3; ++i)
    {
      ROS_INFO("Thruster [%i]: [%f]",i, thrusters.at(i));
    }
    ROS_INFO("=================");
*/

    //Convert and publish//
    evologics_driver::AcousticModemPayload ac_thrusters_msg;
    //Adress
    ac_thrusters_msg.address = 1;
    //Payload
    std::string type = boost::lexical_cast<std::string>(0); // Type 0 for topics, 1 for services
    std::string ID = boost::lexical_cast<std::string>(1);   // ID of the topic
    std::string data;                                       // Data of the topic
    std::string separator = boost::lexical_cast<std::string>(',');
    payload.append(type);
    payload.append(separator);
    payload.append(ID);

    for (int i = 0; i < 3; ++i)
    {
      payload.append(separator);
      data = boost::lexical_cast<std::string>(thrusters.at(i));
      payload.append(data);
    }
    ac_thrusters_msg.payload = payload;
    
    pub_topic.publish(ac_thrusters_msg);
  }


private:
  ros::NodeHandle n; 
  ros::Subscriber sub_topic;
  ros::Publisher pub_topic;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_out_node");

  topic_link thrusters_link;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}