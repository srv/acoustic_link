#include "ros/ros.h"
#include "evologics_driver/AcousticModemPayload.h"
#include <string> 
#include <sstream>


/*
Node to transform a service "/lights_on" to "im/out" in an AcousticModemPayload.msg
TODO: Format of the payload to contain the data string
*/

class service_link
{
public:

  service_link()
  {
    srv = n.advertiseService("/lights_on", lights);
    pub_srv = n.advertise<evologics_driver::AcousticModemPayload>("im/out", 1);
  }

  void lights()
  {
  	evologics_driver::AcousticModemPayload lights_on_srv;
  	lights_on_srv.address = 1;
  	/*
  	std::string type = '1'; // Service
  	std::string X = '6'; 	  // ID
  	std::string Data = '1'; // ON/OFF
  	lights_on_srv.payload = type + X + Data;
    */
    pub_srv.publish(lights_on_srv);
  }

private:
  ros::NodeHandle n; 
  ros::ServiceServer srv;
  ros::Publisher pub_srv;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_out_node");

  service_link lights_link;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}