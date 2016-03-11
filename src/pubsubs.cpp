#include <ros/ros.h>
#include "cola2_control/Setpoints.h"
#include "evologics_driver/AcousticModemPayload.h"

class AcousticLink
{
public:
  AcousticLink()
  {
    required_topics_check();

    // Subscribers
    sub_thrusters_data = n.subscribe("cola2_control/thrusters_data", 1, &Session::thrusters_dataCallback, this);
    sub_emus_bms = n.subscribe("cola2_safety/emus_bms", 1, &Session::emus_bmsCallback, this);
    sub_nav_sts = n.subscribe("cola2_navigation/nav_sts", 1, &Session::nav_stsCallback, this);
    sub_loop_closings = n.subscribe("slam/loop_closings", 1, &Session::loop_closingsCallback, this);
    sub_keyframes = n.subscribe("slam/keyframes", 1, &Session::keyframesCallback, this);
    sub_modem_position = n.subscribe("/usbl_position/modem_position", 1, &Session::modem_positionCallback, this);
    sub_evologics = n.subscribe("/im/in", 1, &Session::evologicsCallback, this);

    // Publishers
    pub_topic = n.advertise<evologics_driver::AcousticModemPayload>("im/out", 1);

    // Services
    //ser_service = n.advertiseService("/lights_on", lights_on_publisher);
  }

  void required_topics_check() {
    // Get node address
    XmlRpc::XmlRpcValue param;
    ros::param::get("~station_address", param);
    int station = (int)param;
    ROS_INFO_STREAM("station: " << station);

    // Publishers
    if (ros::param::has("~topics")) fill_info("~topics", station);
    else ROS_WARN("Failed to establish the TOPIC connections dictated by require parameter.");

    // Services
    if (ros::param::has("~services")) fill_info("~services",station);
    else ROS_WARN("Failed to establish the SERVICE connections dictated by require parameter.");
  }

  void thrusters_dataCallback(const cola2_control::Setpoints& msg)
  {
    evologics_driver::AcousticModemPayload acoustic_msg;
    // Use de input......................
    acoustic_msg.address = 1;
    acoustic_msg.payload = 'e';

    pub_.publish(output);
  }

  void fill_info(std::string param_name, int station) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++) {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      rosserial_msgs::TopicInfo topic_info;
      if (param_name == "~topics"){
        topic_info.topic_id = (int)data["topic_id"];
        topic_info.topic_name = (std::string)data["topic_name"];
        topic_info.message_type = (std::string)data["message_type"];
        topic_info.address = (int)data["destination_address"];
        topic_info.drop = (int)data["drop"];
        if (topic_info.address == station){ 
          setup_publisher(topic_info);
          ROS_INFO_STREAM("\n\tSETUP: PUBLISHER\n\t Topic name: " << topic_info.topic_name << "\n\t Station: " << station);
        }
        else {
          setup_subscriber(topic_info);
          ROS_INFO_STREAM("\n\tSETUP: SUBSCRIBER\n\t Topic name: " << topic_info.topic_name << "\n\t Station: " << station);
        } 
      }
      else{
        topic_info.topic_id = (int)data["service_id"];
        topic_info.topic_name = (std::string)data["service_name"];
        topic_info.message_type = "std_srvs/Empty";
        topic_info.address = (int)data["owner_address"];
        if (topic_info.address != station) {
          setup_service_publisher(topic_info);
          ROS_INFO_STREAM("\n\tSETUP: SERVICE PUBLISHER\n\t Topic name: " << topic_info.topic_name << "\n\t Station: " << station);
        } 
          else {
            setup_service_subscriber(topic_info);
            ROS_INFO_STREAM("\n\tSETUP: SERVICE SUBSCRIBER\n\t Topic name: " << topic_info.topic_name << "\n\t Station: " << station);
          } 
        }
    }
  }

  void setup_publisher(rosserial_msgs::TopicInfo topic_info) {
    PublisherPtr pub(new Publisher(nh_, topic_info));
    publishers_[topic_info.topic_id] = pub;
    callbacks_[topic_info.topic_id] = boost::bind(&Publisher::handle, pub, _1);
  }

  void setup_subscriber(rosserial_msgs::TopicInfo topic_info) {
    SubscriberPtr sub(new Subscriber(nh_, topic_info,
        boost::bind(&Session::write_message, this, _1, topic_info, client_version)));
    subscribers_[topic_info.topic_id] = sub;
  }

  void setup_service_publisher(rosserial_msgs::TopicInfo topic_info) {
    if (!service_publishers_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service publisher for topic %s",topic_info.topic_name.c_str());
      ServiceServerPtr srv(new ServiceServer(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, topic_info, client_version)));
      service_publishers_[topic_info.topic_name] = srv;
    }
  }

  void setup_service_subscriber(rosserial_msgs::TopicInfo topic_info) {
    if (!service_subscribers_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service subscriber for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(nh_,topic_info));
      service_subscribers_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class AcousticLink

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class AcousticLink that will take care of everything
  AcousticLink SAPObject;

  ros::spin();

  return 0;
}