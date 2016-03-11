#include "ros/ros.h"

#include <string> 
#include <sstream>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <rosserial_msgs/TopicInfo.h>

#include <rosserial_msgs/Log.h>
#include <std_msgs/Time.h>

#include "control/Setpoints.h"
#include "safety/EMUSBMS.h"
#include "auv_msgs/NavSts.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "evologics_ros/AcousticModemPayload.h"


int station;
rosserial_msgs::TopicInfo topic_info_thrusters_data;
rosserial_msgs::TopicInfo topic_info_emus_bms;
rosserial_msgs::TopicInfo topic_info_nav_sts;
rosserial_msgs::TopicInfo topic_info_loop_closings;
rosserial_msgs::TopicInfo topic_info_keyframes;
rosserial_msgs::TopicInfo topic_info_modem_position;


class Session
{
public:
  Session()
  {

  }

  void start()
  {
    ROS_INFO("Starting session.");

    // List the required topics
    required_topics_check();

    instant_sub_ = n.subscribe<evologics_ros::AcousticModemPayload>("im/in", 1, &Session::genericCb, this);
    instant_pub_ = n.advertise<evologics_ros::AcousticModemPayload>("im/out", 1);
  }

//CALLBACKS
  void thrusters_dataCallback(const control::Setpoints& msg)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg.header;
    acoustic_msg.ack = true;
    acoustic_msg.address = topic_info_thrusters_data.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(topic_info_thrusters_data.topic_id)); 
    list.push_back(boost::lexical_cast<std::string>((float)msg.setpoints[0]));
    list.push_back(boost::lexical_cast<std::string>((float)msg.setpoints[1]));
    list.push_back(boost::lexical_cast<std::string>((float)msg.setpoints[2]));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  thrusters_data");
  }

  void emus_bmsCallback(const safety::EMUSBMS& msg)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg.header;
    acoustic_msg.ack = true;
    acoustic_msg.address = topic_info_thrusters_data.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(topic_info_emus_bms.topic_id)); 
    list.push_back(boost::lexical_cast<std::string>((float)msg.stateOfCharge));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  emus_bms");
  }

  void nav_stsCallback(const auv_msgs::NavSts& msg)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg.header;
    
    acoustic_msg.ack = true;
    acoustic_msg.address = topic_info_nav_sts.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(topic_info_nav_sts.topic_id)); 
    //list.push_back(boost::lexical_cast<std::string>((float)msg.global_position.latitude));
    //list.push_back(boost::lexical_cast<std::string>((float)msg.global_position.longitude));
    list.push_back(boost::lexical_cast<std::string>((float)msg.position.north));
    list.push_back(boost::lexical_cast<std::string>((float)msg.position.east));
    list.push_back(boost::lexical_cast<std::string>((float)msg.position.depth));
    //list.push_back(boost::lexical_cast<std::string>((float)msg.altitude));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  nav_sts");
  }

  void loop_closingsCallback(const std_msgs::String& msg)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header.stamp = ros::Time::now();
    acoustic_msg.ack = true;
    acoustic_msg.address = topic_info_loop_closings.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(topic_info_keyframes.topic_id)); 
    list.push_back(msg.data);
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  loop_closings");
  }

  void keyframesCallback(const std_msgs::String& msg)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header.stamp = ros::Time::now();
    acoustic_msg.ack = true;
    acoustic_msg.address = topic_info_keyframes.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(topic_info_keyframes.topic_id)); 
    list.push_back(msg.data);
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  keyframes");
  }

  void modem_positionCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg.header;
    acoustic_msg.ack = true;
    acoustic_msg.address = topic_info_modem_position.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(topic_info_modem_position.topic_id)); 
    list.push_back(boost::lexical_cast<std::string>((float)msg.pose.pose.position.x));
    list.push_back(boost::lexical_cast<std::string>((float)msg.pose.pose.position.y));
    list.push_back(boost::lexical_cast<std::string>((float)msg.pose.pose.position.z));
    list.push_back(boost::lexical_cast<std::string>((float)msg.pose.covariance[0]));
    list.push_back(boost::lexical_cast<std::string>((float)msg.pose.covariance[7]));
    list.push_back(boost::lexical_cast<std::string>((float)msg.pose.covariance[13]));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  modem_position");
  }

private:
    //// RECEIVING MESSAGES ////

  void genericCb(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg) {

    // Extract message
    std::string payload = acoustic_msg->payload;

    std::vector<std::string> data;
    boost::split(data, payload, boost::is_any_of(","));
    

    // Extract topic id
    int topic_id = boost::lexical_cast<int>(data[0]);
    data.erase(data.begin());

    switch (topic_id)
      {
        case 20:
            thrusters_dataParse(acoustic_msg, data);
          break;
        case 21:
            emus_bmsParse(acoustic_msg, data);
          break;
        case 22:
            nav_stsParse(acoustic_msg, data);
          break;
        case 23:
            loop_closingsParse(acoustic_msg, data);
          break;
        case 24:
            keyframesParse(acoustic_msg, data);
          break;
        case 25:
            modem_positionParse(acoustic_msg, data);
          break;
      }
  }

  void thrusters_dataParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, const std::vector<std::string> data)
  {
    control::Setpoints msg;
    msg.header = acoustic_msg->header;
    msg.setpoints.resize(data.size());
    msg.setpoints[0] = atof(data[0].c_str());
    msg.setpoints[1] = atof(data[1].c_str());
    msg.setpoints[2] = atof(data[2].c_str());
    pub_thrusters_data_.publish(msg);
    ROS_INFO("PUBLISH: thrusters_data");
  }

  void emus_bmsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, const std::vector<std::string> data)
  {
    safety::EMUSBMS msg;
    msg.header = acoustic_msg->header;
    msg.stateOfCharge = atof(data[0].c_str());
    pub_emus_bms_.publish(msg);
    ROS_INFO("PUBLISH: emus_bms");
  }

  void nav_stsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, const std::vector<std::string> data)
  {
    auv_msgs::NavSts msg;
    msg.header = acoustic_msg->header;
    //msg.global_position.latitude = atof(data[0].c_str());
    //msg.global_position.longitude = atof(data[1].c_str());
    msg.position.north = atof(data[0].c_str());
    msg.position.east = atof(data[1].c_str());
    msg.position.depth = atof(data[2].c_str());
    //msg.altitude = atof(data[5].c_str());
    pub_nav_sts_.publish(msg);
    ROS_INFO("PUBLISH: nav_sts");
  }

  void loop_closingsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, const std::vector<std::string> data)
  {
    std_msgs::String msg;
    //msg.header = acoustic_msg->header;
    msg.data = atof(data[0].c_str());
    pub_loop_closings_.publish(msg);
    ROS_INFO("PUBLISH: loop_closings");
  }

  void keyframesParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, const std::vector<std::string> data)
  { //TODO change, topics to msg selection
    std_msgs::String msg;
    //msg.header = acoustic_msg->header.stamp;
    msg.data = atof(data[0].c_str());
    pub_keyframes_.publish(msg);
    ROS_INFO("PUBLISH: keyframes");
  }

  void modem_positionParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, const std::vector<std::string> data)
  {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header = acoustic_msg->header;
    msg.pose.pose.position.x = atof(data[0].c_str());
    msg.pose.pose.position.y = atof(data[1].c_str());
    msg.pose.pose.position.z = atof(data[2].c_str());
    msg.pose.covariance[0] = atof(data[3].c_str());
    msg.pose.covariance[7] = atof(data[4].c_str());
    msg.pose.covariance[13] = atof(data[5].c_str());
    pub_modem_position_.publish(msg);
    ROS_INFO("PUBLISH: modem_position");
  }
  void required_topics_check() 
  {
    // Get node address
    XmlRpc::XmlRpcValue param;
    ros::param::get("~station_address", param);
    station = (int)param;
    ROS_INFO_STREAM("station: " << station);

    // Publishers
    if (ros::param::has("~topics")) fill_info("~topics");
    else ROS_WARN("Failed to establish the TOPIC connections dictated by require parameter.");

    // Services
    if (ros::param::has("~services")) fill_info("~services");
    else ROS_WARN("Failed to establish the SERVICE connections dictated by require parameter.");
  }

  void fill_info(std::string param_name) 
  {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++) 
    {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      rosserial_msgs::TopicInfo topic_info;
      if (param_name == "~topics"){
        topic_info.topic_id = (int)data["topic_id"];
        topic_info.topic_name = (std::string)data["topic_name"];
        topic_info.message_type = (std::string)data["message_type"];
        topic_info.address = (int)data["destination_address"];
        topic_info.drop = (int)data["drop"];
        setup_topic(topic_info);
      }
      else{
        topic_info.topic_id = (int)data["service_id"];
        topic_info.topic_name = (std::string)data["service_name"];
        topic_info.message_type = "std_srvs/Empty";
        topic_info.address = (int)data["owner_address"];
        //if (topic_info.address != station) {
        //  setup_service_publisher(topic_info);
        //  ROS_INFO_STREAM("\n\tSETUP: SERVICE PUBLISHER\n\t Topic name: " << topic_info.topic_name << "\n\t Station: " << station);
        //} 
        //  else {
        //    setup_service_subscriber(topic_info);
        //    ROS_INFO_STREAM("\n\tSETUP: SERVICE SUBSCRIBER\n\t Topic name: " << topic_info.topic_name << "\n\t Station: " << station);
        //  } 
        }
    }
  }

  //// RECEIVED MESSAGE HANDLERS ////

  void setup_topic(const rosserial_msgs::TopicInfo topic_info) 
  {
    bool pub = false;
    if ((int)topic_info.address == station) pub = true;
    
    switch (topic_info.topic_id)
    {
      case 20:
        topic_info_thrusters_data = topic_info;
        if (pub) 
          pub_thrusters_data_ = n.advertise<control::Setpoints>(topic_info.topic_name, 1);
        else 
          sub_thrusters_data_ = n.subscribe(topic_info.topic_name, 1, &Session::thrusters_dataCallback, this);
        //ROS_INFO_STREAM("setup_topic: pub is " << pub << "\t Station" << station );
        break;
      case 21:
        topic_info_emus_bms = topic_info;
        if (pub) 
          pub_emus_bms_ = n.advertise<safety::EMUSBMS>(topic_info.topic_name, 1);
        else 
          sub_emus_bms_ = n.subscribe(topic_info.topic_name, 1, &Session::emus_bmsCallback, this);
        ROS_INFO_STREAM("setup_topic: pub is " << pub << "\t Station" << station );
        break;
      case 22:
        topic_info_nav_sts = topic_info;
        if (pub) 
          pub_nav_sts_ = n.advertise<auv_msgs::NavSts>(topic_info.topic_name, 1);
        else 
          sub_nav_sts_ = n.subscribe(topic_info.topic_name, 1, &Session::nav_stsCallback, this);
        ROS_INFO_STREAM("setup_topic: pub is " << pub << "\t Station" << station );
        break;
      case 23:
        topic_info_loop_closings = topic_info;
        if (pub) 
          pub_loop_closings_ = n.advertise<std_msgs::String>(topic_info.topic_name, 1);
        else 
          sub_loop_closings_ = n.subscribe(topic_info.topic_name, 1, &Session::loop_closingsCallback, this);
        ROS_INFO_STREAM("setup_topic: pub is " << pub << "\t Station" << station );
        break;
      case 24:
        topic_info_keyframes = topic_info;
        if (pub) 
          pub_keyframes_ = n.advertise<std_msgs::String>(topic_info.topic_name, 1);
        else 
          sub_keyframes_ = n.subscribe(topic_info.topic_name, 1, &Session::keyframesCallback, this);
        ROS_INFO_STREAM("setup_topic: pub is " << pub << "\t Station" << station );
        break;
      case 25:
        topic_info_modem_position = topic_info;
        if (pub) 
          pub_modem_position_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_info.topic_name, 1);
        else 
          sub_modem_position_ = n.subscribe(topic_info.topic_name, 1, &Session::modem_positionCallback, this);
        ROS_INFO_STREAM("setup_topic: pub is " << pub << "\t Station" << station );
        break;
    }
  }


  //void setup_service_publisher(rosserial_msgs::TopicInfo topic_info) {
  //  if (!service_publishers_.count(topic_info.topic_name)) {
  //    ROS_DEBUG("Creating service publisher for topic %s",topic_info.topic_name.c_str());
  //    ServiceServerPtr srv(new ServiceServer(
  //      nh_,topic_info,boost::bind(&Session::write_message, this, _1, topic_info, client_version)));
  //    service_publishers_[topic_info.topic_name] = srv;
  //  }
  //}
//
  //void setup_service_subscriber(rosserial_msgs::TopicInfo topic_info) {
  //  if (!service_subscribers_.count(topic_info.topic_name)) {
  //    ROS_DEBUG("Creating service subscriber for topic %s",topic_info.topic_name.c_str());
  //    ServiceClientPtr srv(new ServiceClient(nh_,topic_info));
  //    service_subscribers_[topic_info.topic_name] = srv;
  //    callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
  //  }
  //}

  ros::NodeHandle n;
  ros::Subscriber instant_sub_;
  ros::Publisher instant_pub_;

  ros::Subscriber sub_thrusters_data_;
  ros::Publisher  pub_thrusters_data_;
  ros::Subscriber sub_emus_bms_;
  ros::Publisher  pub_emus_bms_;
  ros::Subscriber sub_nav_sts_;
  ros::Publisher  pub_nav_sts_;
  ros::Subscriber sub_loop_closings_;
  ros::Publisher  pub_loop_closings_;
  ros::Subscriber sub_keyframes_;
  ros::Publisher  pub_keyframes_;
  ros::Subscriber sub_modem_position_;
  ros::Publisher  pub_modem_position_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "acoustic_link_node");

  Session acoustic_link;
  acoustic_link.start();

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}