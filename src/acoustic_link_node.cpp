#include "ros/ros.h"

#include <string> 
#include <sstream>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>


#include "control/Setpoints.h"
#include "safety/EMUSBMS.h"
#include "auv_msgs/NavSts.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "evologics_ros/AcousticModemPayload.h"
#include "std_srvs/Empty.h"

// INSERT NEW TOPICS
// Complete Callback, genericCb, Parser, setup_topic, handles

// INSERT NEW SERVICES
// Complete Callback, genericCb, setup_service, handles


int station;

struct Topic
{
  int id;
  std::string name;
  std::string msg;
  int address;
  int num; // Position of the topic in the topics vector
  int drop;
  int counter;
};

struct Service
{
  int id;
  std::string name;
  std::string msg;
  int address;
};

std::vector<Topic> topics;
//std::vector<Service> services;



class Session
{
public:
  Session()
  {

  }

  void start()
  {
    ROS_INFO("Starting session.");

    instant_sub_ = n.subscribe<evologics_ros::AcousticModemPayload>("im/in", 1, &Session::genericCb, this);
    instant_pub_ = n.advertise<evologics_ros::AcousticModemPayload>("im/out", 1);

    // List the required topics
    required_topics_check();
  }

  bool dropTopic(const Topic t)
  {
    //ROS_INFO_STREAM("dropTopic: " << topics[t.num].counter << "/" << t.drop);
    if (topics[t.num].counter < t.drop){
      topics[t.num].counter ++;
      return true;
    }
    else {
      topics[t.num].counter = 1;
      return false;
    }
  }

//CALLBACKS
  void setpointsCallback(const control::Setpoints::ConstPtr& msg, const Topic t)
  {
    if (dropTopic(t)) return;

    ROS_INFO("setpointsCallback");
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg->header;
    acoustic_msg.ack = true;
    acoustic_msg.address = t.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id)); 
    list.push_back(boost::lexical_cast<std::string>((float)msg->setpoints[0]));
    list.push_back(boost::lexical_cast<std::string>((float)msg->setpoints[1]));
    list.push_back(boost::lexical_cast<std::string>((float)msg->setpoints[2]));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  setpoints");
  }

  void emus_bmsCallback(const safety::EMUSBMS::ConstPtr& msg, const Topic t)
  {
    if (dropTopic(t)) return;
    ROS_INFO("emus_bmsCallback");
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg->header;
    acoustic_msg.ack = true;
    acoustic_msg.address = t.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id)); 
    list.push_back(boost::lexical_cast<std::string>((float)msg->stateOfCharge));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  emus_bms");
  }

  void nav_stsCallback(const auv_msgs::NavSts::ConstPtr& msg, const Topic t)
  {
    if (dropTopic(t)) return;
    ROS_INFO("nav_stsCallback");
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg->header;
    
    acoustic_msg.ack = true;
    acoustic_msg.address = t.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id)); 
    //list.push_back(boost::lexical_cast<std::string>((float)msg->global_position.latitude));
    //list.push_back(boost::lexical_cast<std::string>((float)msg->global_position.longitude));
    list.push_back(boost::lexical_cast<std::string>((float)msg->position.north));
    list.push_back(boost::lexical_cast<std::string>((float)msg->position.east));
    list.push_back(boost::lexical_cast<std::string>((float)msg->position.depth));
    //list.push_back(boost::lexical_cast<std::string>((float)msg->altitude));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  nav_sts");
  }

  void stringCallback(const std_msgs::String::ConstPtr& msg, const Topic t)
  {
    if (dropTopic(t)) return;
    ROS_INFO("stringCallback");
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header.stamp = ros::Time::now();
    acoustic_msg.ack = true;
    acoustic_msg.address = t.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id)); 
    list.push_back(msg->data);
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  string");
  }


  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, const Topic t)
  {
    if (dropTopic(t)) return;
    ROS_INFO("poseCallback");
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = msg->header;
    acoustic_msg.ack = true;
    acoustic_msg.address = t.address;

    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id)); 
    list.push_back(boost::lexical_cast<std::string>((float)msg->pose.pose.position.x));
    list.push_back(boost::lexical_cast<std::string>((float)msg->pose.pose.position.y));
    list.push_back(boost::lexical_cast<std::string>((float)msg->pose.pose.position.z));
    list.push_back(boost::lexical_cast<std::string>((float)msg->pose.covariance[0]));
    list.push_back(boost::lexical_cast<std::string>((float)msg->pose.covariance[7]));
    list.push_back(boost::lexical_cast<std::string>((float)msg->pose.covariance[14]));
    acoustic_msg.payload = boost::algorithm::join(list, ",");
    
    if (acoustic_msg.payload.size() <= 64) instant_pub_.publish(acoustic_msg); 
    else ROS_WARN("Payload size bigger than expected:  PoseWithCovarianceStampedCallback");
  }

  bool empty_srvCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, const Service s)
  {
    ROS_INFO("empty_srvCallback");
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header.stamp = ros::Time::now();
    acoustic_msg.ack = true;
    acoustic_msg.address = s.address;
    acoustic_msg.payload = boost::lexical_cast<std::string>(s.id);
    
    instant_pub_.publish(acoustic_msg); 
  }

private:
  // EDIT for new topics
  void genericCb(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg) 
  {
    // Extract message
    std::string payload = acoustic_msg->payload;

    std::vector<std::string> data;
    boost::split(data, payload, boost::is_any_of(","));
    

    // Extract topic id
    int id = boost::lexical_cast<int>(data[0]);
    data.erase(data.begin());
  
    if (id == 20) 
    {
      control::Setpoints msg;
      setpointsParse(acoustic_msg, data, msg);
      pub_setpoints_.publish(msg);
      ROS_INFO("PUBLISH: thrusters_data");
    }
    if (id == 21) 
    {
      safety::EMUSBMS msg;
      emus_bmsParse(acoustic_msg, data, msg);
      pub_emus_bms_.publish(msg);
      ROS_INFO("PUBLISH: emus_bms");
    }
    if (id == 22) 
    {
      auv_msgs::NavSts msg;
      nav_stsParse(acoustic_msg, data, msg);
      pub_nav_sts_.publish(msg);
      ROS_INFO("PUBLISH: nav_sts");
    }
    if (id == 23) 
    {
      std_msgs::String msg;
      stringParse(acoustic_msg, data, msg);
      pub_loop_closings_.publish(msg);
      ROS_INFO("PUBLISH: loop_closings");
    }
    if (id == 24) 
    {
      std_msgs::String msg;
      stringParse(acoustic_msg, data, msg);
      pub_keyframes_.publish(msg);
      ROS_INFO("PUBLISH: keyframes ");
    }
    if (id == 25) 
    {
      geometry_msgs::PoseWithCovarianceStamped msg;
      poseParse(acoustic_msg, data, msg);
      pub_modem_position_.publish(msg);
      ROS_INFO("PUBLISH: modem_position");
    }
    if (id == 26) 
    {
      geometry_msgs::PoseWithCovarianceStamped msg;
      poseParse(acoustic_msg, data, msg);
      pub_depth_raw_.publish(msg);
      ROS_INFO("PUBLISH: depth_raw");
    }
    if (id == 27) 
    {
      std_msgs::String msg;
      stringParse(acoustic_msg, data, msg);
      pub_ping_.publish(msg);
      ROS_INFO("PUBLISH: ping ");
    }
    if (id >= 40) // Services 
    {
      std_srvs::Empty srv;
      if (id == 40) cli_sonar_on.call(srv);
      if (id == 41) cli_sonar_off.call(srv);
      if (id == 42) cli_bagfile_on.call(srv);
      if (id == 43) cli_bagfile_off.call(srv);
      if (id == 44) cli_trajectory_on.call(srv);
      if (id == 45) cli_trajectory_off.call(srv);
      if (id == 46) cli_thrusters_on.call(srv);
      if (id == 47) cli_thrusters_off.call(srv);
      if (id == 48) cli_recording_on.call(srv);
      if (id == 49) cli_recording_off.call(srv);      
      if (id == 50) cli_lights_on.call(srv);
      if (id == 51) cli_lights_off.call(srv);
      if (id == 52) cli_laser_on.call(srv);
      if (id == 53) cli_laser_off.call(srv);
      if (id == 54) cli_reset_navigation.call(srv);
      if (id == 55) cli_usbl_on.call(srv);
      if (id == 56) cli_usbl_off.call(srv);
      ROS_INFO("Calling Service");
    }
  }

  void setpointsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, 
                      const std::vector<std::string> data,
                      control::Setpoints& msg)
  {
    msg.header = acoustic_msg->header;
    msg.setpoints.resize(data.size());
    msg.setpoints[0] = atof(data[0].c_str());
    msg.setpoints[1] = atof(data[1].c_str());
    msg.setpoints[2] = atof(data[2].c_str());
  }

  void emus_bmsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, 
                     const std::vector<std::string> data,
                     safety::EMUSBMS& msg)
  {
    msg.header = acoustic_msg->header;
    msg.stateOfCharge = atof(data[0].c_str());
  }

  void nav_stsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, 
                    const std::vector<std::string> data,
                    auv_msgs::NavSts& msg)
  {
    msg.header = acoustic_msg->header;
    //msg.global_position.latitude = atof(data[0].c_str());
    //msg.global_position.longitude = atof(data[1].c_str());
    msg.position.north = atof(data[0].c_str());
    msg.position.east = atof(data[1].c_str());
    msg.position.depth = atof(data[2].c_str());
    //msg.altitude = atof(data[5].c_str());
  }

  void stringParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, 
                   const std::vector<std::string> data,
                   std_msgs::String& msg)
  { 
    //msg.header = acoustic_msg->header.stamp;
    msg.data = atof(data[0].c_str());
  }

  void poseParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg, 
                 const std::vector<std::string> data,
                 geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    msg.header = acoustic_msg->header;
    msg.pose.pose.position.x = atof(data[0].c_str());
    msg.pose.pose.position.y = atof(data[1].c_str());
    msg.pose.pose.position.z = atof(data[2].c_str());
    msg.pose.covariance[0] = atof(data[3].c_str());
    msg.pose.covariance[7] = atof(data[4].c_str());
    msg.pose.covariance[13] = atof(data[5].c_str());
  }

  void required_topics_check() 
  {
    ROS_INFO("required_topics_check");
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

  void fill_info(const std::string param_name) 
  {
    ROS_INFO("fill_info");
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++) 
    {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
      if (param_name == "~topics")
      {
        Topic t; // Struct
        t.id = (int)data["topic_id"];
        t.name = (std::string)data["topic_name"];
        t.msg = (std::string)data["message_type"];
        t.address = (int)data["destination_address"];
        t.num = topics.size();
        t.drop = (int)data["drop"];
        t.counter = t.drop;
        topics.push_back(t);
        setup_topic(t);
      }
      else{
        Service s; // Struct
        s.id = (int)data["service_id"];
        s.name = (std::string)data["service_name"];
        s.msg = "std_srvs/Empty"; //Introduce other kind of services if necessary
        s.address = (int)data["owner_address"];
        //services.push_back(s);
        setup_service(s);
      }
      
    }
  }

  // EDIT for new topics
  void setup_topic(const Topic t) 
  {
    ROS_INFO("setup_topic");
    if (t.address == station)
    {
      if (t.id == 20)
        pub_setpoints_ = n.advertise<control::Setpoints>(t.name, 1);
      if (t.id == 21)
        pub_emus_bms_ = n.advertise<safety::EMUSBMS>(t.name, 1);
      if (t.id == 22)
        pub_nav_sts_ = n.advertise<auv_msgs::NavSts>(t.name, 1);
      if (t.id == 23)
        pub_loop_closings_ = n.advertise<std_msgs::String>(t.name, 1);
      if (t.id == 24)
        pub_keyframes_ = n.advertise<std_msgs::String>(t.name, 1);
      if (t.id == 25)
        pub_modem_position_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(t.name, 1);
      if (t.id == 26)
        pub_depth_raw_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(t.name, 1);
      if (t.id == 27)
        pub_ping_ = n.advertise<std_msgs::String>(t.name, 1);
    }
    else 
    {
      if (t.id == 20)
        sub_setpoints_ = n.subscribe<control::Setpoints>(t.name, 1, boost::bind(&Session::setpointsCallback, this, _1, t));
      if (t.id == 21)
        sub_emus_bms_ = n.subscribe<safety::EMUSBMS>(t.name, 1, boost::bind(&Session::emus_bmsCallback, this, _1, t));
      if (t.id == 22)
        sub_nav_sts_ = n.subscribe<auv_msgs::NavSts>(t.name, 1, boost::bind(&Session::nav_stsCallback, this, _1, t));
      if (t.id == 23)
        sub_loop_closings_ = n.subscribe<std_msgs::String>(t.name, 1, boost::bind(&Session::stringCallback, this, _1, t));
      if (t.id == 24)
        sub_keyframes_ = n.subscribe<std_msgs::String>(t.name, 1, boost::bind(&Session::stringCallback, this, _1, t));
      if (t.id == 25)
        sub_modem_position_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(t.name, 1, boost::bind(&Session::poseCallback, this, _1, t));
      if (t.id == 26)
        sub_depth_raw_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(t.name, 1, boost::bind(&Session::poseCallback, this, _1, t));
      if (t.id == 27)
        sub_ping_ = n.subscribe<std_msgs::String>(t.name, 1, boost::bind(&Session::stringCallback, this, _1, t));
    
    }
  }
   
  void setup_service(const Service s) 
  {
    //, All services are Empty
    if (s.address == station)
    {
      if (s.id == 40)
        srv_sonar_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 41)
        srv_sonar_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 42)
        srv_bagfile_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 43)
        srv_bagfile_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 44)
        srv_trajectory_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 45)
        srv_trajectory_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 46)
        srv_thrusters_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 47)
        srv_thrusters_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 48)
        srv_recording_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 49)
        srv_recording_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 50)
        srv_lights_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 51)
        srv_lights_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 52)
        srv_laser_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 53)
        srv_laser_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 54)
        srv_reset_navigation = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 55)
        srv_usbl_on = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.id == 56)
        srv_usbl_off = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name,  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
    }
    else {
      if (s.id == 40) cli_sonar_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 41) cli_sonar_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 42) cli_bagfile_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 43) cli_bagfile_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 44) cli_trajectory_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 45) cli_trajectory_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 46) cli_thrusters_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 47) cli_thrusters_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 48) cli_recording_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 49) cli_recording_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 50) cli_lights_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 51) cli_lights_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 52) cli_laser_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 53) cli_laser_off = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 54) cli_reset_navigation = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 55) cli_usbl_on = n.serviceClient<std_srvs::Empty>(s.name);
      if (s.id == 56) cli_usbl_off = n.serviceClient<std_srvs::Empty>(s.name);
    }
    

  }


  ros::NodeHandle n;
  ros::Subscriber instant_sub_;
  ros::Publisher instant_pub_;

  ros::Subscriber sub_setpoints_;
  ros::Subscriber sub_emus_bms_;
  ros::Subscriber sub_nav_sts_;
  ros::Subscriber sub_loop_closings_;
  ros::Subscriber sub_keyframes_;
  ros::Subscriber sub_modem_position_;
  ros::Subscriber sub_depth_raw_;
  ros::Subscriber sub_ping_;
  ros::Publisher  pub_setpoints_;
  ros::Publisher  pub_emus_bms_;
  ros::Publisher  pub_nav_sts_;
  ros::Publisher  pub_loop_closings_;
  ros::Publisher  pub_keyframes_;
  ros::Publisher  pub_modem_position_;
  ros::Publisher  pub_depth_raw_;
  ros::Publisher  pub_ping_;

  ros::ServiceServer srv_sonar_on;
  ros::ServiceServer srv_sonar_off;
  ros::ServiceServer srv_bagfile_on;
  ros::ServiceServer srv_bagfile_off;
  ros::ServiceServer srv_trajectory_on;
  ros::ServiceServer srv_trajectory_off;
  ros::ServiceServer srv_thrusters_on;
  ros::ServiceServer srv_thrusters_off;
  ros::ServiceServer srv_recording_on;
  ros::ServiceServer srv_recording_off;
  ros::ServiceServer srv_lights_on;
  ros::ServiceServer srv_lights_off;
  ros::ServiceServer srv_laser_on;
  ros::ServiceServer srv_laser_off;
  ros::ServiceServer srv_reset_navigation;
  ros::ServiceServer srv_usbl_on;
  ros::ServiceServer srv_usbl_off;

  ros::ServiceClient cli_sonar_on;
  ros::ServiceClient cli_sonar_off;
  ros::ServiceClient cli_bagfile_on;
  ros::ServiceClient cli_bagfile_off;
  ros::ServiceClient cli_trajectory_on;
  ros::ServiceClient cli_trajectory_off;
  ros::ServiceClient cli_thrusters_on;
  ros::ServiceClient cli_thrusters_off;
  ros::ServiceClient cli_recording_on;
  ros::ServiceClient cli_recording_off;
  ros::ServiceClient cli_lights_on;
  ros::ServiceClient cli_lights_off;
  ros::ServiceClient cli_laser_on;
  ros::ServiceClient cli_laser_off;
  ros::ServiceClient cli_reset_navigation;
  ros::ServiceClient cli_usbl_on;
  ros::ServiceClient cli_usbl_off;
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