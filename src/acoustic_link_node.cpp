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
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "evologics_ros/AcousticModemPayload.h"
#include "evologics_ros/AcousticModemUSBLLONG.h"
#include "evologics_ros/AcousticModemUSBLANGLES.h"
#include "std_srvs/Empty.h"


#include <iostream>
#include <cstring>
#include <vector>
#include <iomanip>


struct Topic
{
  int id;
  std::string name;
  std::string msg;
  int address;
  bool ack;
  int num; // Position of the topic in the topics_ vector
  int drop;
  int counter;
};

struct Service
{
  int id;
  std::string name;
  std::string msg;
  int address;
  bool ack;

};


class Session
{
public:
  Session() {}

  void start() // TODO: Send to the constructor
  {
    node_name_ = ros::this_node::getName();

    // Im subscribers
    instant_sub_ = n_.subscribe<evologics_ros::AcousticModemPayload>("im/in", 1, &Session::acousticCallback, this);
    instant_pub_ = n_.advertise<evologics_ros::AcousticModemPayload>("im/out", 1);

    // List the required topics
    required_topics_check();

    if (station_ == 1)
      timer_ = n_.createTimer(ros::Duration(0.1), &Session::timerCallback, this);
  }


protected:

  bool dropTopic(const Topic& t)
  {
    //ROS_INFO_STREAM("dropTopic: " << topics_[t.num].counter << "/" << t.drop);
    if (topics_[t.num].counter < t.drop)
    {
      topics_[t.num].counter ++;
      return true;
    }
    else
    {
      topics_[t.num].counter = 1;
      return false;
    }
  }


  void publishIm(const std_msgs::Header header, const std::vector<std::string>& list, const Topic& t)
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.header = header;
    acoustic_msg.ack = t.ack;
    acoustic_msg.address = t.address;
    acoustic_msg.payload = boost::algorithm::join(list, ";");
    if (acoustic_msg.payload.size() <= 64)
    {
      instant_pub_.publish(acoustic_msg);
      send_time_ = ros::Time::now();
    }
    else ROS_WARN_STREAM("[" << node_name_ << "]: Payload size bigger than expected: max 64");
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (ros::Time::now() - send_time_ > ros::Duration(2.0))
    {
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      std::vector<std::string> list;
      list.push_back("a");
      Topic t;
      t.ack = true;
      t.address = 2;
      publishIm(header, list, t);
    }
  }

  // CALLBACKS
  void setpointsCallback(const control::Setpoints::ConstPtr& msg, const Topic& t)
  {
    if (dropTopic(t)) return;
    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id));
    list.push_back(f2s((float)msg->setpoints[0]));
    list.push_back(f2s((float)msg->setpoints[1]));
    list.push_back(f2s((float)msg->setpoints[2]));
    publishIm(msg->header, list, t);
  }

  void emus_bmsCallback(const safety::EMUSBMS::ConstPtr& msg, const Topic& t)
  {
    if (dropTopic(t)) return;
    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id));
    list.push_back(f2s((float)msg->stateOfCharge));
    publishIm(msg->header, list, t);
  }

  void nav_stsCallback(const auv_msgs::NavSts::ConstPtr& msg, const Topic& t)
  {
    if (dropTopic(t)) return;
    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id));
    list.push_back(f2s((float)msg->global_position.latitude));
    list.push_back(f2s((float)msg->global_position.longitude));
    list.push_back(f2s((float)msg->position.north));
    list.push_back(f2s((float)msg->position.east));
    list.push_back(f2s((float)msg->position.depth));
    list.push_back(f2s((float)msg->altitude));
    publishIm(msg->header, list, t);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, const Topic& t)
  {
    if (dropTopic(t)) return;
    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(t.id));
    list.push_back(f2s((float)msg->pose.pose.position.x));
    list.push_back(f2s((float)msg->pose.pose.position.y));
    list.push_back(f2s((float)msg->pose.pose.position.z));
    list.push_back(f2s((float)msg->pose.covariance[0]));
    list.push_back(f2s((float)msg->pose.covariance[7]));
    list.push_back(f2s((float)msg->pose.covariance[14]));
    publishIm(msg->header, list, t);
  }

  bool empty_srvCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, const Service& s)
  {
    std::vector<std::string> list;
    list.push_back(boost::lexical_cast<std::string>(s.id));

    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.ack = s.ack;
    acoustic_msg.address = s.address;
    instant_pub_.publish(acoustic_msg);
  }

  void acousticCallback(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg)
  {
    // Extract message
    std::string payload = acoustic_msg->payload;

    std::vector<std::string> data;
    boost::split(data, payload, boost::is_any_of(";"));

    // Handle empty messages
    if (data.size() < 2) return;

    // Extract topic id
    int id = boost::lexical_cast<int>(data[0]);
    data.erase(data.begin());

    // Extract the name by id
    std::string name;
    IdName::iterator it = id_name_map_.find(id);
    if (it == id_name_map_.end())
    {
      ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to find a topic/service for the id " << id);
      return;
    }
    else
      name = id_name_map_[id];

    // Calls
    std_srvs::Empty srv;
    if (name == "control/thrusters_data")
    {
      control::Setpoints msg;
      setpointsParse(acoustic_msg, data, msg);
      pub_setpoints_.publish(msg);
    }
    if (name == "safety/emus_bms")
    {
      safety::EMUSBMS msg;
      emus_bmsParse(acoustic_msg, data, msg);
      pub_emus_bms_.publish(msg);
    }
    if (name == "navigation/nav_sts")
    {
      auv_msgs::NavSts msg;
      nav_stsParse(acoustic_msg, data, msg);
      pub_nav_sts_.publish(msg);
    }
    if (name == "sensors/modem_delayed")
    {
      geometry_msgs::PoseWithCovarianceStamped msg;
      poseParse(acoustic_msg, data, msg);
      pub_modem_delayed_.publish(msg);
    }

    if (name == "sonar_on") cli_sonar_on_.call(srv);
    if (name == "sonar_off") cli_sonar_off_.call(srv);
    if (name == "start_recording") cli_start_recording_.call(srv);
    if (name == "stop_recording") cli_stop_recording_.call(srv);
    if (name == "control/enable_trajectory") cli_trajectory_on_.call(srv);
    if (name == "control/disable_trajectory") cli_trajectory_off_.call(srv);
    if (name == "control/disable_thrusters") cli_thrusters_off_.call(srv);
    if (name == "lights_on") cli_lights_on_.call(srv);
    if (name == "lights_off") cli_lights_off_.call(srv);
    if (name == "laser_on") cli_laser_on_.call(srv);
    if (name == "laser_off") cli_laser_off_.call(srv);
  }

  void setpointsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg,
                      const std::vector<std::string>& data,
                      control::Setpoints& msg)
  {
    msg.header = acoustic_msg->header;
    msg.setpoints.resize(data.size());
    msg.setpoints[0] = s2f(data[0].c_str());
    msg.setpoints[1] = s2f(data[1].c_str());
    msg.setpoints[2] = s2f(data[2].c_str());
  }

  void emus_bmsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg,
                     const std::vector<std::string>& data,
                     safety::EMUSBMS& msg)
  {
    msg.header = acoustic_msg->header;
    msg.stateOfCharge = s2f(data[0].c_str());
  }

  void nav_stsParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg,
                    const std::vector<std::string>& data,
                    auv_msgs::NavSts& msg)
  {
    msg.header = acoustic_msg->header;
    msg.global_position.latitude = s2f(data[0].c_str());
    msg.global_position.longitude = s2f(data[1].c_str());
    msg.position.north = s2f(data[2].c_str());
    msg.position.east = s2f(data[3].c_str());
    msg.position.depth = s2f(data[4].c_str());
    msg.altitude = s2f(data[5].c_str());
  }

  void poseParse(const evologics_ros::AcousticModemPayload::ConstPtr& acoustic_msg,
                 const std::vector<std::string>& data,
                 geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    msg.header = acoustic_msg->header;
    msg.pose.pose.position.x = s2f(data[0].c_str());
    msg.pose.pose.position.y = s2f(data[1].c_str());
    msg.pose.pose.position.z = s2f(data[2].c_str());
    msg.pose.covariance[0] = s2f(data[3].c_str());
    msg.pose.covariance[7] = s2f(data[4].c_str());
    msg.pose.covariance[13] = s2f(data[5].c_str());
  }

  void required_topics_check()
  {
    // Get node address
    XmlRpc::XmlRpcValue param;
    ros::param::get("~station_address", param);
    station_ = (int)param;

    // Publishers
    if (ros::param::has("~topics")) fillInfo("~topics");
    else ROS_WARN_STREAM("[" << node_name_ << "]: Failed to establish the TOPIC connections dictated by require parameter.");

    // Services
    if (ros::param::has("~services")) fillInfo("~services");
    else ROS_WARN_STREAM("[" << node_name_ << "]: Failed to establish the SERVICE connections dictated by require parameter.");
  }

  void fillInfo(const std::string& param_name)
  {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    int id_counter_topics   = 1;
    int id_counter_services = 50;
    for (XmlRpc::XmlRpcValue::iterator it = param_list.begin(); it != param_list.end(); it++)
    {
      XmlRpc::XmlRpcValue data = it->second;
      ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      if (param_name == "~topics")
      {
        Topic t; // Struct
        t.id = id_counter_topics;
        t.name = (std::string)data["topic_name"];
        t.address = (int)data["destination_address"];
        t.ack = (bool)data["acknowledgement"];
        t.num = topics_.size();
        t.drop = (int)data["drop"];
        t.counter = t.drop;
        topics_.push_back(t);
        setup_topic(t);

        // Update map
        IdName::iterator it = id_name_map_.find(id_counter_topics);
        if (it == id_name_map_.end())
          id_name_map_.insert(IdName::value_type(id_counter_topics, t.name));
        else
          ROS_ERROR_STREAM("[" << node_name_ << "]: Error configuring topic " << t.name);

        // Increase id
        id_counter_topics++;
      }
      else
      {
        Service s; // Struct
        s.id = id_counter_services;
        s.name = (std::string)data["service_name"];
        s.address = (int)data["owner_address"];
        s.ack = false;
        setup_service(s);

        // Update map
        IdName::iterator it = id_name_map_.find(id_counter_services);
        if (it == id_name_map_.end())
          id_name_map_.insert(IdName::value_type(id_counter_services, s.name));
        else
          ROS_ERROR_STREAM("[" << node_name_ << "]: Error configuring service " << s.name);

        // Increase id
        id_counter_services++;
      }
    }
  }

  // EDIT for new topics_
  void setup_topic(const Topic& t)
  {
    if (t.address == station_)
    {
      if (t.name == "control/thrusters_data")
        pub_setpoints_ = n_.advertise<control::Setpoints>(t.name + "_acoustic", 1);
      if (t.name == "safety/emus_bms")
        pub_emus_bms_ = n_.advertise<safety::EMUSBMS>(t.name + "_acoustic", 1);
      if (t.name == "navigation/nav_sts")
        pub_nav_sts_ = n_.advertise<auv_msgs::NavSts>(t.name + "_acoustic", 1);
      if (t.name == "sensors/modem_delayed")
        pub_modem_delayed_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(t.name + "_acoustic", 1);
    }
    else
    {
      if (t.name == "control/thrusters_data")
        sub_setpoints_ = n_.subscribe<control::Setpoints>(t.name, 1, boost::bind(&Session::setpointsCallback, this, _1, t));
      if (t.name == "safety/emus_bms")
        sub_emus_bms_ = n_.subscribe<safety::EMUSBMS>(t.name, 1, boost::bind(&Session::emus_bmsCallback, this, _1, t));
      if (t.name == "navigation/nav_sts")
        sub_nav_sts_ = n_.subscribe<auv_msgs::NavSts>(t.name, 1, boost::bind(&Session::nav_stsCallback, this, _1, t));
      if (t.name == "sensors/modem_delayed")
        sub_modem_delayed_ = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(t.name, 1, boost::bind(&Session::poseCallback, this, _1, t));
    }
  }

  void setup_service(const Service& s)
  {
    // All services are Empty
    if (s.address == station_)
    {
      if (s.name == "sonar_on")
        srv_sonar_on_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "sonar_off")
        srv_sonar_off_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "start_recording")
        srv_start_recording_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "stop_recording")
        srv_stop_recording_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "control/enable_trajectory")
        srv_trajectory_on_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "control/disable_trajectory")
        srv_trajectory_off_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "control/disable_thrusters")
        srv_thrusters_off_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "lights_on")
        srv_lights_on_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "lights_off")
        srv_lights_off_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "laser_on")
        srv_laser_on_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
      if (s.name == "laser_off")
        srv_laser_off_ = n_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(s.name + "_acoustic",  boost::bind(&Session::empty_srvCallback, this, _1, _2, s));
    }
    else
    {
      if (s.name == "sonar_on") cli_sonar_on_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "sonar_off") cli_sonar_off_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "start_recording") cli_start_recording_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "stop_recording") cli_stop_recording_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "control/enable_trajectory") cli_trajectory_on_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "control/disable_trajectory") cli_trajectory_off_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "control/disable_thrusters") cli_thrusters_off_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "lights_on") cli_lights_on_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "lights_off") cli_lights_off_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "laser_on") cli_laser_on_ = n_.serviceClient<std_srvs::Empty>(s.name);
      if (s.name == "laser_off") cli_laser_off_ = n_.serviceClient<std_srvs::Empty>(s.name);
    }

  }

  std::string f2s(float p)
  {
    unsigned char* pc = reinterpret_cast<unsigned char*>(&p);
    return std::string(pc, pc + sizeof(float));
  }

  float s2f(const std::string& s)
  {
    const unsigned char* pt = reinterpret_cast<const unsigned char*>(s.c_str());
    const float* d2 = reinterpret_cast<const float*>(pt);
    return *const_cast<float*>(d2);
  }

private:

  typedef std::map<int, std::string> IdName;

  // Operational variables
  std::string node_name_;
  ros::NodeHandle n_;
  int station_;
  std::vector<Topic> topics_;

  // Ping timer
  ros::Time send_time_;
  ros::Timer timer_;

  // Im in/out
  ros::Subscriber instant_sub_;
  ros::Publisher instant_pub_;

  // Subscribers
  ros::Subscriber sub_setpoints_;
  ros::Subscriber sub_emus_bms_;
  ros::Subscriber sub_nav_sts_;
  ros::Subscriber sub_modem_delayed_;

  // Publishers
  ros::Publisher  pub_setpoints_;
  ros::Publisher  pub_emus_bms_;
  ros::Publisher  pub_nav_sts_;
  ros::Publisher  pub_modem_delayed_;

  // Service servers
  ros::ServiceServer srv_sonar_on_;
  ros::ServiceServer srv_sonar_off_;
  ros::ServiceServer srv_start_recording_;
  ros::ServiceServer srv_stop_recording_;
  ros::ServiceServer srv_trajectory_on_;
  ros::ServiceServer srv_trajectory_off_;
  ros::ServiceServer srv_thrusters_off_;
  ros::ServiceServer srv_lights_on_;
  ros::ServiceServer srv_lights_off_;
  ros::ServiceServer srv_laser_on_;
  ros::ServiceServer srv_laser_off_;

  // Service clients
  ros::ServiceClient cli_sonar_on_;
  ros::ServiceClient cli_sonar_off_;
  ros::ServiceClient cli_start_recording_;
  ros::ServiceClient cli_stop_recording_;
  ros::ServiceClient cli_trajectory_on_;
  ros::ServiceClient cli_trajectory_off_;
  ros::ServiceClient cli_thrusters_off_;
  ros::ServiceClient cli_lights_on_;
  ros::ServiceClient cli_lights_off_;
  ros::ServiceClient cli_laser_on_;
  ros::ServiceClient cli_laser_off_;

  // Correspondence between ids and names
  IdName id_name_map_;
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
