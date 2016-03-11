/**
 *
 *  \file
 *  \brief      Classes which manage the Publish and Subscribe relationships
 *              that a Session has with its client.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_TOPIC_HANDLERS_H
#define ROSSERIAL_SERVER_TOPIC_HANDLERS_H

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <rosserial_msgs/RequestMessageInfo.h>
#include <rosserial_msgs/RequestServiceInfo.h>
#include <topic_tools/shape_shifter.h>
#include <std_srvs/Empty.h>

namespace rosserial_server
{

class Publisher {
public:
  Publisher(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info) {
    if (!message_service_.isValid()) {
      // lazy-initialize the service caller.
      message_service_ = nh.serviceClient<rosserial_msgs::RequestMessageInfo>("message_info");
      if (!message_service_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("Timed out waiting for message_info service to become available.");
      }
    }

    rosserial_msgs::RequestMessageInfo info;
    info.request.type = topic_info.message_type;
    if (message_service_.call(info)) {
      topic_info.md5sum = info.response.md5;
      message_.morph(topic_info.md5sum, topic_info.message_type, info.response.definition, "false");
      publisher_ = message_.advertise(nh, topic_info.topic_name, 1);
    } else {
      ROS_WARN("Failed to call message_info service. Proceeding without full message definition.");
    }

    message_.morph(topic_info.md5sum, topic_info.message_type, info.response.definition, "false");
    publisher_ = message_.advertise(nh, topic_info.topic_name, 1);
  }

  void handle(ros::serialization::IStream stream) {
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, message_);
    publisher_.publish(message_);
  }

  std::string get_topic() {
    return publisher_.getTopic();
  }

private:
  ros::Publisher publisher_;
  topic_tools::ShapeShifter message_;

  static ros::ServiceClient message_service_;
};

ros::ServiceClient Publisher::message_service_;
typedef boost::shared_ptr<Publisher> PublisherPtr;


class Subscriber {
public:
  Subscriber(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t> buffer)> write_fn)
    : counter_(0), write_fn_(write_fn) {
    if (!sub_message_service_.isValid()) {
      // lazy-initialize the service caller.
      sub_message_service_ = nh.serviceClient<rosserial_msgs::RequestMessageInfo>("message_info");
      if (!sub_message_service_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("Timed out waiting for message_info service to become available.");
      }
    }

    drop_ = topic_info.drop;

    rosserial_msgs::RequestMessageInfo info;
    info.request.type = topic_info.message_type;
    if (sub_message_service_.call(info)) {
      topic_info.md5sum = info.response.md5;
      ros::SubscribeOptions opts;
      opts.init<topic_tools::ShapeShifter>(
          topic_info.topic_name, 1, boost::bind(&Subscriber::handle, this, _1));
      opts.md5sum = topic_info.md5sum;
      opts.datatype = topic_info.message_type;
      subscriber_ = nh.subscribe(opts);
    } else {
      ROS_ERROR("Subscriber: failed to call message_info service.");
    }
  }

  std::string get_topic() {
    return subscriber_.getTopic();
  }

private:
  void handle(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
    counter_++;
    if (counter_ == drop_){
      size_t length = ros::serialization::serializationLength(*msg);
      std::vector<uint8_t> buffer(length);

      ros::serialization::OStream ostream(&buffer[0], length);
      ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

      write_fn_(buffer);
      counter_ = 0;
    }
  }

  int drop_;
  int counter_;
  ros::Subscriber subscriber_;
  static ros::ServiceClient sub_message_service_;
  boost::function<void(std::vector<uint8_t> buffer)> write_fn_;
};

ros::ServiceClient Subscriber::sub_message_service_;
typedef boost::shared_ptr<Subscriber> SubscriberPtr;

class ServiceServer {
public:
  ServiceServer(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t> buffer)> write_fn)
    : write_fn_(write_fn) {
    if (!service_info_service_.isValid()) {
      // lazy-initialize the service caller.
      service_info_service_ = nh.serviceClient<rosserial_msgs::RequestServiceInfo>("service_info");
      if (!service_info_service_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("Timed out waiting for service_info service to become available.");
      }
    }

    rosserial_msgs::RequestServiceInfo info;
    info.request.service = topic_info.message_type;
    ROS_DEBUG("Calling service_info service for service name %s",topic_info.topic_name.c_str());
    if (service_info_service_.call(info)) {
      ros::AdvertiseServiceOptions opts;
      opts.init<std_srvs::Empty>(
          topic_info.topic_name, boost::bind(&ServiceServer::handle, this, _1, _2));
      opts.md5sum = info.response.service_md5;
      opts.req_datatype = topic_info.message_type;
      opts.res_datatype = topic_info.message_type;
      opts.datatype = topic_info.message_type;
      service_server_ = nh.advertiseService(opts);
    } else {
      ROS_ERROR("ServiceServer: Failed to call service_info service.");
    }
  }

  bool handle(const std_srvs::Empty::Request request, const std_srvs::Empty::Response response) {
    ROS_INFO("Publishing service from SERVICE SERVER");
    // Send no information
    std::vector<uint8_t> buffer(0);
    write_fn_(buffer);
    return true;
  }

private:
  ros::ServiceServer service_server_;
  static ros::ServiceClient service_info_service_;
  boost::function<void(std::vector<uint8_t> buffer)> write_fn_;
};

ros::ServiceClient ServiceServer::service_info_service_;
typedef boost::shared_ptr<ServiceServer> ServiceServerPtr;

class ServiceClient {
public:
  ServiceClient(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info)
    {
    if (!service_info_service_.isValid()) {
      // lazy-initialize the service caller.
      service_info_service_ = nh.serviceClient<rosserial_msgs::RequestServiceInfo>("service_info");
      if (!service_info_service_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("Timed out waiting for service_info service to become available.");
      }
    }

    rosserial_msgs::RequestServiceInfo info;
    info.request.service = topic_info.message_type;
    ROS_DEBUG("Calling service_info service for topic name %s",topic_info.topic_name.c_str());
    if (service_info_service_.call(info)) {
      ros::ServiceClientOptions opts;
      opts.service = topic_info.topic_name;
      opts.md5sum = info.response.service_md5;
      opts.persistent = false; // always false for now
      service_client_ = nh.serviceClient(opts);
    } else {
      ROS_ERROR("ServiceClient: Failed to call service_info service.");
    }
  }

  void handle(ros::serialization::IStream stream) {
    ROS_INFO("Recived subscribed service act as SERVICE CLIENT");
    std_srvs::Empty::Request request;
    std_srvs::Empty::Response response;
    service_client_.call(request, response);
  }

private:
  ros::ServiceClient service_client_;
  static ros::ServiceClient service_info_service_;
};

ros::ServiceClient ServiceClient::service_info_service_;
typedef boost::shared_ptr<ServiceClient> ServiceClientPtr;

}  // namespace

#endif  // ROSSERIAL_SERVER_TOPIC_HANDLERS_H
