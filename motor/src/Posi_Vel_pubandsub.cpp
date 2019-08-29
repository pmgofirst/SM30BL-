/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include <geometry_msgs/Point32.h>

// %EndTag(MSG_HEADER)%

#include <sstream>

float steering_info[3]={0.0,0.0,0.0};
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
void paveInfoCallback(const geometry_msgs::Point32& steerinfo)
  {steering_info[0] = steerinfo.x;
  steering_info[1] =steerinfo.y;
  ROS_INFO("steering_position:%f  velocity:%f",steering_info[0],steering_info[1]);
  }
int main(int argc, char **argv)
{

// %Tag(INIT)%
  ros::init(argc, argv, "Posi_Vel_pubandsub");// the name of node
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)%
  ros::Publisher commands_pub = n.advertise<geometry_msgs::Point32>("gimbal_commands", 100);
  ros::Subscriber po_ve_sub = n.subscribe("/place_velocity_info", 1, paveInfoCallback);
  // chatter(topic name) Type std_msgs::String
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    geometry_msgs::Point32 msg;
    msg.x=0.0;
    msg.y=30.0;
    msg.z=0.0;

//    std::stringstream ss;
//   ss << "hello world " << count;
    //msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("publishertest%f_%f_%f", msg.x,msg.y,msg.z);
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    commands_pub.publish(msg);
// %EndTag(PUBLISH)%
    ros::Duration(12.0).sleep();
    ROS_INFO("steering_position:%f  velocity:%f",steering_info[0],steering_info[1]);
    msg.x=9.42;
    msg.y=70.0;
    msg.z=0.0;
    ROS_INFO("publishertest%f_%f_%f", msg.x,msg.y,msg.z);
    commands_pub.publish(msg);
    ros::Duration(8.0).sleep();
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  //  ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
