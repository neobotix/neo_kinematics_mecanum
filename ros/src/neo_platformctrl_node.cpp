/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <boost/thread.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include "../../common/include/neo_PlatformCtrl/Kinematics.h"
#include "../../common/include/neo_PlatformCtrl/MecanumKinematics.h"
#include <tf/transform_broadcaster.h>

class PlatformCtrlNode 
{
	public:
	PlatformCtrlNode();
	virtual ~PlatformCtrlNode();
	ros::NodeHandle n;
	ros::Publisher topicPub_Odometry;	
	ros::Subscriber topicSub_DriveState;
	ros::Publisher topicPub_DriveCommands;	
	ros::Subscriber topicSub_ComVel;
	tf::TransformBroadcaster odom_broadcaster;

	int init();
	void receiveCmd(const geometry_msgs::Twist& twist);
	void sendOdom(const sensor_msgs::JointState& js);
	private:
	boost::mutex mutex;
	OdomPose p;
	Kinematics* kin;
	bool sendTransform;
};

PlatformCtrlNode::PlatformCtrlNode()
{
	kin = NULL;
}

PlatformCtrlNode::~PlatformCtrlNode()
{
       if( kin != NULL ) delete kin;
}

int PlatformCtrlNode::init()
{

	double devX, devY, devZ, devRoll, devPitch, devYaw;

	n.param<bool>("sendTransform",sendTransform,false);
	if(sendTransform)
	{
        ROS_INFO("neo_kinematics_mecanum_node: sending transformation");
	} else {

        ROS_INFO("neo_kinematics_mecanum_node: sending no transformation");
	}

	double wheelDiameter, axisLength, axisWidth;
	Mecanum4WKinematics* mecKin = new Mecanum4WKinematics;
	kin = mecKin;
	if(n.hasParam("wheelDiameter"))
	{
		n.getParam("wheelDiameter",wheelDiameter);
		mecKin->setWheelDiameter(wheelDiameter);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter wheelDiameter not set, using default 0.3");
		mecKin->setWheelDiameter(0.3);
	}
	if(n.hasParam("robotWidth"))
	{
		n.getParam("robotWidth",axisWidth);
		mecKin->setAxis1Length(axisWidth);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter robotWidth not set, using default 0.5");
		mecKin->setAxis1Length(0.5);
	}
	if(n.hasParam("robotLength"))
	{
		n.getParam("robotLength",axisLength);
		mecKin->setAxis2Length(axisLength);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter robotLength not set, using default 0.5");
		mecKin->setAxis2Length(0.5);
	}
	////////////////////////////////////
	if(n.hasParam("devX"))
	{
		n.getParam("devX",devX);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter devX not set, using default 0.1");
		devX = 0.1;
	}
	if(n.hasParam("devY"))
	{
		n.getParam("devY",devY);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter devY not set, using default 0.1");
		devY = 0.1;
	}
	if(n.hasParam("devZ"))
	{
		n.getParam("devZ",devZ);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter devZ not set, using default 0.1");
		devZ = 0.1;
	}
	///////////////////////////////////////
	if(n.hasParam("devRoll"))
	{
		n.getParam("devRoll",devRoll);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter devRoll not set, using default 0.1");
		devRoll = 0.1;
	}
	if(n.hasParam("devPitch"))
	{
		n.getParam("devPitch",devPitch);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter devPitch not set, using default 0.1");
		devPitch = 0.1;
	}
	if(n.hasParam("devYaw"))
	{
		n.getParam("devYaw",devYaw);
	}
	else 
	{
		ROS_WARN("neo_kinematics_mecanum_node: Parameter devYaw not set, using default 0.1");
		devYaw = 0.1;
	}

	mecKin->setStdDev(devX, devY, devZ, devRoll, devPitch, devYaw);
	
	if(kin == NULL) return 1;
	p.xAbs = 0; p.yAbs = 0; p.phiAbs = 0;
	topicPub_Odometry = n.advertise<nav_msgs::Odometry>("/odom",1);	
    topicSub_DriveState = n.subscribe("/drives/joint_states",1,&PlatformCtrlNode::sendOdom, this);
    topicPub_DriveCommands = n.advertise<trajectory_msgs::JointTrajectory>("/drives/joint_trajectory",1);
	topicSub_ComVel = n.subscribe("/cmd_vel",1,&PlatformCtrlNode::receiveCmd, this);
	return 0;
}



void PlatformCtrlNode::receiveCmd(const geometry_msgs::Twist& twist)
{
   mutex.lock();
	trajectory_msgs::JointTrajectory traj;
	kin->execInvKin(twist,traj);
	topicPub_DriveCommands.publish(traj);
   mutex.unlock();
}

void PlatformCtrlNode::sendOdom(const sensor_msgs::JointState& js)
{

   mutex.lock();
    //check if js has data from 4 motors
	if(sizeof(js.velocity) < 4)
		return;
	//odometry msg
	nav_msgs::Odometry odom;
	odom.header.stamp = js.header.stamp;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	kin->execForwKin(js, odom, p);
	topicPub_Odometry.publish(odom);
	//odometry transform:
	if(sendTransform)
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = odom.header.stamp;
		odom_trans.header.frame_id = odom.header.frame_id;
		odom_trans.child_frame_id = odom.child_frame_id;
		odom_trans.transform.translation.x = odom.pose.pose.position.x;
		odom_trans.transform.translation.y = odom.pose.pose.position.y;
		odom_trans.transform.translation.z = odom.pose.pose.position.z;
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		odom_broadcaster.sendTransform(odom_trans);
	}
   mutex.unlock();
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "neo_kinematics_mecanum_node");
	PlatformCtrlNode node;
    if(node.init() != 0) ROS_ERROR("neo_kinematics_mecanum_node: init failed!");
	ros::spin();
	return 0;
}

