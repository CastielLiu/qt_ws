/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/operater/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace operater {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode()
{
  path_info.maxOffset_left = 0.0;
  path_info.maxOffset_right =0.0;
  path_info.other_info = 0;
  path_info.traffic_sign = 0;
}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(argc_,argv_,"operater");
  if ( ! ros::master::check() )
  {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;
	// Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  sub_debug = nh.subscribe("/debug",10,&QNode::debug_callback,this);
  pub_pathInfo = nh.advertise<little_ant_msgs::PathInfo>("/path_info",1);

  spin_thread_ptr = boost::shared_ptr<boost::thread >
      (new boost::thread(boost::bind(&QNode::spinThread, this)));

	start();
	return true;
}

void QNode::publishPathInfo_slot()
{
  pub_pathInfo.publish(path_info);
}

void QNode::startNode()
{
  boost::thread _thread(boost::bind(&QNode::startNode_thread, this));
}

void QNode::startNode_thread()
{
  if(!ros::master::check())
    system("roscore");
  sleep(5);
  while(!ros::master::check())
    usleep(100000);
  this->init();
}


void QNode::debug_callback(const state_detection::Debug::ConstPtr& msg)
{
  log(LogLevel(msg->level),msg->info);
}

void QNode::run()
{
  while ( ros::ok() )
  {
    usleep(100000);

	}
  //std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG]:" << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
        logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);


  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace operater
