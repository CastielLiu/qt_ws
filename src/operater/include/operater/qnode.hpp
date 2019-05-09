/**
 * @file /include/operater/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef operater_QNODE_HPP_
#define operater_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <state_detection/Debug.h>
#include<little_ant_msgs/PathInfo.h>
#include<boost/thread/thread.hpp>
#include<boost/bind/bind.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace operater {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
  QNode();
	virtual ~QNode();
  bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

  void debug_callback(const state_detection::Debug::ConstPtr& msg);
  void spinThread() {ros::spin();}
  void setPathInfo(const little_ant_msgs::PathInfo& info){path_info = info;}
  void startNode_thread();
  void setRosInitArg(int argc,char** argv){argc_=argc; argv_=argv;}
  void startNode();
  void stopNode(){system("rosnode kill operater");}
  bool checkMaster(){return ros::master::check();}

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

public Q_SLOTS:
  void publishPathInfo_slot();


private:
  ros::Subscriber sub_debug;
  ros::Publisher pub_pathInfo;

  little_ant_msgs::PathInfo path_info;

  QStringListModel logging_model;
  boost::shared_ptr<boost::thread> spin_thread_ptr;
  int argc_;
  char** argv_;
};

}  // namespace operater

#endif /* operater_QNODE_HPP_ */
