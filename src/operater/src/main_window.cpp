/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/operater/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace operater {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
{
    ui.setupUi(this);

    this->setMaximumSize(497,320);
    this->setMinimumSize(497,320);

    qnode.setRosInitArg(argc,argv);

    //ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));

    //QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this,SIGNAL(publishPathInfo_signal()),&qnode,SLOT(publishPathInfo_slot()));

    ui.listView_systemLog->setModel(qnode.loggingModel());

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.listView_systemLog->setEditTriggers(QAbstractItemView::NoEditTriggers);

    ui.listView_systemLog->setAlternatingRowColors(true);
    ui.plainTextEdit_recordRoadCurrentSettings->setReadOnly(true);
    ui.pushButton_recordPath->setDisabled(true);
    ui.pushButton_toDriverlessPage->setEnabled(false);

    path_info.maxOffset_left = 0.0;
    path_info.maxOffset_right = 0.0;
    path_info.traffic_sign = little_ant_msgs::PathInfo::_NONE;
}

MainWindow::~MainWindow() {}


void MainWindow::updateLoggingView()
{
  /*QPalette p = ui.listView_systemLog->palette();
  p.setColor(QPalette::Active,QPalette::Text,Qt::red);
  p.setColor(QPalette::Inactive,QPalette::Text,Qt::red);

  ui.listView_systemLog->setPalette(p);*/

  ui.listView_systemLog->scrollToBottom();
}


void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "operater");
    /*
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }*/
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "operater");
    /*
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
  */
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace operater

void operater::MainWindow::showMessage_in_statusBar(QString qstr,int time_out)
{
    ui.statusBar->showMessage(qstr,time_out);
}

void operater::MainWindow::on_pushButton_startRecordPathMsg_clicked(bool checked)
{
  if(ui.pushButton_startRecordPathMsg->text() == "Stop")
  {
    system("rosnode kill /record_gps_data");
    system("rosnode kill /novatel_node");
    ui.pushButton_startRecordPathMsg->setText("Start");
    ui.lineEdit_pathMSgFileName->setDisabled(false);
    return;
  }

  if(!is_OffsetValid())
    return;

  std::string leftOffset_str = ui.lineEdit_maxOffset_left->text().toStdString();
  if(leftOffset_str.substr(0,1) != "-")
    leftOffset_str = std::string("-") + leftOffset_str;
  ui.lineEdit_maxOffset_left->setText(QString::fromStdString(leftOffset_str));


  std::string file_name = ui.lineEdit_pathMSgFileName->text().toStdString();
  if(file_name.empty())
  {
    ui.statusBar->showMessage("error: please input the file name",3000);
    return;
  }

  size_t index;

  index = file_name.find_first_of(".");
  if(index == std::string::npos)
  {
    ui.statusBar->showMessage("error: please input the file externsion .txt",3000);
    return;
  }

  file_name = file_name.substr(0,index) + ".txt";

  index = file_name.find_first_not_of(" ");
  if(index != 0)
    file_name = file_name.substr(index);

  //printf("1:%s\n",file_name.c_str());

  ui.lineEdit_pathMSgFileName->setText(QString::fromStdString(file_name));

  std::stringstream cmd_ss;
  cmd_ss << "gnome-terminal -x roslaunch little_ant record_gps_data.launch ";
  cmd_ss << "file_name:=" << file_name <<" ";
  cmd_ss << "initial_maxOffset_left:=" << ui.lineEdit_maxOffset_left->text().toStdString()<<" ";
  cmd_ss << "initial_maxOffset_right:=" << ui.lineEdit_maxOffset_right->text().toStdString()<<" ";
  cmd_ss << "traffic_sign:=" << ui.comboBox_trfficSigns->currentIndex();

  system(cmd_ss.str().c_str());

  ui.pushButton_startRecordPathMsg->setText("Stop");
  ui.lineEdit_pathMSgFileName->setDisabled(true);
}

void operater::MainWindow::showRecordRoadCurrentSettings()
{
  ui.plainTextEdit_recordRoadCurrentSettings->clear();
  ui.plainTextEdit_recordRoadCurrentSettings->appendPlainText("file name: "+
                                                              ui.lineEdit_pathMSgFileName->text());
  std::stringstream ss("");
  ss << "left   maximun offset: " << path_info.maxOffset_left;
  ui.plainTextEdit_recordRoadCurrentSettings->appendPlainText(QString::fromStdString(ss.str()));

  ss.str("");

  ss << "right  maximun offset: " << path_info.maxOffset_right;
  ui.plainTextEdit_recordRoadCurrentSettings->appendPlainText(QString::fromStdString(ss.str()));

  ui.plainTextEdit_recordRoadCurrentSettings->appendPlainText("traffic sign: "+
                                                         ui.comboBox_trfficSigns->currentText());
}

void operater::MainWindow::on_pushButton_setRoadWidth_clicked()
{
  if(!is_OffsetValid())
    return;

  std::string leftOffset_str = ui.lineEdit_maxOffset_left->text().toStdString();
  if(leftOffset_str.substr(0,1) != "-")
    leftOffset_str = std::string("-") + leftOffset_str;
  ui.lineEdit_maxOffset_left->setText(QString::fromStdString(leftOffset_str));


  float left = ui.lineEdit_maxOffset_left->text().toFloat();
  float right = ui.lineEdit_maxOffset_right->text().toFloat();

  QMessageBox msgBox(this);
  msgBox.setText("left: "+ui.lineEdit_maxOffset_left->text() + "\tright: "+ui.lineEdit_maxOffset_right->text());
  msgBox.setInformativeText("Confirm the settings?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::Yes);
  if(msgBox.exec() == QMessageBox::No)
    return ;

  path_info.maxOffset_left = left;
  path_info.maxOffset_right = right;

  qnode.setPathInfo(path_info);

  Q_EMIT publishPathInfo_signal();
  showRecordRoadCurrentSettings();

   //printf("%f\t%f\n",left,right);
}

bool operater::MainWindow::is_OffsetValid()
{
  float left = ui.lineEdit_maxOffset_left->text().toFloat();
  float right = ui.lineEdit_maxOffset_right->text().toFloat();

  if(ui.lineEdit_maxOffset_left->text().isEmpty() ||
     ui.lineEdit_maxOffset_right->text().isEmpty())
  {
    ui.statusBar->showMessage("error: input empty!!!",3000);
  }
  else if((ui.lineEdit_maxOffset_left->text() != "0" && left == 0) ||
    (ui.lineEdit_maxOffset_right->text() != "0" && right == 0))
  {
    ui.statusBar->showMessage("error: input error!!!");
  }
  else if(right < 0.0)
  {
    ui.statusBar->showMessage("error: maxOffset_right Must be greater than 0 !!");
  }
  else
    return true;
  return false;
}

void operater::MainWindow::on_pushButton_startNode_clicked(bool checked)
{
  if(checked)
  {
    if(!qnode.init())
    {
      ui.statusBar->showMessage("error: please run roscore firstly!!!",3000);
       ui.pushButton_startNode->setChecked(false);
       return;
    }
    ui.pushButton_startNode->setText("Stop Node");
    ui.statusBar->showMessage("operator node started.",3000);
    ui.pushButton_recordPath->setEnabled(true);
    ui.pushButton_toDriverlessPage->setEnabled(true);
    return;
  }
  QMessageBox msgBox(this);
  msgBox.setText("Closing Operating Node.");
  msgBox.setInformativeText("Are you sure?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::Yes);
  if(msgBox.exec() == QMessageBox::No)
  {
    ui.pushButton_startNode->setChecked(true);
    return ;
  }
  qnode.stopNode();
  ui.pushButton_startNode->setText("Start Node");

  ui.pushButton_recordPath->setDisabled(true);
  ui.pushButton_toDriverlessPage->setEnabled(false);
}

void operater::MainWindow::on_pushButton_recordPath_clicked()
{
  QString text = QInputDialog::getText (this,"Get Permission","please input operation password",
                  QLineEdit::Password);

  if(text == "seu666")
    ui.stackedWidget->setCurrentIndex(1);
  else
    return;
}

void operater::MainWindow::on_pushButton_home_clicked()
{
    ui.stackedWidget->setCurrentIndex(0);
}

void operater::MainWindow::on_pushButton_startDriverless_clicked()
{
    //start all node about driverless
  std::string prefix = "gnome-terminal ";
  std::string driverless_cmd = "roslaunch little_ant driverless ";
  std::string lidar_cmd = "roslaunch hesai p40p.launch ";
  std::string cluster_cmd = "roslaunch euclidean_cluster  euclidean_cluster .launch ";

  system((prefix + " -tab -e \"" + lidar_cmd +"\" -tab -e \"" + cluster_cmd +"\"").c_str());

}

void operater::MainWindow::on_pushButton_toDriverlessPage_clicked()
{
    ui.stackedWidget->setCurrentIndex(2);
}

void operater::MainWindow::on_pushButton_homeP2_clicked()
{
     ui.stackedWidget->setCurrentIndex(0);
}

void operater::MainWindow::on_comboBox_trfficSigns_currentIndexChanged(int index)
{
  static int lastIndex = 0;

  if(index == lastIndex) return;

  QMessageBox msgBox(this);
  msgBox.setText("modify traffic sign :");
  msgBox.setInformativeText(ui.comboBox_trfficSigns->currentText() + "?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::Yes);
  if(msgBox.exec() == QMessageBox::No)
  {
    ui.comboBox_trfficSigns->setCurrentIndex(lastIndex);
    return ;
  }

  path_info.traffic_sign = index;

  qnode.setPathInfo(path_info);

  Q_EMIT publishPathInfo_signal();
  showRecordRoadCurrentSettings();
  lastIndex = index;
}
