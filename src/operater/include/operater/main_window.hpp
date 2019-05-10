/**
 * @file /include/operater/main_window.hpp
 *
 * @brief Qt based gui for operater.
 *
 * @date November 2010
 **/
#ifndef operater_MAIN_WINDOW_H
#define operater_MAIN_WINDOW_H



#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


namespace operater {


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:

    void updateLoggingView(); // no idea why this can't connect automatically

    void on_pushButton_startRecordPathMsg_clicked(bool checked);

    void showMessage_in_statusBar(QString qstr,int time_out);

    void on_pushButton_setRoadWidth_clicked();

    void on_pushButton_startNode_clicked(bool checked);

     void on_pushButton_recordPath_clicked();

private:
    void showRecordRoadCurrentSettings();
    bool is_OffsetValid();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  little_ant_msgs::PathInfo path_info;

Q_SIGNALS:
    void publishPathInfo_signal();

public Q_SLOTS:
    void on_pushButton_home_clicked();

    void on_pushButton_startDriverless_clicked();

    void on_pushButton_toDriverlessPage_clicked();

    void on_pushButton_homeP2_clicked();

    void on_comboBox_trfficSigns_currentIndexChanged(int index);
};

}  // namespace operater

#endif // operater_MAIN_WINDOW_H
