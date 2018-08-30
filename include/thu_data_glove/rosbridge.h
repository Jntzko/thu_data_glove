#ifndef ROSBRIDGE_H
#define ROSBRIDGE_H

#include <QtCore/QThread>
#include <thu_data_glove/datastruct.h>
#include <ros/ros.h>
#include <tf/tf.h>

class rosBridge : public QThread
{
	Q_OBJECT

private:
  ros::Publisher pub_;
  bool bias_ = false;

public:
	explicit rosBridge(int argc, char **argv, QObject *parent = nullptr);
	~rosBridge();

private:
	void run() override;

public slots:
	void sensorData_received(SensorData sd);

signals:
	void open();
	void close();
};

#endif // ROSBRIDGE_H
