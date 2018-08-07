#ifndef ROSBRIDGE_H
#define ROSBRIDGE_H

#include <QtCore/QThread>
#include "datastruct.h"
#include <ros/ros.h>

class rosBridge : public QThread
{
	Q_OBJECT

private:
	ros::Publisher _sensor_data_pub;
	ros::Publisher _glove_data_pub;

public:
	explicit rosBridge(int argc, char **argv, QObject *parent = nullptr);
	~rosBridge();

private:
	void run() override;

public slots:
	void sensorData_received(SensorData sd);
	void gloveData_received(GloveData gd);

signals:
	void open();
	void close();
};

#endif // ROSBRIDGE_H
