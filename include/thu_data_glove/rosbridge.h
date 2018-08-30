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
  //soon deprecated
	ros::Publisher _glove_data_pub;
  bool bias_ = false;
  std::vector<tf::Quaternion> bias_values_;
  std::vector<tf::Quaternion> prev_values_;

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
