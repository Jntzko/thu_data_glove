#include "rosbridge.h"
#include <glove/SensorData.h>
#include <glove/GloveData.h>
#include <signal.h>
#include <QtCore/QDebug>
#include <QtCore/QCoreApplication>

void ros_quit(int)
{
	ros::shutdown();
	qDebug() << "Node Killed";
	QCoreApplication::quit();
}

rosBridge::rosBridge(int argc, char ** argv, QObject *parent) : QThread(parent)
{
	ros::init(argc, argv, "WiFiGlove");
	this->start();
}

rosBridge::~rosBridge()
{
	this->terminate();
}

void rosBridge::run()
{
	ros::NodeHandle _nh;
	signal(SIGINT, ros_quit);
	this->_sensor_data_pub = _nh.advertise<glove::SensorData>("sensor_raw", 2);
	this->_glove_data_pub = _nh.advertise<glove::GloveData>("glove_joint", 2);
	ros::spin();
}

void rosBridge::sensorData_received(SensorData sd)
{
	glove::SensorData data;
	data.gloveID = sd.gloveID;
	data.timestemp = sd.timestemp;
	for (int i = 0 ; i < 18 ; i++){
		data.sensor_data[i].sensor_id = sd.sensor_data[i].sensor_id;
		data.sensor_data[i].acceleration.x = sd.sensor_data[i].Ax;
		data.sensor_data[i].acceleration.y = sd.sensor_data[i].Ay;
		data.sensor_data[i].acceleration.z = sd.sensor_data[i].Az;

		data.sensor_data[i].angular.x = sd.sensor_data[i].Wx;
		data.sensor_data[i].angular.y = sd.sensor_data[i].Wy;
		data.sensor_data[i].angular.z = sd.sensor_data[i].Wz;

		data.sensor_data[i].magnetic.x = sd.sensor_data[i].Mx;
		data.sensor_data[i].magnetic.y = sd.sensor_data[i].My;
		data.sensor_data[i].magnetic.z = sd.sensor_data[i].Mz;

		data.sensor_data[i].rpy.x = sd.sensor_data[i].roll;
		data.sensor_data[i].rpy.y = sd.sensor_data[i].pitch;
		data.sensor_data[i].rpy.z = sd.sensor_data[i].yaw;
	}
	this->_sensor_data_pub.publish(data);
}

void rosBridge::gloveData_received(GloveData gd)
{
	glove::GloveData data;
	data.gloveID = gd.gloveID;
	data.arm0 = gd.arm.a0;
	data.arm1_0 = gd.arm.a10;
	data.arm1_1 = gd.arm.a11;

	data.thumb0_0 = gd.thumb.t00;
	data.thumb0_1 = gd.thumb.t01;
	data.thumb1 = gd.thumb.t1;
	data.thumb2 = gd.thumb.t2;

	data.forefinger0_0 = gd.forefinger.f00;
	data.forefinger0_1 = gd.forefinger.f01;
	data.forefinger1 = gd.forefinger.f1;
	data.forefinger2 = gd.forefinger.f2;

	data.middlefinger0_0 = gd.middlefinger.m00;
	data.middlefinger0_1 = gd.middlefinger.m01;
	data.middlefinger1 = gd.middlefinger.m1;
	data.middlefinger2 = gd.middlefinger.m2;

	data.ringfinger0_0 = gd.ringfinger.r00;
	data.ringfinger0_1 = gd.ringfinger.r01;
	data.ringfinger1 = gd.ringfinger.r1;
	data.ringfinger2 = gd.ringfinger.r2;

	data.littlefinger0_0 = gd.littlefinger.l00;
	data.littlefinger0_1 = gd.littlefinger.l01;
	data.littlefinger1 = gd.littlefinger.l1;
	data.littlefinger2 = gd.littlefinger.l2;

	data.base.w = gd.base.w;
	data.base.x = gd.base.x;
	data.base.y = gd.base.y;
	data.base.z = gd.base.z;

	this->_glove_data_pub.publish(data);
}
