#include <thu_data_glove/rosbridge.h>
#include <thu_data_glove/SensorData.h>
#include <thu_data_glove/GloveData.h>
#include <signal.h>
#include <QtCore/QDebug>
#include <QtCore/QCoreApplication>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#define NEW_LH 1192974848
#define NEW_RH 1193564786
#define OLD_LH 1192778367
#define OLD_RH 1192974877

static std::vector<std::string> frames{ "lf_distal", "lf_middle", "lf_knuckle", "rf_distal", "rf_middle", "rf_knuckle", "mf_distal", "mf_middle", "mf_knuckle", "ff_distal", "ff_middle", "ff_knuckle", "th_distal", "th_middle", "th_knuckle", "palm", "arm", "wrist"};

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

  for (int i=0; i<18; i++) {
	  this->imu_pub_.push_back(_nh.advertise<sensor_msgs::Imu>(frames[i] + "/imu/data_raw", 2));
	  this->mag_pub_.push_back(_nh.advertise<sensor_msgs::MagneticField>(frames[i] + "/mag/data_raw", 2));
  }

  // soon deprecated
	this->_glove_data_pub = _nh.advertise<thu_data_glove::GloveData>("glove_joint", 2);
	ros::spin();
}

void rosBridge::sensorData_received(SensorData sd)
{
  if (!sd.gloveID == NEW_RH && !sd.gloveID == OLD_RH && !sd.gloveID == NEW_LH && !sd.gloveID == OLD_LH){
    ROS_ERROR_STREAM("Unknown glove ID: " << sd.gloveID);
    return;
  }
  // TODO rosparam
  if (sd.gloveID != OLD_RH)
    return;

  ros::Time time = ros::Time::now(); //todo

  for (int i=0; i<18; i++) {

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = frames[i];
    imu_msg.header.stamp = time;

    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(sd.sensor_data[i].roll *3.1415/180, -sd.sensor_data[i].pitch *3.1415/180, -sd.sensor_data[i].yaw *3.1415/180);

    imu_msg.angular_velocity.x = sd.sensor_data[i].Wx;
    imu_msg.angular_velocity.y = sd.sensor_data[i].Wy;
    imu_msg.angular_velocity.z = sd.sensor_data[i].Wz;

    imu_msg.linear_acceleration.x = sd.sensor_data[i].Ax;
    imu_msg.linear_acceleration.y = sd.sensor_data[i].Ay;
    imu_msg.linear_acceleration.z = sd.sensor_data[i].Az;

    this->imu_pub_[i].publish(imu_msg);

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.frame_id = frames[i];
    mag_msg.header.stamp = time;

    mag_msg.magnetic_field.x = sd.sensor_data[i].Mx;
    mag_msg.magnetic_field.y = sd.sensor_data[i].My;
    mag_msg.magnetic_field.z = sd.sensor_data[i].Mz;

    this->mag_pub_[i].publish(mag_msg);
  }
}

// soon deprecated
void rosBridge::gloveData_received(GloveData gd)
{
	thu_data_glove::GloveData data;
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
