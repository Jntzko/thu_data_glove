#include <thu_data_glove/rosbridge.h>
#include <thu_data_glove/SensorData.h>
#include <thu_data_glove/GloveData.h>
#include <thu_data_glove/ImuArray.h>
#include <signal.h>
#include <QtCore/QDebug>
#include <QtCore/QCoreApplication>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/tf.h>

#define NEW_LH 1192974848
#define NEW_RH 1193564786
#define OLD_LH 1192778367
#define OLD_RH 1192974877

static std::vector<std::string> frames{ "lf_distal", "lf_middle", "lf_proximal", "rf_distal", "rf_middle", "rf_proximal", "mf_distal", "mf_middle", "mf_proximal", "ff_distal", "ff_middle", "ff_proximal", "th_distal", "th_middle", "th_proximal", "wrist", "arm", "palm"};

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

	this->pub_ = _nh.advertise<thu_data_glove::ImuArray>("/imu/data_raw", 2);

  this->bias_values_.resize(18);
  this->prev_values_.resize(18);
	ros::spin();
}

void rosBridge::sensorData_received(SensorData sd)
{
  if (!sd.gloveID == NEW_RH && !sd.gloveID == OLD_RH && !sd.gloveID == NEW_LH && !sd.gloveID == OLD_LH){
    ROS_ERROR_STREAM("Unknown glove ID: " << sd.gloveID);
    return;
  }
  // TODO rosparam
  if (sd.gloveID != NEW_LH)
    return;

  ros::Time time = ros::Time::now(); //todo

  thu_data_glove::ImuArray imus;
  imus.header.stamp = ros::Time::now();

  for (int i=0; i<18; i++) {
    if (i == 15) { // broken wrist imu, use palm values
      // Imu
      sensor_msgs::Imu imu_msg;
      imu_msg.header.frame_id = frames[i];
      imu_msg.header.stamp = time;

      imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(sd.sensor_data[17].roll *3.1415/180, -sd.sensor_data[17].pitch *3.1415/180, -sd.sensor_data[17].yaw *3.1415/180);

      imu_msg.angular_velocity.x = sd.sensor_data[17].Wx;
      imu_msg.angular_velocity.y = sd.sensor_data[17].Wy;
      imu_msg.angular_velocity.z = sd.sensor_data[17].Wz;

      imu_msg.linear_acceleration.x = sd.sensor_data[17].Ax;
      imu_msg.linear_acceleration.y = sd.sensor_data[17].Ay;
      imu_msg.linear_acceleration.z = sd.sensor_data[17].Az;

      imus.imu.push_back(imu_msg);

      // Magnetic field
      sensor_msgs::MagneticField mag_msg;
      mag_msg.header.frame_id = frames[i];
      mag_msg.header.stamp = time;

      mag_msg.magnetic_field.x = sd.sensor_data[17].Mx;
      mag_msg.magnetic_field.y = sd.sensor_data[17].My;
      mag_msg.magnetic_field.z = sd.sensor_data[17].Mz;

      imus.magnetic_field.push_back(mag_msg);
      continue;
    }
    // Imu
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

    imus.imu.push_back(imu_msg);

    // Magnetic field
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.frame_id = frames[i];
    mag_msg.header.stamp = time;

    mag_msg.magnetic_field.x = sd.sensor_data[i].Mx;
    mag_msg.magnetic_field.y = sd.sensor_data[i].My;
    mag_msg.magnetic_field.z = sd.sensor_data[i].Mz;

    imus.magnetic_field.push_back(mag_msg);
  }
  bias_ = true;
  this->pub_.publish(imus);
}

// soon deprecated
void rosBridge::gloveData_received(GloveData gd)
{
}
