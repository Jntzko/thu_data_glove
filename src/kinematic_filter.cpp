#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <thu_data_glove/ImuArray.h>

bool bias;
std::vector<tf::Quaternion> bias_values;
ros::Publisher pub;

void callback(const thu_data_glove::ImuArrayConstPtr &msg) {
  thu_data_glove::ImuArray imus = *msg;

  // substract biased wrist from all finger imus
  tf::Quaternion wrist_q;
  tf::quaternionMsgToTF(imus.imu[15].orientation, wrist_q);

  for (int i=0; i<imus.imu.size(); i++) {
    if (i == 15)
      continue;

    tf::Quaternion q;
    tf::quaternionMsgToTF(imus.imu[i].orientation, q);

    if (bias) {
      bias_values[i] = tf::inverse(wrist_q)*q;
    }
    // bias raw imu orientation
    q *= tf::inverse(bias_values[i]);
    tf::quaternionTFToMsg(q, imus.imu[i].orientation);
  }
  bias = false;

  pub.publish(imus);
}

bool serviceCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  bias = true;
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "KinematicFilter");
  ros::NodeHandle nh;

  bias = true;
  bias_values.resize(18);

  ros::Subscriber sub = nh.subscribe("/imu/data_raw", 1, callback);
  pub = nh.advertise<thu_data_glove::ImuArray>("/imu/data",2);

  // Bias service
  ros::ServiceServer service = nh.advertiseService("bias_imus", serviceCallback);

  ros::spin();

  return 0;
}
