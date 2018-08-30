#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <thu_data_glove/ImuArray.h>

#define X_AXIS tf::Vector3(1,0,0)
#define Y_AXIS tf::Vector3(0,1,0)
#define Z_AXIS tf::Vector3(0,0,1)

ros::Publisher pub;
ros::Publisher tmp_pub;

bool init = false;
double alpha = 0.1;
sensor_msgs::JointState last_joint_states;

std::vector<std::string> joint_names{"ff_j4", "ff_j3", "ff_j2", "ff_j1", "mf_j4", "mf_j3", "mf_j2", "mf_j1", "rf_j4", "rf_j3", "rf_j2", "rf_j1", "lf_j4", "lf_j3", "lf_j2", "lf_j1", "th_j4", "th_j3", "th_j2", "th_j1"}; 

// imu1 = parent, imu2 = child
double getAngle(sensor_msgs::Imu imu1, sensor_msgs::Imu imu2, tf::Vector3 plane_norm) {

  tf::Quaternion q1;
  tf::Quaternion q2;

  tf::quaternionMsgToTF(imu1.orientation, q1);
  tf::quaternionMsgToTF(imu2.orientation, q2);

  q2 = tf::inverse(q1)*q2;
  q1 =tf::Quaternion(0,0,0,1);

/*
  sensor_msgs::Imu tmp;
  tmp.header.stamp = ros::Time::now();
  tmp.header.frame_id = "world";
  tf::quaternionTFToMsg(q1, tmp.orientation);
  tmp_pub.publish(tmp);
*/



  tf::Vector3 q1_v = tf::quatRotate(q1, tf::Vector3(1,0,0)); // x axis of parent
  tf::Vector3 q2_v = tf::quatRotate(q2, tf::Vector3(1,0,0)); // x axis of child

  // Project child axis on plane
  tf::Vector3 n_q1 = tf::quatRotate(q1, plane_norm); // plane normal for projection 
  double dist = n_q1.dot(q2_v); // distance of vector end to plane
  tf::Vector3 proj_x = q2_v - dist*n_q1; // Projected vector

  // Get angle between parent x and projected x from child
  double angle = q1_v.angle(proj_x);

  // Determine the sign
  tf::Vector3 cross_norm = tf::tfCross(q1_v, proj_x);
  angle = plane_norm.dot(cross_norm) < 0.f ? -angle : angle;

  return angle;

}

void eWMAFilter(sensor_msgs::JointState &joint_states) {
  if (!init) {
    last_joint_states = joint_states;
    init = true;
  }

  for(int i=0; i< joint_states.position.size(); i++) {
    joint_states.position[i] = alpha * joint_states.position[i] + (1-alpha) * last_joint_states.position[i];
  }
  last_joint_states = joint_states;
}

void callback(const thu_data_glove::ImuArrayConstPtr &imus) {
  sensor_msgs::JointState joint_states;

  joint_states.header.stamp = imus->header.stamp;

  joint_states.name = joint_names;

  // compute angles for joint states
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[11], Z_AXIS*-1));  // ff_j4
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[11], Y_AXIS));     // ff_j3
  joint_states.position.push_back(getAngle(imus->imu[11], imus->imu[10], Y_AXIS));     // ff_j2
  joint_states.position.push_back(getAngle(imus->imu[10], imus->imu[9],  Y_AXIS));     // ff_j1
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[8],  Z_AXIS*-1));  // mf_j4
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[8],  Y_AXIS));     // mf_j3
  joint_states.position.push_back(getAngle(imus->imu[8],  imus->imu[7],  Y_AXIS));     // mf_j2
  joint_states.position.push_back(getAngle(imus->imu[7],  imus->imu[6],  Y_AXIS));     // mf_j1
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[5],  Z_AXIS*-1));  // rf_j4
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[5],  Y_AXIS));     // rf_j3
  joint_states.position.push_back(getAngle(imus->imu[5],  imus->imu[4],  Y_AXIS));     // rf_j2
  joint_states.position.push_back(getAngle(imus->imu[4],  imus->imu[3],  Y_AXIS));     // rf_j1
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[2],  Z_AXIS*-1));  // lf_j4
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[2],  Y_AXIS));     // lf_j3
  joint_states.position.push_back(getAngle(imus->imu[2],  imus->imu[1],  Y_AXIS));     // lf_j2
  joint_states.position.push_back(getAngle(imus->imu[1],  imus->imu[0],  Y_AXIS));     // lf_j1
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[14], Z_AXIS*-1));  // th_j4
  joint_states.position.push_back(getAngle(imus->imu[17], imus->imu[14], Y_AXIS));     // th_j3
  joint_states.position.push_back(getAngle(imus->imu[14], imus->imu[13], Y_AXIS));     // th_j2
  joint_states.position.push_back(getAngle(imus->imu[13], imus->imu[12], Y_AXIS));     // th_j1

  // reduce noise
  eWMAFilter(joint_states);

  pub.publish(joint_states);
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "GloveJointStatePublisher");
  ros::NodeHandle nh;

  ros::Subscriber sub_wrist = nh.subscribe("/imu/data", 10, callback);

  pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

tmp_pub = nh.advertise<sensor_msgs::Imu>("test_Q", 1);

  ros::spin();

  return 0;
}

