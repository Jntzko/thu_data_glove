#include <ros/ros.h>
#include <thu_data_glove/ImuArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

static std::vector<std::string> frames{ "lf_distal", "lf_middle", "lf_proximal", "rf_distal", "rf_middle", "rf_proximal", "mf_distal", "mf_middle", "mf_proximal", "ff_distal", "ff_middle", "ff_proximal", "th_distal", "th_middle", "th_proximal", "wrist", "arm", "palm"};

tf::TransformListener* listener;
tf::TransformBroadcaster* broadcaster;

void callback(const thu_data_glove::ImuArrayConstPtr &imus) {
  std::vector<geometry_msgs::TransformStamped> tf_transforms;

  // loop over all imus
  for(int i=0; i<imus->imu.size(); i++) {
    geometry_msgs::TransformStamped tf_transform;
    tf_transform.header.stamp = ros::Time::now();
    tf_transform.header.frame_id = "world"; 
    tf_transform.child_frame_id = frames[i] + "_imu";
    tf_transform.transform.rotation = imus->imu[i].orientation;
    try{
      tf::StampedTransform parent_tf;
      listener->lookupTransform("world", frames[i] + "_imu_link",  ros::Time(0), parent_tf);
      tf_transform.transform.translation.x = parent_tf.getOrigin().x();
      tf_transform.transform.translation.y = parent_tf.getOrigin().y();
      tf_transform.transform.translation.z = parent_tf.getOrigin().z();
    }
    catch (tf::TransformException ex){}

    tf_transforms.push_back(tf_transform);
  }

  geometry_msgs::TransformStamped tf_transform;
  tf_transform.header.stamp = ros::Time::now();
  tf_transform.header.frame_id = "world"; 
  tf_transform.child_frame_id = "wrist";
  tf_transform.transform.rotation = imus->imu[15].orientation;

  tf_transforms.push_back(tf_transform);
  
  broadcaster->sendTransform(tf_transforms);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ImuTransformBroadcaster");
  ros::NodeHandle nh;

  ros::Subscriber sub_wrist = nh.subscribe("/imu/data", 10, callback);

  broadcaster = new tf::TransformBroadcaster();
  listener = new tf::TransformListener();

  ros::spin();
  return 0;
}

