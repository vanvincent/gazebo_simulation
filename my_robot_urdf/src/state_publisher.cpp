#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <unistd.h>

// myrobot_tf frame
static tf::Transform myrobot_tf(tf::Transform::getIdentity());

void tfCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  /* mapping from joystick to turtle velocity
  
    X vel proportional to left joystick fore and aft
    Y vel proportional to left joystick left and right
    Z vel proportional to Y and A butons
    rotation vel about X proportional to X and B butons
    rotation vel about Y proportional to right joysticj fore and aft
    rotation vel about Z proportional to right joysticj left and right

  */
  //declare variables
  static tf::TransformBroadcaster br;
  static ros::Time now = ros::Time::now();
  static ros::Time last;
  tf::Transform myrobot_tf_delta; //small change for frame
  Eigen::Matrix3d R(3,3); //rotation matrix
  Eigen::Matrix3d J(3,3); //jacobian
  Eigen::Vector3d Vel(3); //linear velocity
  //get a time period
  last = now;
  ros::Duration dt;
  now = ros::Time::now();
  dt = now -last;
  //update linear velocity
  Vel(0) = twist->linear.x * dt.toSec();
  Vel(1) = twist->linear.y * dt.toSec();
  Vel(2) = twist->linear.z * dt.toSec();
  myrobot_tf_delta.setOrigin(tf::Vector3( Vel(0),Vel(1),Vel(2)));
  //compute and update roation matrix
  J(0,0) = 0.;
  J(0,1) = -twist->angular.z;
  J(0,2) =  twist->angular.y;
  J(1,0) =  twist->angular.z;
  J(1,1) = 0.;
  J(1,2) = -twist->angular.x;
  J(2,0) = -twist->angular.y;
  J(2,1) =  twist->angular.x;
  J(2,2) = 0.;
  J = J * dt.toSec();
  R = J.exp();
  myrobot_tf_delta.setBasis(tf::Matrix3x3 (
        R(0,0),R(0,1),R(0,2),
        R(1,0),R(1,1),R(1,2),
        R(2,0),R(2,1),R(2,2)
    ));
  myrobot_tf = myrobot_tf * myrobot_tf_delta;
  //broadcast transform
  br.sendTransform(tf::StampedTransform(myrobot_tf, ros::Time::now(),"world","base_link"));

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    //tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);
    ros::Subscriber sub = n.subscribe("myrobot/cmd_vel", 10, &tfCallback);

    const double degree = M_PI/180;

    // robot state
    double left=M_PI/2, tinc_l = degree*5, tinc_r = degree*5, right=-M_PI/2, angle=0, head=0, hinc=0.005;

    // message declarations
    //geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    //odom_trans.header.frame_id = "odom";
    //odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] ="joint1";
        joint_state.position[0] = 0;
        joint_state.name[1] ="joint2";
        joint_state.position[1] = 0;
        joint_state.name[2] ="joint3";
        joint_state.position[2] = 0;
        joint_state.name[3] ="joint4";
        joint_state.position[3] = left;
        joint_state.name[4] ="joint5";
        joint_state.position[4] = right;
        joint_state.name[5] ="joint6";
        joint_state.position[5] = head;

        // update transform
        // (moving in a circle with radius=2)
        //odom_trans.header.stamp = ros::Time::now();
        //odom_trans.transform.translation.x = cos(angle)*2;
        //odom_trans.transform.translation.y = sin(angle)*2;
        //odom_trans.transform.translation.z = .2;
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        //broadcaster.sendTransform(odom_trans);

        // Create new robot state
        left += tinc_l;
        if (left<0 || left>M_PI) tinc_l *= -1;
        right += tinc_r;
        if (right<-M_PI || right>0) tinc_r *= -1;
        head += hinc;
        if (head>.05 || head<-.15) hinc *= -1;
        
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();

        ros::spinOnce();
    }


    return 0;
}