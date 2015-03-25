// joy teleop turtlesim example 2014-02-19 LLW
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <unistd.h>

// global cmd vel twist
static geometry_msgs::Twist command_velocity;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  /* mapping from joystick to turtle velocity
  
    X vel proportional to left joystick fore and aft
    Y vel proportional to left joystick left and right
    Z vel proportional to Y and A butons
    rotation vel about X proportional to X and B butons
    rotation vel about Y proportional to right joysticj fore and aft
    rotation vel about Z proportional to right joysticj left and right

  */

  // X vel driven by left joystick for and aft
  command_velocity.linear.x  = 2.0*joy->axes[1];
  command_velocity.linear.y  = 2.0*joy->axes[0];
  command_velocity.linear.z  = 2.0*joy->axes[1];
  command_velocity.linear.z  = -2.0*joy->buttons[0] + 2.0*joy->buttons[3];
  // heading driven by left joysticj left and right
  command_velocity.angular.x  = -2.0*joy->buttons[1] + 2.0*joy->buttons[2];
  command_velocity.angular.y  = 2.0*joy->axes[4];
  command_velocity.angular.z  = 2.0*joy->axes[3];

}


int main(int argc, char** argv)
{

  // init ros
  ros::init(argc, argv, "myrobot_teleop_joy");

  // create node handle
  ros::NodeHandle node;

  // advertise topic that this node will publish
  ros::Publisher  turtle_vel =
    node.advertise<geometry_msgs::Twist>("myrobot/cmd_vel", 10);

  // subcscribe to joy topic
  ros::Subscriber sub = node.subscribe("joy", 10, &joyCallback);

  // lets publish at 20 Hz
  ros::Rate rate(20);

  // this is a crude way to publish at 20 Hz.  Better to use timers.
  while (node.ok())
    {
      // publish the cmd vel
      turtle_vel.publish(command_velocity);
 
      // spin
      ros::spinOnce();

      // wait 50 ms
      rate.sleep();

    }

  return 0;

};


