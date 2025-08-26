Velocity control
==========================

In this tutorial we will explore how to control the Iris drone using velocity commands in offboard mode.
For this tutorial, we'll utilize a teleop node to generate velocity commands.

Prerequicts
---------------------
Install teleop_keyboard package which takes keyboard inputs and publish in cmd_vel topic.

.. code-block:: bash

	sudo apt-get install ros-noetic-teleop-twist-keyboard

Step by Step Guide
------------------------

- Open a new terminal and launch the gazebo world

.. code-block:: bash

    roslaunch sitl_tutorials spawn_gazebo_world.launch

- In another terminal, run the following command to spawn the Iris drone and trigger the offboard mode:

.. code-block:: bash

    roslaunch sitl_tutorials spawn_sitl.launch module:=2

-  In yet another terminal, execute the following command to run the `keyboard_teleop` node, which publishes velocity commands to the `/cmd_vel` topic:

.. code-block:: bash

	rosrun teleop_twist_keyboard teleop_twist_keyboard.py


You can now fly the drone using the teleop keyboard commands. Try out the controls and let me know how it feels.

.. raw:: html

   <iframe width="700" height="400" src="https://www.youtube.com/embed/Dck6KpjZeBY?si=UWcLI-oBzXBHA1l9" title="Offboard control" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
   </iframe>

Code
---------

.. code-block:: cpp

	#include <ros/ros.h>
	#include <geometry_msgs/TwistStamped.h>
	#include <geometry_msgs/Twist.h>
	#include <mavros_msgs/CommandBool.h>
	#include <mavros_msgs/SetMode.h>
	#include <mavros_msgs/State.h>
	#include <mavros_msgs/SetMavFrame.h>

	geometry_msgs::TwistStamped velocity;
	mavros_msgs::State current_state;
	geometry_msgs::Twist keyboard_cmd_vel;

	// Callback function for copter state
	void state_cb(const mavros_msgs::State::ConstPtr& msg){
		current_state = *msg;
	}

	// Callback function for keyboard input
	void keyboard_teleop_callback(const geometry_msgs::Twist::ConstPtr& msg){
		keyboard_cmd_vel = *msg;
	}

	int main(int argc, char **argv)
	{

		ros::init( argc, argv, "keyboard_node");
		ros::NodeHandle nh;

		// subscriber
		ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
		ros::Subscriber keyboard_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, keyboard_teleop_callback);

		// publisher
		ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",10);

		// client
		ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

		// the setpoint publishing rate must be faster that 2Hz
		ros::Rate rate(20.0);

		// wait for FCU connection
		while(ros::ok() && !current_state.connected){
			ros::spinOnce();
			rate.sleep();
		}

		keyboard_cmd_vel.linear.x= 0;
		keyboard_cmd_vel.linear.y= 0;
		keyboard_cmd_vel.linear.z= 0;

        /* MAVROS message for setting the mode
        Reference: different modes available :http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
        */
    	mavros_msgs::SetMode offb_set_mode;
    	offb_set_mode.request.custom_mode = "OFFBOARD";
        // MAVROS message for arming
    	mavros_msgs::CommandBool arm_cmd;
    	arm_cmd.request.value = true;

		ros::Time last_request = ros::Time::now();

		while(ros::ok()){
			if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
				{
					ROS_INFO("Offboard enabled");
				}
				last_request = ros::Time::now();
			}
			else
			{
				if( !current_state.armed && (ros::Time::now() -last_request > ros::Duration(5.0)))
				{
					if( arming_client.call(arm_cmd) && arm_cmd.response.success)
					{
						ROS_INFO("Vehicle armed");
					}
					last_request = ros::Time::now();
				}
			}
			velocity_pub.publish(keyboard_cmd_vel); 
			ros::spinOnce();
			rate.sleep();
		}
		return 0;


	}

Code Explanation
----------------

First, we instantiate subscribers, publishers, and clients with appropriate topics and services to request arming and mode change. If you are launching the node using a launch file, check for the `group` keyword to know the namespace name.

- `mavros/setpoint_velocity/cmd_vel`: Data is published with respect to the local FLU frame, and PX4 converts this to the intended FRD frame.

.. code-block:: cpp

    	// subscriber
    	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    	ros::Subscriber keyboard_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, keyboard_teleop_callback);

    	// publisher
		ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",10);

    	// client
    	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

PX4 has a timeout of 500ms between two OFFBOARD commands. If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering OFFBOARD mode. Therefore, we are setting the rate to 20Hz.

Before moving on to the next step, we need to ensure that PX4 has started successfully and a connection is established between MAVROS and the autopilot. The loop breaks once the connection is established, i.e., `current_state.connected` is set to `True`.

Next, initialize the MAVROS message for arming the drone and setting the drone to offboard mode. Now we will set the goal point using `geometry_msgs`. Note that we are setting the goal point in the local frame of reference, so the origin will be the starting point of the drone.

.. code-block:: cpp

        /* MAVROS message for setting the mode
        Reference: different modes available :http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
        */
    	mavros_msgs::SetMode offb_set_mode;
    	offb_set_mode.request.custom_mode = "OFFBOARD";
        // MAVROS message for arming
    	mavros_msgs::CommandBool arm_cmd;
    	arm_cmd.request.value = true;

Function callback for `mavros/state`  which has information about current state of autopilot

.. code-block:: cpp

	// Callback function for copter state
	void state_cb(const mavros_msgs::State::ConstPtr& msg){
		current_state = *msg;
	}

Function callback for `/cmd_vel` published by teleop node 

.. code-block:: cpp 

	void keyboard_teleop_callback(const geometry_msgs::Twist::ConstPtr& msg){
		keyboard_cmd_vel = *msg;
	}

Once the PX4 is connected, we will try to arm the drone and then change the drone to Offboard mode. 
Once the drone changes the mode, we will break from the while loop in the mean time default velocity commands are published.

.. code-block:: cpp

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if( !current_state.armed && (ros::Time::now() -last_request > ros::Duration(5.0)))
			{
				if( arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		velocity_pub.publish(keyboard_cmd_vel); 
		ros::spinOnce();
		rate.sleep();
	}

