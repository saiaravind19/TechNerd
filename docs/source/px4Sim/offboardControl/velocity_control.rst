Velocity control
==========================

Prerequicts
-----------------
Install teleop_keyboard package which takes keyboard inputs and publish in cmd_vel topic.

.. code-block:: bash

	sudo apt-get install ros-<distro>-teleop-twist-keyboard


- Open a new terminal and launch the gazebo world

.. code-block:: bash

    roslaunch mavros_bridge spawn_gazebo_world.launch

- Open one more terminal and run the following which will spawn iris and triggers offboard mode.

.. code-block:: bash

    roslaunch mavros_bridge spawn_sitl.launch module:=2

- Open one more terminal and run the following to run keyboard_teleop node which will publish velocity commands in `/cmd_vel` topic.

.. code-block:: bash
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py


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

    geometry_msgs::TwistStamped remap_velocity_cmd()
    {
    velocity.header.stamp = ros::Time::now();
    velocity.twist.linear.x = keyboard_cmd_vel.linear.x;
    velocity.twist.linear.y = keyboard_cmd_vel.linear.y;
    velocity.twist.linear.z = keyboard_cmd_vel.linear.z;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = keyboard_cmd_vel.angular.z;
    return velocity;

    }


    int main(int argc, char **argv)
    {


    	ros::init( argc, argv, "keyboard_node");
    	ros::NodeHandle nh;

    	// subscriber
    	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    	ros::Subscriber keyboard_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, keyboard_teleop_callback);

    	// publisher
    	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

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
    	ROS_INFO("FCU connected");

    	// Change from FRAME_LOCAL_NED to FRAME_BODY_NED
    	mavros_msgs::SetMavFrame frame_id;
    	frame_id.request.mav_frame = 8;

    	// send a few setpoints before starting
    	for( int i = 100; ros::ok() && i > 0; --i){
    		velocity.header.stamp = ros::Time::now();
    		velocity_pub.publish(velocity);
    		ros::spinOnce();
    		rate.sleep();
    	}

        /* MAVROS message for setting the mode
        Reference: different modes available :http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
        */
    	mavros_msgs::SetMode offb_set_mode;
    	offb_set_mode.request.custom_mode = "OFFBOARD";

    	velocity.twist.linear.x = 0;
    	velocity.twist.linear.y = 0;
    	velocity.twist.linear.z = -1;
        // MAVROS message for arming
    	mavros_msgs::CommandBool arm_cmd;
    	arm_cmd.request.value = true;

    	ros::Time last_request = ros::Time::now();

    	while(ros::ok()){
            if (current_state.mode == "OFFBOARD" && current_state.armed ) break;
    		else if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
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
    		velocity_pub.publish(remap_velocity_cmd()); 
    		ros::spinOnce();
    		rate.sleep();
    	}

    	while(ros::ok()){
    		velocity_pub.publish(remap_velocity_cmd()); 
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
    	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

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
		
    	velocity.twist.linear.x = 0;
    	velocity.twist.linear.y = 0;
    	velocity.twist.linear.z = -1;
        // MAVROS message for arming
    	mavros_msgs::CommandBool arm_cmd;
    	arm_cmd.request.value = true;

Function callback subscribing to `/cmd_vel` which is published using keyboard_teleop node 

.. code-block:: cpp

    geometry_msgs::TwistStamped remap_velocity_cmd()
    {
    velocity.header.stamp = ros::Time::now();
    velocity.twist.linear.x = keyboard_cmd_vel.linear.x;
    velocity.twist.linear.y = keyboard_cmd_vel.linear.y;
    velocity.twist.linear.z = keyboard_cmd_vel.linear.z;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = keyboard_cmd_vel.angular.z;
    return velocity;

    }

Once the PX4 is connected, we will try to arm the drone and then change the drone to Offboard mode. 
Once the drone changes the mode, it will start execuiting the velocity commands from keyboard.

.. code-block:: cpp

    while(ros::ok()){
       // Set the drone mode to offboard mode
       if( current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - last_request > ros::Duration(5.0))){
           if( set_mode_client.call(offb_set_mode) &&
               offb_set_mode.response.mode_sent){
               ROS_INFO("Offboard enabled");
           }
           last_request = ros::Time::now();
        } 
        else {
           // Once the drone is in offboard mode, arm the drone
           if( !current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( arming_client.call(arm_cmd) &&
                   arm_cmd.response.success){
                   ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            // Publish the goal point at desired rate
            local_pos_pub.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }

