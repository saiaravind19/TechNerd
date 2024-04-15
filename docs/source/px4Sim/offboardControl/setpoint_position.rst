Setpoint Position
==========================
In this tutorial, we will learn how to use **setpoint_position** in offboard mode. By the end of this tutorial, we will understand how to hover the iris at a setpoint.

Step by Step Guide
------------------------

- Open a new terminal and launch the gazebo world

.. code-block:: bash

    roslaunch sitl_tutorials spawn_gazebo_world.launch

- Open  another terminal and run the following which will spawn iris and triggers offboard mode.

.. code-block:: bash

    roslaunch sitl_tutorials spawn_sitl.launch


Once the drone is spawned, it should start flying and stabilize itself at a certain point, just like you've seen in the video.

.. raw:: html

    <iframe width="700" height="400" src="https://www.youtube.com/embed/Dck6KpjZeBY?si=UWcLI-oBzXBHA1l9" title="Offboard control" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
    </iframe>


Code
----

.. code-block:: cpp

    #include <ros/ros.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <mavros_msgs/CommandBool.h>
    #include <mavros_msgs/SetMode.h>
    #include <mavros_msgs/State.h>

    mavros_msgs::State current_state;
    
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "offb_node");
        ros::NodeHandle nh;

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
                "mavros/state", 10, state_cb);
        ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
                "mavros/setpoint_position/local", 10);
        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
                "mavros/cmd/arming");
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
                "mavros/set_mode");

        ros::Rate rate(20.0);

        // The setpoint publishing rate MUST be faster than 2Hz

        // Wait for FCU connection
        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        // MAVROS message for arming
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        /* MAVROS message for setting the mode
        Reference: different modes available :http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
        */
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        // Set the goal position of the drone
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;

        ros::Time last_request = ros::Time::now();

        while(ros::ok()){

            // Set the drone mode to offboard mode
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();

            } else {
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

        return 0;
    }

Code Explanation
----------------

First, we instantiate subscribers, publishers, and clients with appropriate topics and services to request arming and mode change. If you are launching the node using a launch file, check for the `group` keyword to know the namespace name.

- `mavros/setpoint_position/local`: Data is published with respect to the local FLU frame, and PX4 converts this to the intended FRD frame.

.. code-block:: cpp

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
            "mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode");

PX4 has a timeout of 500ms between two OFFBOARD commands. If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering OFFBOARD mode. Therefore, we are setting the rate to 20Hz.

Before moving on to the next step, we need to ensure that PX4 has started successfully and a connection is established between MAVROS and the autopilot. The loop breaks once the connection is established, i.e., `current_state.connected` is set to `True`.

Next, initialize the MAVROS message for arming the drone and setting the drone to offboard mode. Now we will set the goal point using `geometry_msgs`. Note that we are setting the goal point in the local frame of reference, so the origin will be the starting point of the drone.

.. code-block:: cpp

    ros::Rate rate(20.0);

    // The setpoint publishing rate MUST be faster than 2Hz

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // MAVROS message for arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // MAVROS message for setting the mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Set the goal position of the drone
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

Once the PX4 is connected, we will try to arm the drone and then change the drone to Offboard mode. Once the drone changes the mode, it will start moving closer to the setpoint.

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