
/mavros/setpoint* if in case you want to use PX4 planner completely
http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack


Code
----------------

.. code-block:: cpp

   #include <ros/ros.h>
   #include <geometry_msgs/PoseStamped.h>
   #include <mavros_msgs/CommandBool.h>
   #include <mavros_msgs/SetMode.h>
   #include <mavros_msgs/State.h>

   mavros_msgs::State current_state;
   ros::ServiceClient set_mode_client;

   void state_cb(const mavros_msgs::State::ConstPtr& msg){
       current_state = *msg;
   }

   int main(int argc, char **argv)
   {
       ros::init(argc, argv, "offb_node");
       ros::NodeHandle nh;

       ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
               ("mavros/state", 10, state_cb);
       ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
               ("mavros/setpoint_position/local", 10);
       ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
               ("mavros/cmd/arming");
       set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
               ("mavros/set_mode");

       ros::Rate rate(20.0);

       //the setpoint publishing rate MUST be faster than 2Hz

       // wait for FCU connection
       while(ros::ok() && !current_state.connected){
           ros::spinOnce();
           rate.sleep();
       }

       //mavros msg for arming
       mavros_msgs::CommandBool arm_cmd;
       arm_cmd.request.value = true;

       //mavros msg for setting the mode
       mavros_msgs::SetMode offb_set_mode;
       offb_set_mode.request.custom_mode = "OFFBOARD";

       //Mavros msg used to arm 
       mavros_msgs::CommandBool arm_cmd;
       arm_cmd.request.value = true;

       /* set the goal position of the drone.
       The location is with respect to the local frame which is relative to the starting point of the drone.
       */
       geometry_msgs::PoseStamped pose;
       pose.pose.position.x = 0;
       pose.pose.position.y = 0;
       pose.pose.position.z = 2;


       ros::Time last_request = ros::Time::now();

       while(ros::ok()){

           //set the drone mode to offboard mode 
           if( current_state.mode != "OFFBOARD" &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( set_mode_client.call(offb_set_mode) &&
                   offb_set_mode.response.mode_sent){
                   ROS_INFO("Offboard enabled");
               }
               last_request = ros::Time::now();

           } 
           else {
               // Once the drone is in offboard mode arm the drone 
               if( !current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0))){
                   if( arming_client.call(arm_cmd) &&
                       arm_cmd.response.success){
                       ROS_INFO("Vehicle armed");
                   }
                   last_request = ros::Time::now();
               }
           }
           // publish the goal point at desired rate 
           local_pos_pub.publish(pose);

           ros::spinOnce();
           rate.sleep();
       }

       return 0;
   }



Code Explanation
------------------

First we instantiate subscriber,publishers and clients with appropriate topics and services to request arming and mode change.
Incase you are using launching the node using the launch file check for group keyword to know the namespace name.

.. code-block:: cpp 

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

PX4 has a timeout of 500ms between two OFFBOARD commands.If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering OFFBOARD mode.So we are setting the rate to 20Hz.

Before moving on to the next step we need to ensure that PX4 has started successfully and connection is established between MAVROS and autopilot.
The loop breaks once the connection is estabalished i.e current_state.connected is set to True.

Next initlise the mavros msg for arming the drone,setting the drone to offboard mode
Now we will set the goal point using geometry_msgs . Note we are set the goal point in local frame of refence so the origin will be the starting point of the drone.

.. code-block:: cpp
   
   ros::Rate rate(20.0);

    //the setpoint publishing rate MUST be faster than 2Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //mavros msg for arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //mavros msg for setting the mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //Mavros msg used to arm 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    /* set the goal position of the drone.
    The location is with respect to the local frame which is relative to the starting point of the drone.
    */
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;