Background knowledge 
===================================

PX4 uses the uORB message bus for internal communication among its various modules. 
For connections with the external world, such as simulation or other hardware, PX4 uses protocols like MAVLink for ground control and uXRCE-DDS for real-time communication with companion computer over ROS2.

As the current tutorials are based on ROS1, we employ MAVROS packages. MAVROS enables MAVLink extendable communication between computers running ROS.

Flight Modes in Multicopter
-----------------------------------

Different flight modes are available in PX4, enabling the autopilot to function in a deterministic way.
These modes are designed to meet specific requirements, enhance safety, and facilitate efficient operation by automating common tasks.

Modes in Multicopter are grouped into three categories: `manual, autonomous, and manual-acrobatic`.

**Manual-Easy:**
   - ``Position mode`` Easiest and safest manual mode for vehicles with a position fix/GPS. It controls acceleration over ground, yaw, and throttle.
   - ``Position Slow mode`` A velocity and yaw rate limited version of Position mode, used to limit speed temporarily.
   - ``Altitude mode`` Safest non-GPS manual mode; levels and maintains altitude upon releasing sticks.
   - ``Manual/Stabilized`` Releasing the sticks levels the vehicle horizontally, maintaining momentum.
**Manual-Acrobatic:**
   - ``Acro`` Manual mode for acrobatic maneuvers, such as rolls and loops.
**Autonomous:**
   - ``Hold`` Vehicle stops and hovers at its current position and altitude.
   - ``Return`` Ascends to a safe altitude, flies to a safe location, and lands (requires GPS).
   - ``Mission`` Executes a predefined flight plan uploaded to the flight controller (requires GPS).
   - ``Takeoff`` Vertical takeoff followed by switching to Hold mode.
   - ``Land`` Immediate landing.
   - ``Orbit`` Flies in a circle, facing towards the center.
   - ``Follow Me`` Follows a beacon providing position setpoints.
   - ``Offboard`` Obeys position, velocity, or attitude setpoints provided via MAVLink or ROS 2.

For more information about the modes, refer to the `PX4 documentation <https://docs.px4.io/main/en/flight_modes_mc/>`_.

Important Services
------------------------------

- `/mavros/cmd/arming <http://wiki.ros.org/mavros#mavros.2FPlugins.Services>`_  : MAVROS service used to arm the drone.

- `/mavros/cmd/land <http://wiki.ros.org/mavros#mavros.2FPlugins.Services>`_    : Service call. When set to true, returns a bool once landing is completed successfully.

- `/mavros/set_mode <http://wiki.ros.org/mavros#mavros.2FPlugins.command>`_     : Service call to set the drone to specific mode, whether it be in manual or autonomous mode .


Some Info
---------------------
Mavros offers a window into the global and local coordinates of drones.

1. **Global Coordinates (`/mavros/global*`):**

   - All the data published are with respect to world coorodinates systems so it has GPS data offering global perspective. 
   - Includes GPS information (latitude, longitude, altitude).
   - Ideal for global navigation, waypoint coordination, and external system integration.

2. **Local Perspective (`/mavros/local*`):**

   - Offers data in the drone's local frame of reference,relative to its starting position.
   - Data published shouch as odemetry published using the starting point as refernce.
   - Used for precision tasks, obstacle avoidance, and local maneuvers.


Coordinate Frames:
---------------------------

In robotics and computer graphics, coordinate frames are essential for representing the position and orientation of objects in space relative to a reference point.Each frame has its own origin and axes, allowing us to perform transformations between them.
By understanding how these frames relate to each other, we can perform transformations between end-effector position and vice versa.

In PX4 there are different frame conventions.The local/world and body frames used by ROS and PX4 are different and it is important to give appropriate commands considering the orienttaion of these frames.

.. list-table::
   :widths: 25 25 50
   :header-rows: 1

   * - Frame
     - PX4
     - ROS
   * - Body 
     - FRD (X Forward, Y Right, Z Down) 
     - FLU (X Forward, Y Left, Z Up)
   * - World
     - FRD or NED (X North, Y East, Z Down)
     - FLU or ENU (X East, Y North, Z Up)

  
Both frames are shown in the image below (FRD on the left/FLU on the right).

.. image:: ../images/frame.png
    :alt:  Frame representation of NED and ENU 

`Image source <https://docs.px4.io/main/en/ros/ros2_comm.html>`_

Frema Conversion
------------------

1. **FLU to FRD :**
   
   - To rotate from FLU to FRD a pi rotation around the X-axis (front) is sufficient.
  
  .. math::

    R_{FLU\_to\_FRD} = \begin{bmatrix}
    1 & 0 & 0 \\
    0 & -1 & 0 \\
    0 & 0 & -1 \\
    \end{bmatrix}

2. **NED to ENU :**
   
   - First a pi/2 rotation around the Z-axis (up),
   - Then a pi rotation around the X-axis (old East/new North). 
  
  .. math::

      R_{NED\_to\_ENU} = \begin{bmatrix}
      0 & 1 & 0 \\
      1 & 0 & 0 \\
      0 & 0 & -1 \\
      \end{bmatrix}

3. **ENU to NED :**
   
   - First a pi/2 rotation around the Z-axis (down),
   - Then a pi rotation around the X-axis (old North/new East).
  
  .. math::

    R_{NED\_to\_ENU} = \begin{bmatrix}
    0 & 1 & 0 \\
    1 & 0 & 0 \\
    0 & 0 & -1 \\
    \end{bmatrix}

https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL



Now that we has some knowledge of local and global namespaces used so in this tutorial we will have a demo using the local coordinate to to navigate the drone to the desired setpoint.
https://docs.px4.io/main/en/ros/external_position_estimation.html
One can alse refer `mavros_msgs/State <https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/State.html>`_  msg for the all the pissible modes. 
