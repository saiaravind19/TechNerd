Multi drone sim
==========================

In this tutorial, we will learn how to spawn a multi-drone simulation using ROS and Gazebo.

Step by Step Guide
------------------------

- Open a new terminal and launch the gazebo world.

.. code-block:: bash

    roslaunch sitl_tutorials spawn_gazebo_world.launch

- Open another terminal and run the following command. This will spawn an iris drone and trigger offboard mode.

.. code-block:: bash

    roslaunch sitl_tutorials spawn_sitl.launch

- Open yet another terminal to spawn a new drone with a different name and location.

.. code-block:: bash

    roslaunch sitl_tutorials spawn_sitl.launch uav_name:=iris_2 pose_y:=5 ID:=2

Voila! You should now see multiple drones spawned and started flying to reach the setpoint in the local coordinate frame.

.. raw:: html
    
    <iframe width="700" height="400" src="https://www.youtube.com/embed/x8q6UnfGvMA?si=ZIvxoS4WMU-m-f9g" title="Multi drone sim" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
    </iframe>


Explanation
-----------

Let's take a closer look at the launch arguments of `spawn_sitl.launch`:

- **pose_x**: The initial x-coordinate position of the drone.

- **pose_y**: The initial y-coordinate position of the drone.

- **pose_z**: The initial z-coordinate position of the drone.

- **uav_name**: The name of the drone. This can be used to identify each drone in the simulation.

- **ID**: Unique identifier for each drone. This can ID is used to define mavlink udp,tcp port for PX4.

- **module**: Specifies the module or plugin to be used for the drone. This can be customized based on the drone's capabilities.

- **uav_type**: Specifies the type of drone to be spawned, e.g., iris, plane, vtol etc.

By customizing these launch arguments, you can spawn multiple drones with different configurations in the simulation. This allows you to test and develop multi-drone control algorithms, swarm behaviors, and more.
