Drone Show
==========

This project lets you control a swarm of drones to create custom shapes in the sky. It combines image processing, transformations, and the power of PX4 to orchestrate the drones' movements.

Excited to try it yourself? In this blog post, we'll walk you through running the system on your local machine.

Step-by-Step Guide
------------------

1. Open a new terminal and launch the gazebo world.

   .. code-block:: bash

      roslaunch sitl_tutorials spawn_gazebo_world.launch

2. Open another terminal and run the following command.

   .. code-block:: bash

      roscd sitl_tutorials/ && cd ..
      ./run_sim.bash

   Enter the number of drones when prompted. You will see the drones spawning automatically in Gazebo.

3. Open yet another terminal and execute the command to launch the mission control.

   .. code-block:: bash

      rosrun goal_allocator app.py

.. raw:: html

   <iframe width="700" height="400" src="https://www.youtube.com/embed/PH_vu_SqwpU?si=fSjG__Zh51kaApee" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


Collision Avoidance
---------------------

**Note:**
Please be aware that the current setup does not include any collision avoidance algorithm. As the number of drones increases, the risk of drone collisions may also rise.
