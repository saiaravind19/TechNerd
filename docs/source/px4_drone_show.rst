Drone swarm formation
======================

This project lets you control a swarm of drones to create custom shapes in the sky. It combines image processing, transformations, and the power of PX4 to orchestrate the drones' movements.

Excited to try it yourself? In this blog post, we'll walk you through running the system on your local machine.

Installation
-----------------------
For settinup work space and dependency liberarues please follow :doc:`installation guide <px4Sim/Installation>`.

Step-by-Step Guide
------------------

1. Open terminal and run the following command.

   .. code-block:: bash

      roscd sitl_tutorials/ && cd ..
      ./run_sim.bash

   Enter the number of drones when prompted. You will see the drones spawning automatically in Gazebo amd a mission control UI popping up.

The mission control has the  folling features:
   1. Arm all the drones.
   2. Setting user defined configuration in run time.
   3. Command the drones to form shapes based on an image selected by the user.
   4. Safety check before execuiting the mission.

**Note:**
The drone swarm uses a `ORCA  <https://gamma.cs.unc.edu/ORCA/>`_ based decentrolised collission avoidance alogorithm.

Demo
-----

.. raw:: html

   <iframe width="700" height="400" src="https://www.youtube.com/embed/PH_vu_SqwpU?si=fSjG__Zh51kaApee" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


