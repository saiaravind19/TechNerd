Installation
===================================

Minimum Requirements
-----------------------------------

Before proceeding with the installation, ensure your system meets the minimum requirements.
Install ROS 1 (Robot Operating System), with Noetic being the preferred version. You can follow the official ROS installation guide `here <https://wiki.ros.org/ROS/Installation>`_.
Preferred ros noetic and Ubuntu 20.04

Install Dependencies
Install mavros and mavlink packages using apt

.. code-block:: bash
   
   sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras


Download the PX4 Source Code
-----------------------------------

The PX4 source code is hosted on GitHub in the `PX4/PX4-Autopilot repository <https://github.com/PX4/PX4-Autopilot>`_.

To fetch the latest version onto your computer, open a terminal and run the following command:

.. code-block:: bash

   git clone https://github.com/PX4/PX4-Autopilot.git --recursive

This command will clone the repository along with its submodules.

Building PX4 for Simulation
-----------------------------------

To build PX4 for simulation with Gazebo Classic, follow these commands:

.. code-block:: bash

   cd PX4-Autopilot
   make px4_sitl gazebo_classic-(name of the model)

Replace ``(name of the model)`` with the specific Gazebo model you intend to use. This will compile PX4 for simulation using Gazebo Classic.

Download QGroundControl
-----------------------------------

To control and monitor your PX4 simulation, you need a Ground Control Station (GCS). Download QGroundControl from `http://qgroundcontrol.com/downloads/ <http://qgroundcontrol.com/downloads/>`_ and follow the installation instructions for your operating system.

Now you're ready to take control of your PX4 simulation with QGroundControl!
