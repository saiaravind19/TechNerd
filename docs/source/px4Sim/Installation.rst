Installation
===================================

Minimum Requirements
-----------------------------------

Before proceeding with the installation, ensure your system meets the minimum requirements.
Install ROS 1 (Robot Operating System), with Noetic being the preferred version.
You can follow the official ROS installation guide `here <https://wiki.ros.org/ROS/Installation>`_.

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
   DONT_RUN=1 make px4_sitl gazebo-classic


Replace ``(name of the model)`` with the specific Gazebo model you intend to use. This will compile PX4 for simulation using Gazebo Classic.


Download QGroundControl
-----------------------------------

To control and monitor your PX4 simulation, you need a Ground Control Station (GCS). Download `QGroundControl <http://qgroundcontrol.com/downloads/>`_ and follow the installation instructions for your operating system.

Now you're ready to take control of your PX4 simulation with QGroundControl!


Work space setup
-----------------------------------
Open a terminal in your home directory

.. code-block:: bash

   mkdir -p ros_ws/src && cd ~/ros_ws/src
   git clone https://github.com/saiaravind19/px4_sitl_tutorials.git
   cd ~/ros_ws/ && catkin_make

Set gazebo path for Simulation
------------------------------------

Add the follwing in your bashrc file 

.. code-block:: bash

   source ~/ros_ws/devel/setup.bash
   export PX4_HOME=~/PX4-Autopilot
   source $PX4_HOME/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_HOME $PX4_HOME/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_HOME
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic

   # Set the plugin path so Gazebo finds our model and sim
   export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic/build
   # Set the model path so Gazebo finds the airframes
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
   # Disable online model lookup since this is quite experimental and unstable
   export GAZEBO_MODEL_DATABASE_URI=""
   # Set path to sitl_gazebo repository
   export SITL_GAZEBO_PATH=$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic