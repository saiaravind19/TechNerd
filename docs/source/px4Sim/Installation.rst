Installation
===================================

Minimum Requirements
-----------------------------------

Before you begin the setting up PX4 simulation , please make sure your system meets the minimum requirements.

- Install ROS 1 (Robot Operating System) desktop version, we recommend using the Noetic version. For detailed installation instructions, please refer to the `ROS installation guide <https://wiki.ros.org/ROS/Installation>`_.


Download the PX4 Source Code 
------------------------------------------------------


Clone `PX4-Autopilot <https://github.com/PX4/PX4-Autopilot>`_.

.. code-block:: bash

   git clone https://github.com/PX4/PX4-Autopilot.git --recursive 

This command will clone the repository along with its submodules.

Install Dependencies
----------------------------------


- Install PX4 dependencies for ubuntu

.. code-block:: bash
      
      cd PX4-Autopilot
      sudo ./Tools/setup/ubuntu.sh  

- Install Mavros Dependencies and Geographiclib 

.. code-block:: bash
      
   sudo apt-get update
   sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
   sudo apt install geographiclib-tools
 
- Install RVO dependencies

.. code-block:: bash

   git clone https://github.com/snape/RVO2-3D.git
   cd RVO2-3D  && mkdir build && cd build
   cmake .. && sudo make install

Building PX4 for Simulation
-----------------------------------

Now lets builds PX4 along with all available Gazebo models.

.. code-block:: bash

   cd PX4-Autopilot
   # Doesnt start Gazebo sim and  build all avaialble gazebo models
   DONT_RUN=1 make px4_sitl gazebo-classic

You can also build specific model using the below commands.

.. code-block:: bash

   # Start Gazebo Classic with iris
   make px4_sitl gazebo-classic_iris

   # Start Gazebo Classic with plane
   make px4_sitl gazebo-classic_plane

   #start Gazebo Classic with typhoon_h480
   make px4_sitl gazebo-classic_typhoon_h480

For more info refere `PX4 Gazebo classic <https://docs.px4.io/main/en/sim_gazebo_classic/>`_


Setting up ROS Workspace
-----------------------------------
Open a terminal in your home directory

.. code-block:: bash

   mkdir -p ros_ws/src && cd ~/ros_ws/src
   git clone https://github.com/saiaravind19/px4_sitl_tutorials.git
   cd ~/ros_ws/ && catkin_make

Set gazebo path for Simulation
------------------------------------

Add the follwing to your bashrc file.

.. code-block:: bash

   source ~/ros_ws/devel/setup.bash
   export PX4=~/PX4-Autopilot
   source $PX4/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4 $PX4/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
   # Set path to sitl_gazebo repository
   export SITL_GAZEBO_PATH=$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic


``Note`` Check if the path is set correctly using 'rospack find mavlink_sitl_gazebo'
