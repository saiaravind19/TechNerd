Sorting System
==================

This project is a proof-of-concept for using Autonomous Mobile Robots (AMRs) to automate sorting tasks in warehouses and industrial environments. It integrates navigation, communication, and intelligent decision-making using ROS2, Python, and Rust.

Overview
-------------
The AMR Sorting System demonstrates:
    - Autonomous navigation between sorting stations
    - Real-time path planning and obstacle avoidance
    - Item pick-and-place operations
    - Scalable fleet management

Architecture & Components
-----------------------------

.. code-block:: text

   +-------------------+         +---------------------+         +-------------------+
   |                   |         |                     |         |                   |
   |   Fleet (ROS2)    +<------->+ Communication Hub   +<------->+   Delivery Hub    |
   |  (Multiple robots)|  gRPC/  |    (Rust, Modbus)   |  Modbus |   (Python, GUI)   |
   |  Navigation, Path | Modbus  |  Status, Messaging  |         |   Bin Management  |
   |  Planning, Tasks  |         |                     |         |                   |
   +-------------------+         +---------------------+         +-------------------+

   - Fleet: Multiple robots managed by ROS2, each with navigation, path planning, and task management.
   - Communication Hub: Central service (Rust) for robot-to-hub communication, status, and package acceptance.
   - Delivery Hub: Simulated PLC (Python) with Modbus server and GUI for bin/package management.

Sequence Diagram of the System
-------------------------------

.. image:: /docs/source/images/sequence_diagram.svg
     :alt: State Machine Diagram
     :width: 500px
     :align: center


Task Manager
---------------------------------
A simple state machine based task manager is implemented to handle different tasks in the fleet.

- **State Machines:**
  - Used for basic task sequencing, but can become complex as the system grows.

- **Behaviour Trees:**
  - Provide modular, scalable, and maintainable logic for advanced robot behaviors
  - Enable easier debugging and extension.

Here is a simple state machine diagram implemented in the project:

.. image:: /docs/source/images/state_machine.png
     :alt: State Machine Diagram
     :width: 500px
     :align: center

Path Planner
---------------

A-star based path planner is used with a predefined map of the environment.


Demo 
--------------------

.. raw:: html
    
    <iframe width="700" height="400" src="https://www.youtube.com/embed/eQAzZhX_X8I?si=TXh9pjiVkXVm8A27" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


For more details, code, and design notes, visit the `GitHub Repository <https://github.com/saiaravind19/sorting-system>`_.
