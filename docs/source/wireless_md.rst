Building a Wireless Motor Driver
================================

Motor Drivers are essential components for robotics and hardware projects, allowing enthusiasts to control various actuators with ease. One popular motor driver is the L298N, but it typically requires a lot of wiring to connect to a microcontroller. In this tutorial, we'll show you how to build a wireless motor driver using the Wemos D1 Mini microcontroller, which simplifies the setup by combining everything into a single PCB.


.. image:: /docs/source/images/Robot.jpg
   :alt: Wireless motor driver
   :width: 400px
   :height: 240px


Why Wireless?
-------------

By integrating the Wemos D1 Mini, which features built-in WiFi functionality, our motor driver can be controlled wirelessly. This significantly reduces the time required to prototype projects and allows you to control your bots via a mobile device through WiFi.

.. image:: /docs/source/images/WirelessMD.jpg
   :alt: Wireless motor driver
   :width: 400px
   :height: 240px



Specifications
--------------

- **Supply Voltage**: 6 - 12V
- **Output Current**: 1.2A (max)
- **500mA Fuse Protection** for Wemos D1 Mini
- **Dual Functionality**: Can be used separately as a conventional L298N Motor Driver

Prerequisites
-------------

Make sure you install one of the `supported versions <https://www.arduino.cc/en/software/OldSoftwareReleases>`_ of Arduino IDE and have the ESP8266 core installed. The package is tested with Arduino IDE v1.8.5 and plugin v0.5.0, so it is recommended to use the same versions.

To install `SPIFFS files uploader <https://github.com/me-no-dev/arduino-esp32fs-plugin>`_, please follow the installation tutorials provided in the tutorial.

Once the plugin and Arduino IDE are installed successfully, move on to the next step of installation of the stack to the controller.

Installation
------------

1. Clone the repo to your system and open the project in Arduino IDE.

.. code-block:: bash

   https://github.com/saiaravind19/Wireless_Motor_Driver

.. note::
   Make sure the `data` folder is present in the current working directory.

2. Use SPIFFS files uploader to flash the HTML UI to ESP. To upload, navigate to `Tools > ESP8266 Sketch Data Upload`.

3. Wait for the files to upload. Once the files are uploaded, compile and upload the program.


Pinout Connections
------------------

.. list-table::
   :header-rows: 1

   * - Controller GPIO
     - Motor Driver Pin
   * - D1
     - EN1
   * - D5
     - IN1
   * - D6
     - IN2
   * - D2
     - EN2
   * - D7
     - IN3
   * - D8
     - IN4
   * - D3
     - L1-led
   * - D0
     - L2-led

Accessing the UI
----------------

If using ESP as an Access Point (AP mode), connect to the network SSID mentioned in the file **data/wifi/ap.txt** present in the working directory.

Open your favorite web browser and type **wirelessmd.local** to access the UI and start controlling the motors wirelessly.

Hardware
--------

All of the schematic and board layout files are present in the project folder. The PCB was designed using KiCAD 6.0. The board is based on the popular Dual Full Bridge Motor Driver L298N and the miniature wireless microcontroller development board Wemos D1 Mini. The PCB can output nearly 1.2A per channel. Since it has the Wemos D1 Mini on board, it gives the user the capability to control the motors wirelessly using our unique GUI.

- **WirelessMD/Hardware/KiCAD Project** - Contains the KiCAD Project File
- **WirelessMD/Hardware/WirelessMD_sch.pdf** - Schematic of Wireless Motor Driver PCB

Demo
--------

.. raw:: html

    <iframe width="700" height="400" src="https://www.youtube.com/embed/LfRNsaYup2E?si=tMpY9x5MIGhloilo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



