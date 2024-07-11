Building a UV Sterilization Robot
=================================

The UV-sterilization bot project aims to create an affordable, modular autonomous robot for diverse applications, with a current focus on sterilizing hospital wards and ICUs using UV light. Developed amid the COVID-19 pandemic, this open-source initiative promotes community collaboration to help mitigate virus spread in various environments.

Installation
------------

1. Install `ESP-IDF <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html>`_ and necessary dependencies based on your development environment.
2. Install `V-Rep <https://www.coppeliarobotics.com/>`_ if you want to simulate the environment and test out the algorithm.
3. Clone the repository, and you are ready to go.

.. code-block:: bash

    git clone https://github.com/tech-igloo/Semi-autonomous-UV-sterilization-bot.git

Architecture
------------

The project uses an ESP32-based microcontroller for hosting an interactive UI and controlling the robot based on user input. With a dual-core CPU, the entire software stack is divided into two main parts: UI and Algorithm & Control. Both of these tasks are assigned to a core for seamless operation.

.. image:: /docs/source/images/software_architecture.png
   :alt: software_architecture
   :width: 800px
   :height: 400px

For a detailed explanation about the algorithm and hardware implentation , refer to the `github wiki <https://github.com/tech-igloo/Semi-autonomous-UV-sterilization-bot/wiki/Introduction>`_.

Simulation
----------

.. raw:: html

    <iframe width="700" height="400" src="https://www.youtube.com/embed/0HbukC9cVcU?si=AE--tATQyW2MRrB8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Demo
----

.. raw:: html

    <iframe width="700" height="400" src="https://www.youtube.com/embed/8vxNBiGw6P0?si=H7ATdnG28POtGZlM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>