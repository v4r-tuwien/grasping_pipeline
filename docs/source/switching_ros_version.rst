Switching between ROS1 and ROS2
===============================

.. note:: Sasha always boots in ros1 environment.

When switching ros environment, it is not enough to just start a different docker container, the arduino on the HSRB also has to be reprogrammed.

In the home directory of sasha (user v4r) is a folder called `scripts` containing the scripts to switch between the ros version.

.. warning:: The switching scripts (both ways) require sudo privileges and might ask you to enter the password.

**************
Switch to ROS2
**************

To switch from ros1 (noetic) to ros2 (humble) use the script `ros2.sh`. 

This script stops all the ros1 system services, programs the arduino and starts the ros2 docker containers. After the script finished, it is required to press and release the e-stop on sasha. The robot should then say: *Sasha2 start*.

Currently, ros2 runs exclusively in docker containers and not native on the robot. This is done to preserve the ros1 environment. As soon as ros1 is no longer needed at all, a clean-install with a ros2 image can be performed.

.. note:: Make sure to switch back to ros1 (program arduino) before turning off sasha, otherwise the next person will wonder why the ros1 environment does not start correctly.

**************
Switch to ROS1
**************

To switch from ros2 to ros1 (noetic) use the script `ros1.sh`. 

This script stops all ros2 docker containers, programs the arduino and starts the ros1 system services. After the script finished, it is required to press and release the e-stop on sasha. The robot should then say: *Sasha start*.

***************
Troubleshooting
***************

If either ros1 or ros2 environment cannot start, the most likely reason it that the arduino was not programmed correctly. The easiest way to fix this, is to just run the switching script again, as this programs the arduino.

If you want to manually program the arduino, the following commands can be used:

.. warning:: If any other process access the arduino as well, the flashing command will fail.

for **ros1**:

.. code-block:: console

    $ sudo avrdude -C /home/v4r/arduino/avrdude.conf -p atmega328p -P /dev/ttyUSB_IO -c arduino -b 115200 -D -U flash:w:/home/v4r/arduino/arduino_firm_ros1.hex:i

for **ros2**:

.. code-block:: console

    $ sudo avrdude -C /home/v4r/arduino/avrdude.conf -p atmega328p -P /dev/ttyUSB_IO -c arduino -b 115200 -D -U flash:w:/home/v4r/arduino/arduino_firm_ros2.hex:i


