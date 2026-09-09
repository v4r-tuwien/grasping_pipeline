Installation
============

.. note:: This only covers the installation of the ros2 environment. For the installation of the ros1 environment please see the ros1 branch of this repository

You can install the grasping pipeline in two ways:

1. Using the grasping pipeline docker image

2. Manually installing the grasping pipeline and all its dependencies

.. note:: 
    Sasha uses `ROS_DOMAIN_ID=1`, to talk to the robot you also need to set `ROS_DOMAIN_ID=1`. When using the simulator make sure to use a different `ROS_DOMAIN_ID` to avoid accidentally sending commands to the robot.

    When using a private laptop instead of *robbie* or *raufbold* it is necessary to add the ip address of the laptop to the cyclone_dds configuration on the robot. The configuration file is located at `/etc/opt/tmc/robot/cyclonedds_profile.xml`. The file should look like this:

    .. code-block:: xml

        <CycloneDDS>
            <Domain id="any">
                <Internal>
                    <SocketReceiveBufferSize min="10MB"/>
                    <Watermarks>
                        <WhcHigh>500kB</WhcHigh>
                    </Watermarks>
                </Internal>
                <General>
                    <AllowMulticast>false</AllowMulticast>
                    <MaxMessageSize>1500B</MaxMessageSize>
                </General>
                <Discovery>
                    <ParticipantIndex>auto</ParticipantIndex>
                    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
                    <Peers>
                        <Peer Address="10.0.0.102"/>
                        <Peer Address="10.0.0.143"/>
                        <Peer Address="10.0.0.221"/>
                        <Peer Address="ADD YOUR IP ADDRESS HERE"/>
                    </Peers>
                </Discovery>
            </Domain>
        </CycloneDDS>

    After changing the dds configuration all ros2 nodes need to be restarted.


****************************************
Using the grasping pipeline docker image
****************************************

This is the easiest way to get started with the grasping pipeline, but comes with the drawback that it does not run natively on the host. This especially means that you are more likely to experience issues regarding the network setup with ROS.

.. note:: The network passthrough with docker and cyclonedds only works on native linux host environments. It does not work on windows using wsl2.

The instructions can be found in the `HSRB_ROS_Docker_Image repository <https://github.com/v4r-tuwien/HSRB-ROS-Docker-Image>`_.

.. note::
   After installation you might want to add the following alias to your .bashrc file to make it easier to start the docker container:

   .. code-block:: console

      $ echo "alias hsr2='cd ~/HSR2/ && bash ./RUN-DOCKER-CONTAINER.bash'" >> ~/.bashrc

   This allows you to start the docker container by simply typing `hsr2` in the terminal.

   After adding the alias, source the new .bashrc file:

   .. code-block:: console

       $ source ~/.bashrc


*********************
Network configuration
*********************

.. note:: These settings are required when using the docker container and when installing manually

.. warning:: These are system settings and require sudo permissions.

In order for our cyclone_dds configuration to work correctly the networking buffer sizes need to be increased.

To permanently change these settings, add a file to /etc/sysctl.d/

.. code-block:: console

    $ sudo nano /etc/sysctl.d/99-custom.conf

The file should have the following content:

.. code-block:: text

    # network settings for ros2 cyclone dds
    net.core.rmem_max=2147483647
    net.core.rmem_default=2147483647
    net.core.wmem_max=2147483647
    net.core.wmem_default=2147483647
    net.ipv4.ipfrag_time=3
    net.ipv4.ipfrag_high_thresh=134217728

The immediately apply these settings use

.. code-block:: console

    $ sudo sysctl --system


******************************************************************
Manually installing the grasping pipeline and all its dependencies
******************************************************************
This option assumes that you already have installed:

* ROS2 humble and the most common ROS packages (ros-humble-desktop)
* ROS2 development tools (ros-dev-tools)
* CycloneDDS (ros-humble-rmw-cyclonedds-cpp)
* moveit (ros-humble-moveit)

If you have not installed these packages yet, please refer to the commands in the **Dockerfile of the HSRB_ROS2_Docker_Image repository** (`Link <https://github.com/v4r-tuwien/HSRB-ROS2-Docker-Image/blob/main/docker/Dockerfile>`_) on how to install ROS2, the toyota HSR packages and moveit. If possible, use the versions specified in the Dockerfile.

.. warning::
   You will need access to the private v4r github repositories, because some of the repositories include confidential data from toyota. This means that you have to setup your github ssh-key (`Link for instructions <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_)

=========================
Install HSRB dependencies
=========================

Before creating the grasping pipeline the HSRB dependencies need to be installed.

See the Dockerfile of the HSRB_ROS2_Docker_Image repository for how to install the moveit commander and the hsrb dependencies.

===========================
Creating a colcon workspace
===========================

We recommend to create a new colcon workspace for the grasping pipeline. You can do so with the following commands:

.. code-block:: console

    $ mkdir -p ~/colcon_ws/src
    $ cd ~/colcon_ws
    $ colcon build

==========================================
Cloning all grasping pipeline repositories
==========================================

After installing the ROS dependencies and setting up the ssh key, you can finally clone all necessary grasping pipeline repositories into your catkin workspace:

.. TODO this script needs to be checked and tested

.. code-block:: console

    $ cd ~/catkin_ws/src
    $ curl -s https://raw.githubusercontent.com/v4r-tuwien/grasping_pipeline/main/scripts/clone_grasping_pipeline.bash | bash

==================================
Installing the python dependencies
==================================

.. TODO this needs to be checked and tested

The grasping pipeline is written in python3 and uses several python packages. The dependencies are listed in the *requirements.txt* file.

.. note:: If you want to install the dependencies in a virtual environment, you have to modify the launch files found in *./grasping_pipeline/launch*:

  .. code-block:: console

      <arg name="venv" value="/path/to/venv/bin/python3" />
      <node> pkg="pkg" type="node.py" name="node" launch-prefix = "$(arg venv)" />

To install the dependencies (either in the virtual environment or system-wide):

.. code-block:: console

    $ cd ~/catkin_ws/src/grasping_pipeline
    $ pip install -r requirements.txt

.. note::
    If you encounter an error while installing the dependencies, you might need to update your pip 
    version:
    
    .. code-block:: console
    
        $ pip install --upgrade pip==22.3.1 --user
    
    This installs the new pip version in the ``~/.local/bin`` directory. Make sure to add this directory to your PATH variable in your .bashrc file:

    .. code-block:: console

        $ echo "export PATH=$PATH:/home/INSERT_USERNAME/.local/bin" >> ~/.bashrc
    
    Afterwards, source the .bashrc file:

    .. code-block:: console

        $ source ~/.bashrc



===============
Helpful aliases
===============
It is recommended to add the following aliases to your .bashrc file to make it easier to use the grasping pipeline.
The aliases make it possible to start the grasping pipeline and rviz by simply typing `gp` and `rv` in the terminal.

------------------------------------------------
Add an alias for starting the grasping pipeline:
------------------------------------------------

.. code-block:: console

    $ echo "alias gp='bash ~/catkin_ws/src/grasping_pipeline/src/pipeline_bringup.sh'" >> ~/.bashrc

This allows you to start the grasping pipeline by simply typing `gp` in the terminal.

--------------------------------------------------------------------------------------------
Add alias for starting rviz with a configuration file customized for the grasping pipeline
--------------------------------------------------------------------------------------------

.. code-block:: console

    $ echo "alias rv='rviz -d ~/catkin_ws/src/grasping_pipeline/config/grasping_pipeline.rviz'" >> ~/.bashrc

This allows you to start rviz with the grasping pipeline configuration by simply typing `rv` in the terminal.


==========================
Building the ROS workspace
==========================

After adding the aliases, source the new .bashrc file:

.. code-block:: console

    $ source ~/.bashrc

Finally, build the workspace:

.. code-block:: console

    $ cd ~/colcon_ws
    $ colcon build

If you encounter an error while building because some packages are missing, please look at the error messages and try to install the missing packages using apt-get or pip and notify one of the roadies of this issue.

After building the workspace, you can source the setup.bash file:

.. code-block:: console

    $ cd ~/colcon_ws
    $ source install/setup.bash
