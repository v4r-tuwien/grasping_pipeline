Installation
============

You can install the grasping pipeline in two ways:

1. Using the grasping pipeline docker image

2. Manually installing the grasping pipeline and all its dependencies

****************************************
Using the grasping pipeline docker image
****************************************

This is the easiest way to get started with the grasping pipeline, but comes with the drawback that it does not run natively on the host. This especially means that you are more likely to experience issues regarding the network setup with ROS.

The instructions can be found in the `HSRB_ROS_Docker_Image repository <https://github.com/v4r-tuwien/HSRB-ROS-Docker-Image>`_.

.. note::
   After installation you might want to add the following alias to your .bashrc file to make it easier to start the docker container:

   .. code-block:: console

      $ echo "alias hsr='cd ~/HSR/ && bash ./RUN-DOCKER-CONTAINER.bash'" >> ~/.bashrc

   This allows you to start the docker container by simply typing `hsr` in the terminal.

   After adding the alias, source the new .bashrc file:

   .. code-block:: console

       $ source ~/.bashrc

******************************************************************
Manually installing the grasping pipeline and all its dependencies
******************************************************************
This option assumes that you already have installed:

* ROS noetic and the most common ROS packages (ros-noetic-desktop-full)
* The toyota HSR packages (ros-noetic-tmc-desktop-full)
* moveit (ros-noetic-moveit)

If you have not installed these packages yet, please refer to the commands in the **Dockerfile of the HSRB_ROS_Docker_Image repository** (`Link <https://github.com/v4r-tuwien/HSRB-ROS-Docker-Image/blob/main/docker/hsr-devel/Dockerfile>`_) on how to install ROS, the toyota HSR packages and moveit. If possible, use the versions specified in the Dockerfile.

.. warning::
   You will need access to the private v4r github repositories, because some of the repositories include confidential data from toyota. This means that you have to setup your github ssh-key (`Link for instructions <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_)

==========================================
Cloning all grasping pipeline repositories
==========================================

After installing the ROS dependencies and setting up the ssh key, you can finally clone all necessary grasping pipeline repositories into your catkin workspace:

.. code-block:: console

    $ cd ~/catkin_ws/src
    $ curl -s https://raw.githubusercontent.com/v4r-tuwien/grasping_pipeline/main/scripts/clone_grasping_pipeline.bash | bash

==================================
Installing the python dependencies
==================================
The grasping pipeline is written in python3 and uses several python packages. The dependenciesare listed in the *requirements.txt* file.

.. note:: If you want to install the dependencies in a virtual environment, you have to modify the launch files found in *./grasping_pipeline/launch*:

  .. code-block:: console

      <arg name="venv" value="/path/to/venv/bin/python3" />
      <node> pkg="pkg" type="node.py" name="node" launch-prefix = "$(arg venv)" />

To install the dependencies (either in the virtual environment or system-wide):

.. code-block:: console

    $ cd ~/catkin_ws/src/grasping_pipeline
    $ pip install -r requirements.txt

===============
Helpful aliases
===============
It is recommended to add the following aliases to your .bashrc file to make it easier to use the grasping pipeline:

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

    $ cd ~/catkin_ws
    $ catkin build

If you encounter an error while building because some packages are missing, please look at the error messages and try to install the missing packages using apt-get or pip and notify one of the roadies of this issue.

