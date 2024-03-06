Starting the grasping pipeline
==============================

This page will tell you how to start the grasping pipeline. It will also explain how to use the tmux session that is created when starting the grasping pipeline. It will also explain how to start the grasping-pipeline rviz visualization and which visualization topics exist.

******************************
Starting the grasping pipeline
******************************
If you set up the grasping pipeline according to the installation instructions, you shoult be able to start the grasping pipeline by running the following command in the terminal:

.. code-block:: console

    $ gp

.. note:: 
   If you are using the Docker container make sure to start the container first:

   .. code-block:: console

       user@host         $ hsr
       root@CONTAINER_ID $ gp 


This should open up a tmux session.

===================
Tmux session layout
===================
This section will explain the windows and panes of the tmux session that is created when starting the grasping pipeline.


==================
Rviz visualization
==================
This section will explain how to start the grasping-pipeline rviz visualization.
All grasping-pipeline related visualization topics will be listed and explained.

Start the grasping-pipeline rviz visualization by running the following command in the terminal:

.. code-block:: console

    $ rv

This should open up a new rviz window with visualization customized for using the grasping-pipeline.


