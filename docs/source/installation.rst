Installation
============

To install the grasping pipeline and all its dependencies, you can use the following commands:

.. code-block:: console

    $ cd ~/catkin_ws/src
    $ git clone 
    $ bash ~/catkin_ws/src/grasping_pipeline/src/install_dependencies.sh

Add alias into your .bashrc file to make it easier to use the grasping pipeline:

.. code-block:: console

    $ echo "alias gp='bash ~/catkin_ws/src/grasping_pipeline/src/pipeline_bringup.sh'" >> ~/.bashrc

Add alias for rviz with config file for visualizing stuff from the grasping pipeline:

.. code-block:: console

    $ echo "alias rv='rviz -d ~/catkin_ws/src/grasping_pipeline/config/grasping_pipeline.rviz'" >> ~/.bashrc

Then, source your .bashrc file:

.. code-block:: console

    $ source ~/.bashrc

Finally, build the workspace:

.. code-block:: console

    $ cd ~/catkin_ws
    $ catkin build