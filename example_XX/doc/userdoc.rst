:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_XX/doc/userdoc.rst

.. _ros2_control_demos_example_xx_userdoc:

Example xx: Cart-Pole
=================================

With *example_xx*, we demonstrate the a robot with passive joints.

.. note::

  Follow the installation instructions on :ref:`ros2_control_demos_install` how to install all dependencies,
  Gazebo Classic should be automatically installed.

  * If you have installed and compiled this repository locally, you can directly use the commands below.
  * If you have installed it via the provided docker image: To run the first two steps of this example (without Gazebo Classic), use the commands as described with :ref:`ros2_control_demos_install`. To run the later steps using Gazebo Classic, execute

    .. code::

      docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_xx cart_pole.launch.py gui:=false

    first. Then on your local machine you can run the Gazebo Classic client with

    .. code-block:: shell

      gzclient

    and/or ``rviz2`` with

    .. code-block:: shell

      rviz2 -d src/ros2_control_demos/ros2_control_demo_description/cart_pole/rviz/cart_pole.rviz


  For details on the ``gazebo_ros2_control`` plugin, see :ref:`gazebo_ros2_control`.

Tutorial steps
--------------------------

The following example shows a cart with a pendulum arm.

.. code-block:: shell

  ros2 launch ros2_control_demo_example_xx cart_pole.launch.py
  ros2 run ros2_control_demo_example_xx trajectory_action_client.py


Files used for this demos
-------------------------

- Launch files:

  + View robot: `view_robot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/cart_pole/launch/view_robot.launch.py>`__
  + Gazebo Classic: `cart_pole.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_XX/bringup/launch/cart_pole.launch.py>`__

- Controllers yaml: `cart_pole_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_XX/bringup/config/cart_pole_controllers.yaml>`__
- URDF file:

  + Main File: `cart_pole.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_XX/description/urdf/cart_pole.urdf.xacro>`__
  + Description: `cart_pole_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/cart_pole/urdf/cart_pole_description.urdf.xacro>`__
  + ``ros2_control`` tag: `cart_pole.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_XX/description/ros2_control/cart_pole.ros2_control.xacro>`__
  + ``gazebo`` tag: `cart_pole.gazebo.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_XX/description/gazebo/cart_pole.gazebo.xacro>`__

- RViz configuration: `cart_pole.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/cart_pole/rviz/cart_pole.rviz>`__
- Action client: `trajectory_action_client.py  <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_XX/scripts/trajectory_action_client.py >`__


Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__

* ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__
