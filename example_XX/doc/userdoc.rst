:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_9/doc/userdoc.rst

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

      docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_9 rrbot_gazebo_classic.launch.py gui:=false

    first. Then on your local machine you can run the Gazebo Classic client with

    .. code-block:: shell

      gzclient

    and/or ``rviz2`` with

    .. code-block:: shell

      rviz2 -d src/ros2_control_demos/example_9/description/rviz/rrbot.rviz


  For details on the ``gazebo_ros2_control`` plugin, see :ref:`gazebo_ros2_control`.

Tutorial steps
--------------------------

The following example shows a cart with a pendulum arm. This uses the effort command interface for the cart's
degree of freedom on the rail, and the physics of the passive joint of the pendulum is solved correctly.

.. code-block:: shell

  ros2 launch gazebo_ros2_control_demos pendulum_example_effort.launch.py
  ros2 run gazebo_ros2_control_demos example_effort


Files used for this demos
-------------------------

- Launch files:

  + Hardware: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/launch/rrbot.launch.py>`__
  + Gazebo Classic: `rrbot_gazebo_classic.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/launch/rrbot_gazebo_classic.launch.py>`__

- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
- Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/config/rrbot_forward_position_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
