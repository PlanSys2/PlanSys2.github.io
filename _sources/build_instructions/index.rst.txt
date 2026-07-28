.. _build-instructions:

Build and Install
#################

Install
*******

PlanSys2 and its dependencies are released as binaries.
You may install it via the following to get the latest stable released version:

  ``sudo apt install ros-<distro>-plansys2-*``


Build
*****

Install ROS
-----------

Please install ROS 2 via the usual `build instructions <https://docs.ros.org/en/kilted/Installation.html>`_ for your desired distribution.

Build PlanSys2
--------------

Create a new workspace, ``plansys2_ws``, and clone the PlanSys2 branch matching your ROS 2 distribution (e.g. ``rolling``, ``kilted``, ``jazzy``, or ``humble-devel``) into it and build it.
Optionally, you can also include the TFD plan solver (you should `install TFD first <https://github.com/PlanSys2/plansys2_tfd_plan_solver>`_) and the examples.


.. code:: bash

  mkdir -p ~/plansys2_ws/src
  cd ~/plansys2_ws/src
  git clone -b <ros2-distro-branch> https://github.com/PlanSys2/ros2_planning_system.git

  # Optional
  git clone -b <ros2-distro-branch> https://github.com/PlanSys2/plansys2_tfd_plan_solver.git
  git clone -b <ros2-distro-branch> https://github.com/PlanSys2/ros2_planning_system_examples.git

  cd ~/plansys2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
  colcon build --symlink-install

* Replace ``<ros2-distro-branch>`` with ``rolling`` if you are tracking ROS 2 Rolling, or with ``<ros2-distro>-devel`` (e.g. ``kilted-devel``, ``jazzy-devel``, ``humble-devel``) for a stable distribution.
