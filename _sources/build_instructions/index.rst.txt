.. _build-instructions:

Build and Install
#################

Install
*******

PlanSys2 and its dependencies are released as binaries.
You may install it via the following to get the latest stable released version:

  ``sudo apt install ros-<distro>-plansys2_*``


Build
*****

Install ROS
-----------

Please install ROS 2 via the usual `build instructions <https://index.ros.org/doc/ros2/Installation>`_ for your desired distribution.

Build PlanSys2
--------------

Create a new workspace, ``plansys2_ws``, and clone PlanSys2 master branch into it and build it. 
Optionally, you can also include the TFD plan solver (you should `install TFD first <https://github.com/IntelligentRoboticsLabs/plansys2_tfd_plan_solver>`_) and the examples.


.. code:: bash

  mkdir -p ~/plansys2_ws/src
  cd ~/plansys2_ws/src
  git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git
  
  # Optional
  git clone https://github.com/IntelligentRoboticsLabs/plansys2_tfd_plan_solver.git
  git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git
  
  cd ~/plansys2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
  colcon build --symlink-install

* If you don't want to use ``master`` branch, you can use ``foxy-devel`` branch instead.
* if your are using ``eloquent``, you should clone the ``eloquent-devel`` of the first two repositories (no TFD available for this distro).
