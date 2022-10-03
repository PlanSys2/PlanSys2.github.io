.. _getting_started:

Getting Started
###############

This document will take you through the process of installing the |PN| binaries
and and using |PN|.

.. note::

  See the :ref:`build-instructions` for other situations such as building from source or
  working with other types of robots.

Installation
************

1. Install the `ROS 2 binary packages`_ as described in the official docs
2. Install the |PN| packages using your operating system's package manager:

   .. code-block:: bash

      sudo apt install ros-<ros2-distro>-plansys2-*

Running the Example
*******************

1. Clone in your workspace the |PN| `examples <https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples>`_

  .. code-block:: bash

      cd <your_workspace>
      git clone -b <ros2-distro>-devel https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git src
      colcon build --symlink-install
  
2. Build the workspace:

  .. code-block:: bash

      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install
  
3. Open a new terminal and run |PN|:

   .. code-block:: bash

      ros2 launch plansys2_simple_example plansys2_simple_example_launch.py

   This launch PlanSys2 this `simple PDDL domain <https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_simple_example/pddl/simple_example.pddl>`_,
   and some processes that implements a fake version of the domain's actions.

4. Open a new terminal and run the |PN| terminal:
   
   .. code-block:: bash

      ros2 run plansys2_terminal plansys2_terminal

5. In the |PN| terminal shell, copy and paste the next commands to init the knowledge of the system and set the goal:

   .. code-block:: lisp

      set instance leia robot
      set instance entrance room
      set instance kitchen room
      set instance bedroom room
      set instance dinning room
      set instance bathroom room
      set instance chargingroom room
      
      set predicate (connected entrance dinning)
      set predicate (connected dinning entrance)
      
      set predicate (connected dinning kitchen)
      set predicate (connected kitchen dinning)
      
      set predicate (connected dinning bedroom)
      set predicate (connected bedroom dinning)
      
      set predicate (connected bathroom bedroom)
      set predicate (connected bedroom bathroom)
      
      set predicate (connected chargingroom kitchen)
      set predicate (connected kitchen chargingroom)
      
      set predicate (charging_point_at chargingroom)
      set predicate (battery_low leia)
      set predicate (robot_at leia entrance)

      set goal (and(robot_at leia bathroom))
      
6. In the |PN| terminal shell, get the plan and/or run the plan:

   .. code-block:: lisp

      get plan
      run
