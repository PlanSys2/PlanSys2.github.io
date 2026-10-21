.. _getting_started:

Getting Started
###############

This page installs |PN| and runs its first example: a robot that plans and executes how to go
from one room to another, recharging its battery on the way.

Installation
************

.. only:: not latest

   1. Install ROS 2 as described in the `official documentation <https://docs.ros.org>`_.
   2. Install |PN|:

      .. code-block:: bash

         sudo apt install 'ros-<distro>-plansys2-*'

      .. note::

         4.0.0 reaches the binaries of each distribution with its next sync: until then, ``apt``
         installs the previous release. To use 4.0.0 now, build it from source as
         :ref:`build-instructions` explains.

   3. Clone the `examples <https://github.com/PlanSys2/ros2_planning_system_examples>`_ in a
      workspace and build them:

      .. code-block:: bash

         mkdir -p ~/plansys2_ws/src
         cd ~/plansys2_ws/src
         git clone -b <distro> https://github.com/PlanSys2/ros2_planning_system_examples.git
         cd ~/plansys2_ws
         rosdep install -y -r -q --from-paths src --ignore-src
         colcon build --symlink-install

.. only:: latest

   Build |PN| and its examples from source as :ref:`build-instructions` explains.

Running the example
*******************

1. In a terminal, start |PN| with the domain of the example and the processes that perform its
   actions (simulated: each one takes the time the plan gives it):

   .. code-block:: bash

      source ~/plansys2_ws/install/setup.bash
      ros2 launch plansys2_simple_example plansys2_simple_example.launch.yaml

2. In another terminal, open the |PN| terminal:

   .. code-block:: bash

      source ~/plansys2_ws/install/setup.bash
      ros2 run plansys2_terminal plansys2_terminal

3. Load the problem of the example: its rooms, the robot ``leia`` at the entrance with low
   battery, and the goal of being in the bathroom. The commands are in the ``commands`` file
   of the example, which ``source`` runs:

   .. code-block:: text

      source <path to plansys2_simple_example>/share/plansys2_simple_example/launch/commands

   ``ros2 pkg prefix plansys2_simple_example`` tells the path. The file contains commands such as:

   .. code-block:: text

      set instance leia robot
      set instance entrance room
      set predicate (connected entrance dinning)
      set predicate (battery_low leia)
      set predicate (robot_at leia entrance)
      set goal (and(robot_at leia bathroom))

4. To watch the plan run, open the TUI in a third terminal (it does not run on Humble: see
   :ref:`migration`):

   .. code-block:: bash

      source ~/plansys2_ws/install/setup.bash
      ros2 run plansys2_tui_cli plansys2_tui

   Its *Plan Monitor* panel shows the plan being executed and the progress of each action,
   *Action Execution* which performer runs each action, and *Knowledge* how the problem changes.
   The *Plan log* tab (key ``2``) tells, as they happen, what the executor does and why.

5. In the |PN| terminal, ask for a plan, and run it:

   .. code-block:: text

      get plan
      run

   The terminal shows the progress of each action, and ``Successfully finished`` at the end;
   the TUI shows the same, action by action.

What next
*********

* :ref:`terminal_usage` explains every command of the terminal.
* The other tools show what |PN| knows and does while a plan runs: the TUI with its plan log
  (:ref:`tools`), and the :ref:`execution_trace`.
* :ref:`tutorials` build robot behaviors step by step.
