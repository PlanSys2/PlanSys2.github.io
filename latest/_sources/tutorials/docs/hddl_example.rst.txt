.. _hddl_example:

Hierarchical planning with HDDL
*******************************

- `Overview`_
- `Tutorial Steps`_

Overview
========

.. note::

   HDDL mode is in the development version of |PN| (*latest*): it is not in 4.0.0.

This tutorial runs the factory of ``plansys2_bt_example`` (see :ref:`bt_actions`) in HDDL. Three
robots, ``r2d2``, ``c3po`` and ``bb8``, bring the wheel, the body and the steering wheel of a car
to the assembly zone, and then one of them assembles it. In PDDL the problem says what must hold
at the end, ``(car_assembled car_1)``. In HDDL it says what is to be done, ``(build car_1)``, and
the domain says how a car is built. See :ref:`hddl_support` for what HDDL mode is.

Tutorial Steps
==============

0- Requisites
-------------

HDDL mode is not in the binaries yet: build |PN| from source (see :ref:`build-instructions`),
with the examples, in the same workspace. Klimax, the planner of HDDL mode, comes with
``dependency_repos.repos``.

1- The domain
-------------

``pddl/bt_example.hddl`` has the actions of ``pddl/bt_example.pddl`` (``move``, ``transport``,
``assemble``, ``recharge``), the same performers run them. On top of them, tasks and methods:

.. code-block:: lisp

   (:task build :parameters (?c - car))
   (:task fetch :parameters (?p - piece ?to - zone))
   (:task ready :parameters (?r - robot ?z - zone))

   (:method build_car
       :parameters (?c - car ?z - zone ?w ?b ?s - piece ?r - robot)
       :task (build ?c)
       :precondition (and
           (is_assembly_zone ?z)
           (piece_is_wheel ?w) (piece_is_body_car ?b) (piece_is_steering_wheel ?s)
           (piece_not_used ?w) (piece_not_used ?b) (piece_not_used ?s))
       :subtasks (and
           (t1 (fetch ?w ?z))
           (t2 (fetch ?b ?z))
           (t3 (fetch ?s ?z))
           (t4 (ready ?r ?z))
           (t5 (assemble ?r ?z ?w ?b ?s ?c)))
       :ordering (and (< t1 t5) (< t2 t5) (< t3 t5) (< t4 t5)))

To build a car, its three pieces are fetched and a robot gets ready, in any order, and then it
assembles the car. A piece is fetched by any robot: ``fetch_there`` if it is already where it
must be, ``fetch_carry`` if the robot is where the piece is, ``fetch_go`` if it has to go there
first, ``fetch_recharge`` if its battery must be charged before. The planner chooses the robots;
with durative actions, it gives the work to the robot that is free first, so the three pieces
come at once.

2- From the terminal
--------------------

* Open a terminal and run the node that simulates EasyNav (each navigation takes 10 s):

   .. code-block:: bash

      ros2 run plansys2_bt_example easynav_sim_node

* In a second terminal, run |PN| in HDDL mode. The launch file loads the domain, the problem of
  ``pddl/problem.hddl`` (one car) and the ``HTNBTBuilder``:

   .. code-block:: bash

      ros2 launch plansys2_bt_example plansys2_bt_example_hddl.launch.yaml

  Its log says ``HDDL mode`` and that the planner is ``KLIMAX``.

* In a third terminal, open the |PN| terminal:

   .. code-block:: bash

      ros2 run plansys2_terminal plansys2_terminal

   .. code-block:: text

      > get problem tasks
      > get plan
      > run

  ``get plan`` shows the three ``transport`` at time 0, each by its robot, and the assembly
  after them. ``set tasks`` and ``remove tasks`` change the task network before running it.

* To follow it by task, open the TUI in a fourth terminal (``ros2 run plansys2_tui_cli
  plansys2_tui``): the *Plan Monitor* shows the decomposition, and the *Behavior tree* tab, as
  Groot2 does on port 1800, the tree of tasks that runs.

3- From a controller
--------------------

``assemble_hddl_controller_node`` builds three cars, one after another. With the simulated
EasyNav and |PN| running as above, run it instead of the terminal:

.. code-block:: bash

   ros2 run plansys2_bt_example assemble_hddl_controller_node

It sets its own problem, in place of the one the launch file loads.

It is the controller of the PDDL version with a task network where that one sets a goal:

.. code-block:: c++

   bool init_knowledge()
   {
     // The problem is this controller's: none loaded by the launch is mixed in
     problem_expert_->clearKnowledge();
     problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
     // ... the robots, zones, pieces and cars, and where everything is

     // What is to be done: each car built, after the one before
     plansys2::TaskNetwork tasks;
     const auto first = tasks.add("(build car_1)");
     const auto second = tasks.add("(build car_2)", first);
     tasks.add("(build car_3)", second);
     if (!problem_expert_->setTaskNetwork(tasks)) {
       RCLCPP_ERROR(get_logger(), "The task network was not accepted");
       return false;
     }
     return true;
   }

Planning and executing are as in PDDL: ``getPlan`` with the domain and the problem, and
``start_plan_execution`` with the plan. In each car the three robots bring a piece at once and
``r2d2`` assembles it: 45 s of plan for the three cars.

4- When a method no longer applies
----------------------------------

The precondition of each method is checked when the plan gets there, in the state of then. If the
world changed, the task fails before any of its actions start, and the execution trace says why.
For instance, if a piece is moved by hand while the robots work, its ``fetch`` fails with the part
of the precondition that does not hold. The controller can then ask for a replanning problem
(``executor/get_replanning_problem``): its task network is what is left of the plan, and the new
plan keeps the actions that go on.

5- Other examples in HDDL
-------------------------

* ``plansys2_bt_example``: the factory with arms (``plansys2_bt_arm_example_hddl.launch.yaml``,
  ``assemble_arm_hddl_controller_node``).
* ``plansys2_patrol_navigation_example``: the patrols (``patrol_example_hddl.launch.yaml``,
  ``patrolling_hddl_controller_node`` and ``patrol_waypoints_hddl_controller_node``).
* ``plansys2_simple_example``, ``plansys2_simple_example_py``, ``plansys2_cascade_example`` and
  ``plansys2_multidomain_example``: each with its ``*_hddl.launch.yaml``.
* ``plansys2_combat_example``: the orders of each round as tasks (``combat_launch.py`` with ``hddl:=true``).
