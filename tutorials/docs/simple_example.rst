.. _simple_example:

The first planning package
**************************

- `Overview`_
- `Tutorial Steps`_

Overview
========

It is common for PlanSys2 to have a package that contains all the elements to plan and execute in a domain:

* A PDDL model.
* A node that implements each action, probably in different executables.
* A launcher that executes the actions and launches PlanSys2 with the appropriate domain.
* Possibly (not in this tutorial) a node that initiates knowledge and manages when to execute a plan and its goals.


In this tutorial we are going to run again and to analyze the package of the simple example shown in :ref:`getting_started`.

Tutorial Steps
==============

0- Requisites
-------------

Clone in your workspace and build the |PN| `examples <https://github.com/PlanSys2/ros2_planning_system_examples>`_

  .. code-block:: bash

      cd <your_workspace>
      git clone -b <ros2-distro> https://github.com/PlanSys2/ros2_planning_system_examples.git src/ros2_planning_system_examples
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install

1- Running the example
----------------------

* Open a new terminal and run |PN|:

   .. code-block:: bash

      ros2 launch plansys2_simple_example plansys2_simple_example.launch.yaml

   This launch PlanSys2 this `simple PDDL domain <https://github.com/PlanSys2/ros2_planning_system_examples/blob/rolling/plansys2_simple_example/pddl/simple_example.pddl>`_,
   and some processes that implements a fake version of the domain's actions.

* Open a new terminal and run the |PN| terminal:
   
   .. code-block:: bash

      ros2 run plansys2_terminal plansys2_terminal

* In the |PN| terminal shell, copy and paste the next commands to init the knowledge of the system and set the goal:

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

* In the |PN| terminal shell, run the plan:

   .. code-block:: lisp

      run

In the PlanSys2 terminal you'll be able to see the plan. In both terminal, you'll see the current actions executed and its level of completion. As 
soon as the plan execution finished, the terminal will be available again. 

2- Package structure
--------------------

Go to ``<your_workspace>/src/ros2_planning_system_examples/plansys2_simple_example``. This is the ROS2 package that contains the example 
of this tutorial. The structure is the usual of a ROS2 package, with a ``package.xml`` and a ``CMakeLists.txt`` as usual. Besides, we have the 
next directories:

* **pddl** The directory with the PDDL file that contains the domain. It is the same of the :ref:`terminal_usage`.
* **launch** It contains the launcher of this example:
* **src** It contains the implementation of the three actions of the domain.

3- Actions implementation
-------------------------

The actions in the domain are *move*, *charge* and *askcharge*. It will contain a "fake" implementation. Each node that implements an action is called *action performer*. Each action will take some seconds
to execute, only incrementing a ``progress`` value and displaying it in the terminal. Each action in PlanSys2 is a 
`managed node <https://design.ros2.org/articles/node_lifecycle.html>`_ , also known as *lifecycle node*. When active, it will iteratively call 
a function to perform the job. Let's analyze the code of *move* action in ``src/move_action_node.cpp``:


  .. code-block:: c++
       :linenos:

       class MoveAction : public plansys2::ActionExecutorClient
       {
       public:
         MoveAction()
         : plansys2::ActionExecutorClient("move")
         {
           progress_ = 0.0;
         }

       private:
         void do_work()
         {
           // As long as the plan says it lasts
           progress_ += 1.0 / std::max(1.0, get_planned_duration() * get_parameter("rate").as_double());
           if (progress_ < 1.0 - 1e-3) {
             send_feedback(progress_, "Move running");
           } else {
             finish(true, 1.0, "Move completed");

             progress_ = 0.0;
             std::cout << std::endl;
           }

           std::cout << "\r\e[K" << std::flush;
           std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
             std::flush;
         }

         float progress_;
       };

       int main(int argc, char ** argv)
       {
         rclcpp::init(argc, argv);
         auto node = std::make_shared<MoveAction>();

         node->set_parameter(rclcpp::Parameter("action_name", "move"));
         node->set_parameter(rclcpp::Parameter("rate", 4.0));
         node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

         rclcpp::spin(node->get_node_base_interface());

         rclcpp::shutdown();

         return 0;
       }


* ``MoveAction`` is a ``plansys2::ActionExecutorClient`` (defined `here <https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/include/plansys2_executor/ActionExecutorClient.hpp>`_), that inherit from ``rclcpp_cascade_lifecycle::CascadeLifecycleNode``. This is, basically, a ``rclcpp_lifecycle::LifecycleNode``, but with an additional property: it can activate in cascade another ``rclcpp_cascade_lifecycle::CascadeLifecycleNode`` when it is active. It's useful when an action requires to activate a node that process a sensor information. It will only be active while the action that requires its output is active. See `this repo <https://github.com/fmrico/cascade_lifecycle>`_ for more info.

  .. code-block:: c++

       : plansys2::ActionExecutorClient("move")

By default, ``do_work()`` will be called every 200ms (5Hz). The ``rate`` ROS parameter (in Hz) changes it, without modifying the C++ code: it is read when the node is configured, so set it before, in the launcher or with ``set_parameter``, as this example does (4 Hz).

* ``plansys2::ActionExecutorClient`` has an API, with these relevant protected methods for the actions:

  .. code-block:: c++
       
       const std::vector<std::string> & get_arguments() const;
       double get_planned_duration() const;
       void send_feedback(float completion, const std::string & status = "");
       void finish(bool success, float completion, const std::string & status = "");

``get_arguments()`` returns the list of arguments of an action. For example, when executing ``(move leia chargingroom kitchen)``, it will return 
this vector of strings: ``{"leia", "chargingroom", "kitchen"}``. ``get_planned_duration()`` returns the duration the
plan gives the action: this example advances its progress so that it lasts as long.

This code is sending feedback of its completion. When finished, ``finish`` method indicates that the action has finished, and send back if 
it successfully finished, the completion value in [0-1] and optional info. Then, the node will pass to inactive state.

  .. code-block:: c++

           progress_ += 1.0 / std::max(1.0, get_planned_duration() * get_parameter("rate").as_double());
           if (progress_ < 1.0 - 1e-3) {
             send_feedback(progress_, "Move running");
           } else {
             finish(true, 1.0, "Move completed");
       
             progress_ = 0.0;
             std::cout << std::endl;
           }

* The action node, once created, must pass to inactive state to be ready to execute.

  .. code-block:: c++

       auto node = std::make_shared<MoveAction>();
     
       node->set_parameter(rclcpp::Parameter("action_name", "move"));
       node->set_parameter(rclcpp::Parameter("rate", 4.0));
       node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

       rclcpp::spin(node->get_node_base_interface());

The parameter ``action_name`` sets what action implements the ``node`` object. 

4- Launcher
-----------

The launcher (``launch/plansys2_simple_example.launch.yaml``) includes the PlanSys2 launcher, selecting the domain, and runs the executables that implement the PDDL actions:

  .. code-block:: yaml
       :linenos:

       launch:
       - arg:
           name: namespace
           default: ''
           description: Namespace
       - include:
           file: $(find-pkg-share plansys2_bringup)/launch/plansys2_bringup_launch_monolithic.py
           arg:
           - {name: model_file, value: $(find-pkg-share plansys2_simple_example)/pddl/simple_example.pddl}
           - {name: namespace, value: $(var namespace)}
       - node:
           pkg: plansys2_simple_example
           exec: move_action_node
           name: move_action_node
           namespace: $(var namespace)
           output: screen
       - node:
           pkg: plansys2_simple_example
           exec: charge_action_node
           name: charge_action_node
           namespace: $(var namespace)
           output: screen
       - node:
           pkg: plansys2_simple_example
           exec: ask_charge_action_node
           name: ask_charge_action_node
           namespace: $(var namespace)
           output: screen

