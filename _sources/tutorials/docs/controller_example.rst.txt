.. _controller_example:

Using a planning controller
***************************

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/fAEGySqefwo?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

- `Overview`_
- `Tutorial Steps`_

Overview
========

In the previous examples, we have started the knowledge of PlanSys2, and we have calculated and executed plans interactively using the terminal. Obviously, when PlanSys2 is embedded in an application, interactive execution is not convenient.

A *planning controller* is a program that:
* Initiate, consult, and update knowledge.
* Set goals.
* Make requests to execute the plan.

This program controls the robot's mission. It is usually implemented as finite state machines and decides what the next goal to be achieved is.

In this tutorial, we will integrate PlanSys2 and Nav2 to make a robot patrol its environment. There are 5 waypoints: ``wp_control``, ``wp_1``, ``wp_2``, ``wp_3``, add ``wp_4``. Each one has different coordinates.

* ``wp_control`` is the junction to which all other waypoints are connected. A patrol always goes through wp_control, since the waypoints are not connected to each other.
* Patrolling a waypoint consists of moving to the waypoint and, once there, rotating for a few seconds to perceive the environment.
* During a patrol, all waypoints will be patrolled. Once patrolled, the patrol will begin again.


In the tutorial's first steps, we will use a test component that simulates the navigation process, and at the 
end of the tutorial, we will launch the real Nav2 and a simulator.

Tutorial Steps
==============

0- Requisites
-------------

If you haven't done yet, clone in your workspace and build the |PN| `examples <https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples>`_

  .. code-block:: bash

      cd <your_workspace>
      git clone -b <ros2-distro>-devel https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git src
      colcon build --symlink-install
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install

1- Running the example
----------------------

* Open a new terminal and run |PN| with a node that simulates de action of navigation. So, the execution of Nav2 will not take place in the first steps of this tutorial:

   .. code-block:: bash

      ros2 launch plansys2_patrol_navigation_example patrol_example_fakesim_launch.py

* Open a new terminal and run:
   
   .. code-block:: bash

      ros2 run plansys2_patrol_navigation_example patrolling_controller_node

This will start the mission. You will see in the first terminal how the plans are calculated and how the actions are executed. In the last terminal you will see the execution of the actions too. 

2- Patrol action performer
--------------------------

The action ``patrol`` (``ros2_planning_system_examples/plansys2_patrol_navigation_example/src/patrol_action_node.cpp``) contains the actions that makes the robot spin. 

  .. code-block:: c++
      :linenos:

       class Patrol : public plansys2::ActionExecutorClient
       {
       public:
         Patrol()
         : plansys2::ActionExecutorClient("patrol", 1s)
         {
         }
       
         rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         on_activate(const rclcpp_lifecycle::State & previous_state)
         {
           progress_ = 0.0;
       
           cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
           cmd_vel_pub_->on_activate();
       
           return ActionExecutorClient::on_activate(previous_state);
         }
       
         rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         on_deactivate(const rclcpp_lifecycle::State & previous_state)
         {
           cmd_vel_pub_->on_deactivate();
       
           return ActionExecutorClient::on_deactivate(previous_state);
         }
       
       private:
         void do_work()
         {
           if (progress_ < 1.0) {
             progress_ += 0.1;
       
             send_feedback(progress_, "Patrol running");
       
             geometry_msgs::msg::Twist cmd;
             cmd.linear.x = 0.0;
             cmd.linear.y = 0.0;
             cmd.linear.z = 0.0;
             cmd.angular.x = 0.0;
             cmd.angular.y = 0.0;
             cmd.angular.z = 0.5;
       
             cmd_vel_pub_->publish(cmd);
           } else {
             geometry_msgs::msg::Twist cmd;
             cmd.linear.x = 0.0;
             cmd.linear.y = 0.0;
             cmd.linear.z = 0.0;
             cmd.angular.x = 0.0;
             cmd.angular.y = 0.0;
             cmd.angular.z = 0.0;
       
             cmd_vel_pub_->publish(cmd);
       
             finish(true, 1.0, "Patrol completed");
           }
         }
       
         float progress_;
       
         rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
       };
       
       int main(int argc, char ** argv)
       {
         rclcpp::init(argc, argv);
         auto node = std::make_shared<Patrol>();
       
         node->set_parameter(rclcpp::Parameter("action", "patrol"));
         node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
       
         rclcpp::spin(node->get_node_base_interface());
       
         rclcpp::shutdown();
       
         return 0;
       }

This action runs with a frequency of 1Hz when activated. In each step, it increases its progress by 10% (line 32), sending speed commands to the robot through the 
topic ``/cmd_vel`` that make it spin (lines 36-44). When it completes the action, it stops the robot (lines 47-54). In each cycle it sends a feedback (line 34) 
or declares that the action has already finished (line 56).

3- Move action performer
------------------------

The action ``move`` (``ros2_planning_system_examples/plansys2_patrol_navigation_example/src/move_action_node.cpp``) contains the actions sends navigation requests to Nav2 using the ROS2 action ``navigate_to_pose``. This
example is quite complex if you are not familiar with `ROS2 actions <https://index.ros.org/doc/ros2/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client/>`_, so we will not enter in details here, only selected pieces of code.


  .. code-block:: c++
      :linenos:

        MoveAction()
        : plansys2::ActionExecutorClient("move", 500ms)
        {
          geometry_msgs::msg::PoseStamped wp;
          wp.header.frame_id = "/map";
          wp.pose.position.x = 0.0;
          wp.pose.position.y = -2.0;
          wp.pose.position.z = 0.0;
          wp.pose.orientation.x = 0.0;
          wp.pose.orientation.y = 0.0;
          wp.pose.orientation.z = 0.0;
          wp.pose.orientation.w = 1.0;
          waypoints_["wp1"] = wp;
      
          wp.pose.position.x = 1.8;
          wp.pose.position.y = 0.0;
          waypoints_["wp2"] = wp;
      
          wp.pose.position.x = 0.0;
          wp.pose.position.y = 2.0;
          waypoints_["wp3"] = wp;
      
          wp.pose.position.x = -0.5;
          wp.pose.position.y = -0.5;
          waypoints_["wp4"] = wp;
      
          wp.pose.position.x = -2.0;
          wp.pose.position.y = -0.4;
          waypoints_["wp_control"] = wp;
      
          using namespace std::placeholders;
          pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            10,
            std::bind(&MoveAction::current_pos_callback, this, _1));
        }

        ... 
        private:
          std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

The coordinates of each waypoint are initialized and inserted in ``waypoints_``.


  .. code-block:: c++
      :linenos:

       rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
       on_activate(const rclcpp_lifecycle::State & previous_state)
       {
         ...
         
         auto wp_to_navigate = get_arguments()[2];
         goal_pos_ = waypoints_[wp_to_navigate];
         navigation_goal_.pose = goal_pos_;

         ...

         send_goal_options.feedback_callback = [this](
           NavigationGoalHandle::SharedPtr,
           NavigationFeedback feedback) {
             send_feedback(
               std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
               "Move running");
           };

         send_goal_options.result_callback = [this](auto) {
             finish(true, 1.0, "Move completed");
           };
        
        ...
       }

* If we execute the action ``(move leia wp_control wp_1)``, the third argument (``get_arguments()[2]``) is the waypoint to navigate, ``wp_1``. We use this id to get the coordinate from ``waypoints_`` for sending it in the Nav2 action.
* When receiving feedback from the Nav2 ROS2 action, we send feedback about the execution of the ``move`` action.
* When Nav2 consider the navigation complete, we call ``finish`` to finalize the execution of ``move`` action.

3- Mission controller
---------------------

The mission controller (``ros2_planning_system_examples/plansys2_patrol_navigation_example/src/patrolling_controller_node.cpp``) init the knowledge in PlanSys2, and implements a FSM to run patrolling plans. It uses 
two PlanSys2 clients to interact with PlanSys2:

  .. code-block:: c++
         
         void init()
           {
             problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
             executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
             init_knowledge();
           }

         private: 
           std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
           std::shared_ptr<plansys2::ExecutorClient> executor_client_;

Using these clients, we can add instances and predicates to PlanSys2:

  .. code-block:: c++

       void init_knowledge()
       {
         problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
         problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
         problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
         problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
         problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
         problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});
     
         problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp1)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp_control)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp2)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp_control)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp3)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp_control)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp4)"));
         problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp_control)"));
       }

In the ``step`` method (called at 5Hz) there is the implementation of the FSM. Each switch case contains:
* The code to execute in the state

  .. code-block:: c++

       case PATROL_WP1:
         {
           auto feedback = executor_client_->getFeedBack();
 
           for (const auto & action_feedback : feedback.action_execution_status) {
             std::cout << "[" << action_feedback.action << " " <<
               action_feedback.completion * 100.0 << "%]";
           }

* The condition to transitate to other state, and the code executed before start the new state. The important part here is how we set the new goal for the new state (using ``setGoal``), and how we call the executor to calculate and execute a plan to achieve it (using the non-blocking call ``executePlan``):

  .. code-block:: c++

       if (executor_client_->getResult().value().success) {
         std::cout << "Plan successful finished " << std::endl;
         
         // Cleanning up
         problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp1)"));
         
         // Set the goal for next state, and execute plan
         
         problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp2))"));
         if (executor_client_->executePlan()) {
           state_ = PATROL_WP2;
         }
       } else {
         ...
       }

4- Running the example with Nav2
--------------------------------

* Make sure that Nav2 is `correctly installed and it is working <https://navigation.ros.org/getting_started/index.html>`_.

* Open a new terminal and run |PN| with a node that simulates de action of navigation. So, the execution of Nav2 will not take place in the first steps of this tutorial:

   .. code-block:: bash

      ros2 launch plansys2_patrol_navigation_example patrol_example_launch.py

* Open a new terminal and run:
   
   .. code-block:: bash

      ros2 run plansys2_patrol_navigation_example patrolling_controller_node