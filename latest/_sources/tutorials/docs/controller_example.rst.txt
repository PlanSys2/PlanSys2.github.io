.. _controller_example:

Using a planning controller
***************************

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/fAEGySqefwo" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

- `Overview`_
- `Tutorial Steps`_

Overview
========

In the previous examples, we have started the knowledge of PlanSys2, and we have calculated and executed plans interactively using the terminal. Obviously, when PlanSys2 is embedded in an application, interactive execution is not convenient.

A *planning controller* is a program that:
* Initiates, consults, and updates knowledge.
* Sets goals.
* Makes requests to execute the plan.

This program controls the robot's mission. It is usually implemented as a finite state machine and decides what the next goal to be achieved is.

In this tutorial, we will integrate PlanSys2 and `EasyNav <https://easynavigation.github.io>`_ to make a robot patrol its environment. There are 5 waypoints: ``wp_control``, ``wp1``, ``wp2``, ``wp3``, and ``wp4``. Each one has different coordinates.

* ``wp_control`` is the junction to which all other waypoints are connected. A patrol always goes through wp_control, since the waypoints are not connected to each other.
* Patrolling a waypoint consists of moving to the waypoint and, once there, rotating for a few seconds to perceive the environment.
* During a patrol, all waypoints will be patrolled. Once patrolled, the patrol will begin again.

In the tutorial's first steps, we will use a node that simulates EasyNav (each navigation takes 10 s), and at the
end of the tutorial, we will launch the real EasyNav and a simulator. The example needs EasyNav: without it, the
navigation action is not built.

Tutorial Steps
==============

0- Requisites
-------------

If you haven't done yet, clone in your workspace and build the |PN| `examples <https://github.com/PlanSys2/ros2_planning_system_examples>`_

  .. code-block:: bash

      cd <your_workspace>
      git clone -b <ros2-distro> https://github.com/PlanSys2/ros2_planning_system_examples.git src/ros2_planning_system_examples
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install

1- Running the example
----------------------

* Open a new terminal and run |PN| with the action performers and a node that simulates EasyNav:

   .. code-block:: bash

      ros2 launch plansys2_patrol_navigation_example patrol_example_fakesim.launch.yaml

* Open a new terminal and run the controller:

   .. code-block:: bash

      ros2 run plansys2_patrol_navigation_example patrolling_controller_node

This will start the mission. You will see in the first terminal how the plans are calculated and how the actions are executed. In the last terminal you will see the execution of the actions too, and every 2 s the navigation feedback of EasyNav.

2- Patrol action performer
--------------------------

The action ``patrol`` (``plansys2_patrol_navigation_example/src/patrol_action_node.cpp``) makes the robot spin.

  .. code-block:: c++

      class Patrol : public plansys2::ActionExecutorClient
      {
      public:
        Patrol()
        : plansys2::ActionExecutorClient("patrol")
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
          // As long as the plan says it lasts
          progress_ += 1.0 / std::max(1.0, get_planned_duration() * get_parameter("rate").as_double());
          if (progress_ < 1.0 - 1e-3) {
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

        node->set_parameter(rclcpp::Parameter("action_name", "patrol"));
        node->set_parameter(rclcpp::Parameter("rate", 1.0));
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

        rclcpp::spin(node->get_node_base_interface());

        rclcpp::shutdown();

        return 0;
      }

The interesting part is ``do_work()``: it publishes a rotation speed while the action runs, and stops the robot when it
finishes. Its progress grows at each call so that the action lasts as long as the plan gives it
(``get_planned_duration()``), at the rate of the node (1 Hz, set in ``main``). ``on_activate`` and ``on_deactivate``
activate and deactivate the publisher, as in any lifecycle node, and then call those of ``ActionExecutorClient``.

3- Move action performer
------------------------

The action ``move`` (``plansys2_patrol_navigation_example/src/move_action_node.cpp``) navigates with EasyNav, through
its ``easynav::GoalManagerClient``. The coordinates of each waypoint come from parameters (``waypoints`` and
``waypoint_coords.<waypoint>``), which ``params/waypoints_fakesim.yaml`` sets:

  .. code-block:: c++

        MoveAction()
        : plansys2::ActionExecutorClient("move")
        {
          // GoalManagerClient needs a plain node; without global arguments, so that the
          // node name given in the launch does not apply to it
          nav_node_ = rclcpp::Node::make_shared(
            std::string(get_name()) + "_nav", get_namespace(),
            rclcpp::NodeOptions().use_global_arguments(false));
          navigation_ = easynav::GoalManagerClient::make_shared(nav_node_);
          // Only the goal manager publishes it: is_connected() also counts other clients
          manager_info_ = nav_node_->create_subscription<easynav_interfaces::msg::GoalManagerInfo>(
            "easynav_manager_info", 10, [](easynav_interfaces::msg::GoalManagerInfo::UniquePtr) {});

          // EasyNav may start late: wait for it this long before failing the action
          server_timeout_ = declare_parameter("server_timeout", 30.0);
          auto wp_names = declare_parameter("waypoints", std::vector<std::string>{});

          for (const auto & wp : wp_names) {
            auto coords = declare_parameter("waypoint_coords." + wp, std::vector<double>{});
            if (coords.size() < 2) {
              RCLCPP_ERROR(get_logger(), "Waypoint [%s] needs [x, y] or [x, y, yaw]", wp.c_str());
              continue;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = coords[0];
            pose.pose.position.y = coords[1];
            const double yaw = coords.size() > 2 ? coords[2] : 0.0;
            pose.pose.orientation.z = sin(yaw / 2.0);
            pose.pose.orientation.w = cos(yaw / 2.0);
            waypoints_[wp] = pose;
          }
        }

When the action is activated, ``on_activate`` takes the destination, the third argument of the action
(``(move r2d2 wp_control wp1)``), and looks up its coordinates. ``do_work()`` sends the goal once EasyNav is there,
waiting up to ``server_timeout`` seconds for it, and then follows the navigation: it sends its progress as feedback
and finishes when EasyNav does:

  .. code-block:: c++

        void do_work()
        {
          if (!goal_pos_) {
            finish(false, 0.0, "Unknown waypoint");
            return;
          }

          const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - phase_start_).count();

          if (!goal_sent_) {
            if (manager_info_->get_publisher_count() > 0 && navigation_->is_connected()) {
              navigation_->send_goal(*goal_pos_);
              goal_sent_ = true;
              phase_start_ = std::chrono::steady_clock::now();
            } else if (elapsed > server_timeout_) {
              finish(false, 0.0, "No EasyNav goal manager in easynav_control");
            } else {
              RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for EasyNav...");
            }
            return;
          }

          switch (navigation_->get_state()) {
            case State::SENT_GOAL:
              if (elapsed > server_timeout_) {
                renew_client();
                finish(false, 0.0, "EasyNav did not answer the navigation goal");
              }
              break;
            case State::SENT_PREEMPT:
            case State::ACCEPTED_AND_NAVIGATING:
              {
                const double distance = navigation_->get_feedback().distance_to_goal;
                if (initial_distance_ < 0.0 && distance > 0.0) {
                  initial_distance_ = distance;
                }
                const double progress = initial_distance_ > 0.0 ?
                  1.0 - distance / initial_distance_ : 0.0;
                send_feedback(std::clamp(progress, 0.0, 1.0), "Move running");
              }
              break;
            case State::NAVIGATION_FINISHED:
              navigation_->reset();
              finish(true, 1.0, "Move completed");
              break;
            default:
              navigation_->reset();
              finish(false, 0.0, "Navigation failed");
          }
        }

4- Mission controller
---------------------

The controller (``plansys2_patrol_navigation_example/src/patrolling_controller_node.cpp``) is a node that creates
the |PN| clients on itself, sets the knowledge, and runs a finite state machine that patrols each waypoint in turn:

  .. code-block:: c++

        void init()
        {
          domain_expert_ = std::make_shared<plansys2::DomainExpertClient>(*this);
          planner_client_ = std::make_shared<plansys2::PlannerClient>(*this);
          problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(*this);
          executor_client_ = std::make_shared<plansys2::ExecutorClient>(*this);
          init_knowledge();
        }

Each state checks the execution of the current plan, and when it ends, sets the next goal, plans and executes the new
plan:

  .. code-block:: c++

            case PATROL_WP1:
              {
                auto feedback = executor_client_->getFeedBack();

                for (const auto & action_feedback : feedback.action_execution_status) {
                  std::cout << "[" << action_feedback.action << " " <<
                    action_feedback.completion * 100.0 << "%]";
                }
                std::cout << std::endl;

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                  switch (executor_client_->getResult().value().result) {
                    case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
                      {
                        // Cleanning up
                        problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp1)"));

                        // Set the goal for next state
                        problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp2))"));
                      }
                      break;
                    case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
                      RCLCPP_INFO(get_logger(), "Plan preempted");
                      break;
                    case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
                      RCLCPP_ERROR(get_logger(), "Plan finished with error. Retrying");
                      for (const auto & action_feedback : feedback.action_execution_status) {
                        if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                          RCLCPP_ERROR_STREAM(
                            get_logger(), "[" << action_feedback.action <<
                              "] finished with error: " << action_feedback.message_status);
                        }
                      }
                      break;
                  }

                  // Compute the plan
                  auto domain = domain_expert_->getDomain();
                  auto problem = problem_expert_->getProblem();
                  auto plan = planner_client_->getPlan(domain, problem);

                  if (!plan.has_value()) {
                    std::cout << "Could not find plan to reach goal " <<
                      parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                    break;
                  }

                  // Execute the plan
                  if (executor_client_->getResult().value().result ==
                    plansys2_msgs::action::ExecutePlan::Result::SUCCESS &&
                    executor_client_->start_plan_execution(plan.value()))
                  {
                    state_ = PATROL_WP2;
                  }
                }
              }
              break;

``main`` calls ``step()`` at 5 Hz. The |PN| clients bring their own answers, so the controller spins its node only
for its own subscription, the navigation feedback of EasyNav, which it shows every 2 s:

  .. code-block:: c++

      int main(int argc, char ** argv)
      {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<PatrollingController>();

        node->init();

        // The clients bring their own answers: the executor is for the subscription of the controller
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);

        rclcpp::Rate rate(5);
        while (rclcpp::ok()) {
          node->step();
          exec.spin_some();
          rate.sleep();
        }

        rclcpp::shutdown();

        return 0;
      }

5- Running the example with EasyNav and a simulator
---------------------------------------------------

The example also runs with a simulated Kobuki in Gazebo, with the kobuki playground of
`easynav_playgrounds <https://github.com/EasyNavigation/easynav_playgrounds>`_ and the EasyNav plugins it uses.
All at once (Gazebo, EasyNav, |PN| and the controller; ``gui:=false`` runs Gazebo headless):

  .. code-block:: bash

      ros2 launch plansys2_patrol_navigation_example patrol_example_kobuki.launch.yaml

Or step by step, each in a terminal:

  .. code-block:: bash

      ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml
      ros2 launch plansys2_patrol_navigation_example patrol_example.launch.yaml
      ros2 run plansys2_patrol_navigation_example patrolling_controller_node

``patrol_example.launch.yaml`` takes ``waypoints_file`` to set the waypoints of another map.
