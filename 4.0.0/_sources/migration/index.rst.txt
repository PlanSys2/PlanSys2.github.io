.. _migration:

Migrating to 4.0.0
##################

|PN| 4.0.0 changes some interfaces. This page lists what an application must change.

.. contents:: Contents
   :local:
   :depth: 1

The clients take the node of the application
********************************************

Before, each client created its own node (two in Python). Now a client creates no node: it works
on the node it is given. Only the constructors change; the methods are the same.

C++:

.. code-block:: c++

   // Before
   domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
   executor_client_ = std::make_shared<plansys2::ExecutorClient>();

   // Now, in a node of the application (rclcpp::Node or rclcpp_lifecycle::LifecycleNode)
   domain_expert_ = std::make_shared<plansys2::DomainExpertClient>(*this);
   executor_client_ = std::make_shared<plansys2::ExecutorClient>(*this);

   // Or with the shared pointer of a node
   auto planner_client = std::make_shared<plansys2::PlannerClient>(node);

* The constructor takes the node (``*this`` works in the constructor of the node too). The same
  applies to ``DomainExpertClient``, ``ProblemExpertClient``, ``PlannerClient`` and
  ``ExecutorClient``.
* The client does not own the node: keep the node alive while the client is used.
* The services and actions are relative to the namespace of the node, as before they were to the
  namespace of the client's node.
* The clients bring their own answers: their services, actions and subscriptions go in a
  callback group of their own, which the executor of the node does not take, and a client
  serves it only while it waits. The application spins its node as it likes, or not at all: the
  calls work from ``main`` before any spin, and from a callback of any executor, single threaded
  too. See `Spinning the node of a controller`_.
* Every call waits until its component is active, however long it takes: an application no
  longer needs a wait of its own for |PN| (the examples had one), and a call no longer fails
  after some seconds because |PN| is not there yet. ``wait_active()`` waits alone.
* A request that can be repeated (queries, and changes that leave the same state) is sent up to
  three times in all when it gets no answer, with a new client after the first: a call without
  answer takes about three seconds to give up, instead of one.
* ``ExecutorClient`` only has constructors that take a node; the one with a node name is gone.
* ``PlannerClient`` declares ``plan_solver_timeout`` on the node, unless it is declared already.

Python:

.. code-block:: python

   # Before
   domain = DomainExpertClient()
   executor = ExecutorClient('my_executor_client', namespace='robot1')

   # Now
   node = rclpy.create_node('my_controller', namespace='robot1')
   domain = DomainExpertClient(node)
   executor = ExecutorClient(node)

* The clients are not nodes any more: ``get_name()``, ``create_timer()`` or ``destroy_node()``
  are methods of the node, not of the client. ``client.node`` is the node of the application.
* rclpy executors take whole nodes, not callback groups: each client has a node of its own,
  ``<node>_<component>_client`` in the namespace of the node of the application, which it serves
  while it waits. ``client.destroy()`` destroys it. As in C++, the application spins its node as
  it likes, or not at all.

Spinning the node of a controller
*********************************

The clients do not depend on how the node of the application is spun. A controller that has
callbacks of its own (subscriptions, timers, services) spins its node as any ROS application:

.. code-block:: c++

   // A loop of its own, as in 3.x
   rclcpp::executors::SingleThreadedExecutor exec;
   exec.add_node(node);
   while (rclcpp::ok() && node->step()) {  // step() calls the clients
     exec.spin_some();                     // the subscriptions of the controller
     rate.sleep();
   }

   // Everything in callbacks: step() in a timer
   sub_ = create_subscription<Battery>("battery", 10, [this](Battery::SharedPtr msg) {...});
   timer_ = create_wall_timer(200ms, [this] {step();});
   ...
   rclcpp::spin(node);  // a single thread is enough

A controller without callbacks of its own needs no executor: ``main`` calls the clients.

As with any blocking call in ROS, a call from a callback keeps its thread while it waits: with a
single thread, the messages of the controller wait in their queue until the call ends. If the
controller must react to them during a long call (``getPlan()``), spin the node with a
``MultiThreadedExecutor`` and put those callbacks in another callback group, or call from the
loop of ``main``.

The lifecycle manager
*********************

``plansys2::LifecycleServiceClient`` is not a node any more. It takes a ``plansys2::ClientNode``
on the node of the manager and the name of the managed node, and ``init()`` is gone:

.. code-block:: c++

   auto manager = rclcpp::Node::make_shared("lifecycle_manager");
   auto client_node = std::make_shared<plansys2::ClientNode>(*manager);
   auto planner = std::make_shared<plansys2::LifecycleServiceClient>(client_node, "planner");
   // Nothing to spin: as any client, it brings its answers while it waits

The bringup
***********

* The monolithic bringup has no ``plansys2`` node and no ``*_lc_mngr`` nodes: |PN| has four
  nodes, ``domain_expert``, ``problem_expert``, ``planner`` and ``executor``. In the distributed
  bringup, ``lifecycle_manager`` keeps watching them once they are active (``supervise: false``
  in its section of the parameters file makes it exit, as before).
* Both bringups start again a component process that ends (``respawn``, ``true`` by default; ``respawn:=false``
  as before), and give the problem expert a ``state_file`` of their own launch, so that it comes
  back with its problem. Applications that launch the nodes themselves get none of this unless
  they set it.
* A plan waits up to ``component_timeout`` seconds (30) for a component that is not active, and
  then fails, saying which one, instead of waiting for ever.
* ``use_real_time`` and ``startup_timeout`` of the monolithic bringup are not parameters in the
  ``plansys2`` section of the parameters file any more, but launch arguments
  (``use_real_time:=true startup_timeout:=10``). Running ``plansys2_node`` directly, they are
  the arguments ``--use-real-time true`` and ``--startup-timeout 10``.
* ``startup_timeout`` of the distributed bringup is still a parameter of ``lifecycle_manager``.

BT builders
***********

``BTBuilder::domain_client()`` and ``problem_client()`` do not create clients any more: a BT
builder needs its clients (``set_clients``) before it builds a tree, and throws
``std::logic_error`` without them. The executor sets them; only a BT builder used on its own
needs the call.

Executors
*********

``plansys2_node`` and the main of each node of the distributed bringup spin each node with a
``MultiThreadedExecutor``: a callback of a component may wait for another component (the domain
expert for the planner, when it validates a domain) while other threads serve the rest. Code that
embeds the |PN| nodes in its own process must do the same. ``plansys2::testing::ExecutorThread``
now calls ``spin()``, so a multi-threaded executor uses all its threads.

Kairos, the default planner
***************************

The planner uses `Kairos <https://github.com/PlanSys2/kairos>`_ by default, instead of POPF.
Kairos supports more of PDDL, so some domains that failed now plan, and the plans may differ from
the ones POPF gave. To keep POPF, set the plan solver of the planner to it, as
:ref:`plugins` explains.

C++20
*****

|PN| needs C++20 (3.1.0 asked for C++23): code that includes its headers builds with C++20 or newer.

Humble
******

The TUI (``plansys2_tui``) does not run on Humble: the Textual it uses needs newer ``rich``,
``typing_extensions`` and ``platformdirs`` than Ubuntu 22.04 has, and it says so when it
starts. The rest of |PN| is the same as in the other distributions.
