.. _hddl_support:

HDDL support
############

.. note::

   HDDL mode is in the development version of |PN| (*latest*): it is not in 4.0.0.

In PDDL, a problem says what must hold at the end: the goal. In HDDL, the language of
hierarchical (HTN) planning, a problem says what is to be done: a network of *tasks*, such as
``(build car_1)``. The domain says how each task can be done: its *methods*, each a network of
smaller tasks, down to the actions. A plan is then the actions *and* their decomposition: which
method was applied to each task, and what each of its subtasks became.

This gives a robot's mission a structure that a goal does not have:

* The mission is the tasks, not a state: "build the three cars, one after another" is said as
  it is meant, and the planner only looks at the ways the domain allows.
* Each method has a precondition, checked when the robot gets there: if the world is no longer
  as when the plan was made, the task fails there, before any of its actions start, and the
  execution trace says which task and why.
* The plan can be followed by task, in the terminal, in the TUI and in Groot.

|PN| runs HDDL in the same way as PDDL: the same experts, the same performers (actions are
executed as always), the same controllers, with a task network where a PDDL problem has a goal.
The planner is `Klimax <https://github.com/PlanSys2/klimax>`_, an HTN planner for HDDL 1.0 and
HDDL 2.1.

HDDL mode
*********

|PN| plans and executes HDDL when both of these hold, and refuses to start when only one does:

* the model files are ``.hddl`` (and the problem file, if any);
* the executor builds its trees with the ``HTNBTBuilder``.

.. code-block:: bash

   ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
     model_file:=house.hddl problem_file:=house_problem.hddl bt_builder_plugin:=HTNBTBuilder

The bringup launch files then set Klimax as the planner by themselves. In that mode:

* The domain expert keeps the tasks and methods of the domain as written, for the planner, and
  reads its actions as those of any PDDL domain.
* The problem expert keeps the task network of the problem (its ``:htn`` section), and writes it
  in the problem it gives the planner. Instances, predicates and functions are as always.
* The planner is Klimax: its plan has the actions in ``items``, as any other, and the
  decomposition in ``tasks`` (``plansys2_msgs/PlanTask``).
* The executor runs the plan by its decomposition (see below).

Klimax is built from source with |PN|: it is in ``dependency_repos.repos``, on the modules of
Kairos.

What Klimax plans
*****************

* HDDL 1.0: totally and partially ordered task networks, method preconditions, constraints
  between variables, the conditions and effects of ADL.
* HDDL 2.1: numeric fluents and a metric, durative actions and durative methods, timed initial
  literals, orderings between the starts and the ends of tasks, constraints on durations and on
  the states of the plan; and object fluents.
* It proves when a problem has no plan. With its ``anytime`` parameter it goes on after the
  first plan to better ones (shorter, or cheaper by the metric).
* Plans with durative actions come with times: actions overlap where the hierarchy and what
  they read and write allow. Among equally good ways, the work goes to whoever is free first (a
  robot, a machine), so that plans run in parallel.

The `semantics <https://github.com/PlanSys2/klimax/blob/main/docs/semantics.md>`_ of Klimax says
what it takes where HDDL 2.1 is silent, including what executing a plan means, and its
`roadmap <https://github.com/PlanSys2/klimax/blob/main/docs/roadmap.md>`_ what it does not plan
yet.

The task network
****************

What is to be done is set as the goal is in PDDL, in any of these ways:

* In the problem file, its ``:htn`` section:

  .. code-block:: lisp

     (:htn :parameters ()
       :ordered-subtasks (and (t1 (build car_1)) (t2 (build car_2))))

* From a controller, with ``ProblemExpertClient::setTaskNetwork``. It replaces the network before;
  each task must be a task or an action of the domain, with arguments of its types.
  ``plansys2::TaskNetwork`` is made from an ``:htn`` section, or a task at a time, each after the
  ones it is given:

  .. code-block:: c++

     plansys2::TaskNetwork tasks;
     auto first = tasks.add("(build car_1)");
     tasks.add("(build car_2)", first);  // after the first
     problem_expert->setTaskNetwork(tasks);

  ``getTaskNetwork`` and ``clearTaskNetwork`` read and remove it. ``DomainExpertClient::getTasks``
  and ``getTask`` give the tasks of the domain and their parameters.

* From Python, ``set_problem_task_network`` (the message or a list of tasks),
  ``get_problem_task_network`` and ``remove_problem_task_network``; ``get_domain_tasks`` in the
  domain client.

* From the terminal:

  .. code-block:: text

     > set tasks (:htn :ordered-subtasks (and (t1 (build car_1)) (t2 (build car_2))))
     > set tasks (build car_1) (build car_2)
     > get problem tasks
     > get model tasks
     > remove tasks

  Tasks given alone, without ``:htn``, are not ordered among themselves.

The problem written for the planner (``getProblem``) has the network as its ``:htn`` section, and
the knowledge that the problem expert publishes (``problem_expert/knowledge``) has it in its
``task_network``.

How a hierarchical plan is executed
***********************************

The ``HTNBTBuilder`` makes the behavior tree of a plan from its decomposition, not from a graph of
its actions:

* Every task is a ``Sequence`` named after the task and its method, ``(build car_1) ← build_car``.
  It succeeds when all its subtasks have, and fails, failing those above it, when one of them
  fails.
* Its first node, ``CheckMethodPrecondition``, checks the precondition of the method in the state
  where the plan applied it: once the actions that had ended by then have ended. If it does not
  hold, the decomposition no longer applies and the task fails before any of its actions start.
* The subtasks run in the order the method gives them: a ``Sequence`` where it orders them all, a
  ``Parallel`` where it does not.
* The actions are the leaves, with the action template of the other builders. Each waits for the
  actions that end before it starts in the plan, and for the checks of the methods applied before
  it (``WaitMethodCheck``): what the methods leave open is done as the planner found it, at once
  where the plan overlaps actions.

A failed check is in the execution trace (see :ref:`execution_trace`) with the task, the method
and the part of the precondition that does not hold:

.. code-block:: text

   FAILURE (fetch steering_wheel_3 assembly_zone): the precondition of its method fetch_go does
   not hold: (not (robot_at bb8 steering_wheels_zone))

Replanning
**********

``executor/get_replanning_problem`` gives, for a hierarchical plan, the problem of what is left:
its task network is the frontier of the plan's tree. The tasks of which no action has ended go
whole (the planner may decompose them again); of those done in part, their subtasks; the actions
still running, as tasks. Each keeps the orderings of the methods above it. The actions that go on
are kept, as in PDDL: the new plan does not start them again.

Following a plan
****************

* ``get plan`` in the terminal shows its actions; the TUI shows it as its decomposition: in the
  *Plan Monitor*, each task with its method and its state (done, running, failed, pending, from
  those of its actions), its subtasks below it. Its *Knowledge* panel shows the task network.
* Groot2, and the *Behavior tree* tab of the TUI, show the tree of the decomposition as it runs:
  which task is running, which method checks passed (see :ref:`tools`).

Known limits
************

* Only Klimax plans HDDL: the other planners do not take it, and Klimax does not take PDDL.
* A method precondition is checked against what the problem expert knows, as any condition in
  |PN|: what the robot perceives counts once it is in the knowledge.

The tutorial :ref:`hddl_example` runs the factory of ``plansys2_bt_example`` in HDDL, from the
terminal and from a controller.
