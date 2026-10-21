.. _plugins:

Plugins
#######

Planner plugins
***************

The planner node runs the plan solver plugins of its ``plan_solver_plugins`` parameter.
`Kairos`_ is the default: it plans every PDDL feature |PN| executes (see :ref:`pddl_support`).

A plugin may say which PDDL requirements it plans with. The planner gives a task only to the
plugins that support everything the domain and the problem use, and answers with the reason
when none does (for instance, a domain with ``:conditional-effects`` is not sent to POPF).
A plugin that says nothing gets every task.

+------------------+----------------------+------------------------------------------+
| Planner          | Plugin               | Highlights                               |
+==================+======================+==========================================+
| `Kairos`_        | `KairosPlanSolver`_  | Default. Everything |PN| executes:       |
|  (default)       |                      | ADL, numeric and object fluents,         |
|                  |                      | durative actions, timed initial literals |
|                  |                      | and fluents, derived predicates, PDDL+,  |
|                  |                      | constraints, preferences, metrics.       |
+------------------+----------------------+------------------------------------------+
| `POPF`_          | `POPFPlanSolver`_    | Numeric fluents; Durative actions;       |
|                  |                      | Timed initial literals.                  |
+------------------+----------------------+------------------------------------------+
| `TFD`_           | `TFDPlanSolver`_     | Numeric fluents; Durative actions.       |
+------------------+----------------------+------------------------------------------+
| `OPTIC`_         | `OPTICPlanSolver`_   | Numeric fluents; Durative actions;       |
|                  |                      | Metrics.                                 |
+------------------+----------------------+------------------------------------------+
| `Mips XXL`_      | `MIPSPlanSolver`_    | Numeric fluents; Durative actions;       |
|                  |                      | Metrics; Negative Preconditions.         |
+------------------+----------------------+------------------------------------------+
| `LPG-TD`_        | `LPGTDPlanSolver`_   | Numeric fluents; Durative actions;       |
|                  |                      | Metrics; Negative Preconditions;         |
|                  |                      | Disjunctive preconditions.               |
+------------------+----------------------+------------------------------------------+
| `Fast Downward`_ | `DownwardPlanner`_   | Derived predicates; Existential          |
|                  |                      | preconditions; Disjunctive preconditions |
+------------------+----------------------+------------------------------------------+
| `SymK`_          | `SymkPlanner`_       | Derived predicates; Existential          |
|                  |                      | preconditions; Disjunctive preconditions |
+------------------+----------------------+------------------------------------------+
| `UPF`_           | `UPFPlanSolver`_     | Unified Planning Framework               |
|                  |                      |                                          |
+------------------+----------------------+------------------------------------------+


.. _Kairos: https://github.com/PlanSys2/kairos
.. _KairosPlanSolver: https://github.com/PlanSys2/ros2_planning_system/tree/rolling/plansys2_kairos_plan_solver
.. _POPF: https://planning.wiki/ref/planners/popf
.. _POPFPlanSolver: https://github.com/PlanSys2/ros2_planning_system/tree/rolling/plansys2_popf_plan_solver
.. _TFD: https://tfd.informatik.uni-freiburg.de/
.. _TFDPlanSolver: https://github.com/PlanSys2/plansys2_tfd_plan_solver
.. _OPTIC: https://planning.wiki/ref/planners/optic
.. _OPTICPlanSolver: https://github.com/sscpac/optic_planner
.. _Mips XXL: http://sjabbar.com/mips-xxl-planner
.. _MIPSPlanSolver: https://github.com/isacg5/ros2_planning_system_mips
.. _LPG-TD: https://lpg.unibs.it/lpg/
.. _LPGTDPlanSolver: https://github.com/Nuriadj/plansys2_lpgtd_plugin/tree/main
.. _Fast Downward: https://www.fast-downward.org/HomePage
.. _DownwardPlanner: https://github.com/kas-lab/downward_ros
.. _SymK: https://github.com/speckdavid/symk
.. _SymkPlanner: https://github.com/kas-lab/symk_ros
.. _UPF: https://github.com/aiplan4eu/unified-planning
.. _UPFPlanSolver: https://github.com/JRL-CARI-CNR-UNIBS/plansys2_upf_plugin

Behavior tree builder plugins
*****************************

The executor turns each plan into a behavior tree with the plugin of its
``bt_builder_plugin`` parameter:

* `STNBTBuilder`_, the default. It follows the time network of the plan, the start and the end
  of each action: an action starts as soon as what it needs is done, also when it only needs
  what another one does at its start.
* `SimpleBTBuilder`_. It waits for whole actions to end: simpler, less parallel.
* `SequentialBTBuilder`_. The actions run one after the other, in the order of the plan.

Whatever the builder, a timed plan runs each action at its time in the plan, not before: plans
with timed initial literals, processes, events, continuous effects or constraints with times,
or any plan when the executor's ``timed_execution`` parameter is true.

The assembly with arms of ``plansys2_bt_example`` runs the same plan with STN and Simple to
compare them.

.. _SimpleBTBuilder: https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/bt_builder_plugins/simple_bt_builder.cpp
.. _STNBTBuilder: https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/bt_builder_plugins/stn_bt_builder.cpp
.. _SequentialBTBuilder: https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/bt_builder_plugins/sequential_bt_builder.cpp
