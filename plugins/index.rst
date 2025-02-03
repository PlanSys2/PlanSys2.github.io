.. _plugins:

Plugins
#######

Planner plugins
***************

This page lists the known planner plugins for PlanSys2.

+------------------+----------------------+------------------------------------------+
| Planner          | Plugin               | Highlights                               |
+==================+======================+==========================================+
| `POPF`_          | `POPFPlanSolver`_    | Numeric fluents; Durative actions.       |
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

This page lists the available BT builder plugins.

+----------------------+
| Plugin               |
+======================+
| `SimpleBTBuilder`_   |
+----------------------+
| `STNBTBuilder`_      |
+----------------------+

.. _SimpleBTBuilder: https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/bt_builder_plugins/simple_bt_builder.cpp
.. _STNBTBuilder: https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_executor/src/plansys2_executor/bt_builder_plugins/stn_bt_builder.cpp
