.. _tools:

Tools
#####

|PN| comes with tools to load knowledge, plan, run plans, and watch what happens.

Terminal
********

``plansys2_terminal`` is an interactive shell: set instances, predicates, functions and goals,
ask for plans and run them, query the domain and the problem. ``source <file>`` runs the
commands of a file. :ref:`terminal_usage` explains every command.

.. code-block:: bash

   ros2 run plansys2_terminal plansys2_terminal

TUI
***

``plansys2_tui`` shows |PN| in a terminal window, in two tabs:

* **Overview** (key ``1``): four panels.

  - *Action Execution*: each performer and the action it runs, from ``performers_status``
    and ``actions_hub``.
  - *Plan Monitor*: the plan being executed and the progress of each action, from
    ``executing_plan`` and ``action_execution_info``.
  - *Performers*: the state of each performer.
  - *Knowledge*: instances, predicates, functions with their values, goal, metric and
    constraints, from ``problem_expert/knowledge``.

* **Plan log** (key ``2``): the execution trace as sentences, one per event; ``c`` clears it.
  See :ref:`execution_trace`.

A panel says when the node that feeds it is not running, and keeps its last data.

.. code-block:: bash

   ros2 run plansys2_tui_cli plansys2_tui

Command line
************

``ros2 plansys2`` prints the same information in the shell:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Command
     - What it shows
   * - ``ros2 plansys2 knowledge``
     - The knowledge of the problem expert.
   * - ``ros2 plansys2 plan_monitor``
     - The plan being executed and the progress of its actions.
   * - ``ros2 plansys2 performers``
     - The registered action performers and their state.
   * - ``ros2 plansys2 execution_monitor``
     - The actions running, from ``actions_hub``.

rqt plugins
***********

``plansys2_tools`` adds three plugins to rqt (menu *Plugins > PlanSys2*): *Knowledge*,
*Performers* and *Plan*. Each one also runs alone:

.. code-block:: bash

   ros2 run plansys2_tools rqt_plansys2_plan

Logger
******

``ros2 run plansys2_tools logger`` writes to its log the messages of ``actions_hub``,
``action_execution_info``, ``executing_plan``, ``performers_status`` and
``problem_expert/knowledge``.
