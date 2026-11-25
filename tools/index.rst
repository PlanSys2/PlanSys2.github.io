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

``plansys2_tui`` shows |PN| in a terminal window, in three tabs:

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

* **Behavior tree** (key ``3``): the behavior tree that runs, node by node. See
  :ref:`tui_behavior_tree`.

The Plan Monitor and Knowledge panels say when the node that feeds them is not running, and keep
their last data. The TUI does not run on Humble (see :ref:`migration`).

.. code-block:: bash

   ros2 run plansys2_tui_cli plansys2_tui

.. _tui_behavior_tree:

Behavior tree tab
=================

The tab shows a behavior tree while it runs, the same as Groot2 does in its monitor mode, without
leaving the terminal. It takes the tree from the monitor of BehaviorTree.CPP
(``Groot2Publisher``), so it shows any tree that publishes it:

* **The plan**: the tree the executor builds from the plan, one branch per action. The executor
  publishes it when ``enable_groot_monitoring`` is ``true``, on ``server_port`` (1800 by
  default). With the bringup launchers (``groot_port`` changes the port):

  .. code-block:: bash

     ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
       model_file:=<your domain> enable_groot_monitoring:=true

* **A BT action**: the tree of an action of ``plansys2_bt_actions``, with the parameters
  ``enable_groot_monitoring`` and ``server_port`` of that performer (see :ref:`bt_actions`).

At the top of the tab is the address of the tree: ``localhost:1800`` (the executor) at first.
Press ``p`` to edit it, and ``Enter`` to connect. It takes ``1700``, ``host:1700`` or
``tcp://host:1700``. The header next to it says the tree, how many nodes there are in each status,
and whether it follows the execution.

Each node shows its type, its name, and its ports:

.. list-table::
   :header-rows: 1
   :widths: 15 25 60

   * - Symbol
     - Status
     - Meaning
   * - ``●`` (yellow)
     - running
     - The node runs now.
   * - ``✔`` (green)
     - success
     - The node ended well.
   * - ``✘`` (red)
     - failure
     - The node failed.
   * - ``–`` (cyan)
     - skipped
     - The node was skipped (a precondition of BehaviorTree.CPP).
   * - ``○`` (dim)
     - idle
     - The node did not run yet, or its tree was halted or reset.

While it follows the execution (``f`` switches it on and off), the tab opens the branches that
run or failed and closes the ones that ended well, so a tree of a hundred nodes is still read at a
glance. Not following, the branches stay as you leave them.

When the executor starts a new plan, or replans, it publishes a new tree: the tab draws it in place
of the old one. If nothing publishes at that address, the header says so and asks whether
``enable_groot_monitoring`` is ``true`` there; if the publisher goes, the tab keeps the last tree it
knew.

The tab needs ``python3-zmq`` (an ``exec_depend`` of ``plansys2_tui_cli``, installed by
``rosdep``).

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
