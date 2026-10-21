.. _execution_trace:

Reading an execution
####################

While a plan runs, the executor says what it does and why: which action waits and for what,
which performer takes it, when it starts and ends against the plan, when the rest of the plan
goes later, what fails. This is the *execution trace*. Read it to understand an execution, or
to find why it did not go as planned.

Where to read it
****************

* **The log of the executor.** Each event is a line that starts with ``[trace]``. The
  executor's parameter ``log_execution_trace`` (default ``true``) turns it off.
* **The topic** ``executor/execution_trace`` (``plansys2_msgs/msg/ExecutionEvent``). It keeps
  the last 1000 events (transient local), so a tool that starts later still gets the whole
  plan.
* **The TUI**, tab *Plan log* (press ``2``): the same events as sentences.

  .. code-block:: bash

     ros2 run plansys2_tui_cli plansys2_tui

The events
**********

Each event has the time since the plan started (``plan_time``), the action it is about, and a
detail.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Event
     - Meaning
   * - ``PLAN_START``
     - A plan starts: how it runs (timed, or each action as soon as it can), its actions, the
       BT builder.
   * - ``WAIT``
     - An action waits, and for what: its time in the plan, its requirements (only those that
       do not hold yet), or another action of the plan.
   * - ``PERFORM``
     - A performer takes the action: which one.
   * - ``START``, ``END``
     - The start or the end of the action happened (its effects are applied), with its
       planned time and how late it is.
   * - ``SHIFT``
     - The rest of the plan goes later, by how much, and why (an action started late).
   * - ``FAILURE``
     - Something failed, and why: the performer's message, the requirements that do not hold.
   * - ``VIOLATION``
     - A constraint of the problem is violated: which one.
   * - ``REPLAN``
     - A new plan replaces the running one: which running actions go on, which are dropped.
   * - ``PLAN_END``
     - How the plan ended: succeeded, failed, cancelled, aborted, or an error setting up the
       new plan.

The message also carries ``planned_time`` and ``shift`` for ``START`` and ``END``: the action
is late by ``plan_time - planned_time - shift``.

Late, and shifted
*****************

In a timed plan, an action that starts late moves the rest of the plan with it, so the
intervals the plan needs between actions hold. The trace says it with a ``SHIFT``, and the
next actions show the shift of the plan apart from their own lateness:

.. code-block:: text

   [trace]    17.925 END (recharge rover):7899 planned 17.899 (+0.005 shifted) late +0.020
   [trace]    17.925 SHIFT (move rover dock corridor):17899: 0.020 s later: it starts late
   [trace]    17.926 START (move rover dock corridor):17899 planned 17.899 (+0.026 shifted) late +0.001

``move`` starts 0.026 s after its planned time, but only 0.001 s late on the plan as it was
shifted: the rest is what the previous actions took.

An example
**********

The PDDL+ greenhouse of ``plansys2_pddlplus_example``, in the TUI, launched so that the recharge
fails (``ros2 launch plansys2_pddlplus_example plansys2_pddlplus_example.launch.yaml
recharge_fails_at:=0.5``); the controller replans from where the world is:

.. code-block:: text

   ━━ 06:24:44 Plan starts: timed, each action at its time, its problem read 1.605 s before, 9 actions, STNBTBuilder
       1.606  (recharge rover) waits for its time in the plan, 7.999
       8.013  (recharge rover) starts on time (planned at 7.999, the plan 0.013 s later)
       8.033  (recharge rover) is performed by recharge_action_node
       9.033  (move rover dock corridor): its START waits for the END of (recharge rover)
      11.032  (recharge rover) fails: its performer recharge_action_node says: failure injected
      11.052  ━━ The plan ends: failed

   ━━ 06:24:54 Plan starts: timed, each action at its time, its problem read 0.053 s before, 7 actions, STNBTBuilder
       0.055  (move rover dock corridor) starts 0.055 s late (planned at 0.000)
       3.068  (move rover dock corridor) ends 0.068 s late (planned at 3.000)
       3.069  (plug rover corridor nursery): the rest of the plan goes 0.068 s later: it starts late
       ...
       9.307  ━━ The plan ends: succeeded

What to look for
****************

* **Long waits.** A ``WAIT`` for requirements that lasts says which conditions do not hold:
  a fact that never comes, a performer that never sets it.
* **Large lateness or shifts.** Performers slower than the planned durations, or a loaded
  system. ``get_planned_duration()`` tells a performer how long the plan gives it.
* **Actions that wait for each other when they could run at once.** With
  ``SimpleBTBuilder``, an action waits for whole actions to end; ``STNBTBuilder`` starts it
  as soon as what it needs is done.
* **Failures and violations**: the detail says the reason.
