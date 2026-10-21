.. _pddl_support:

PDDL support
############

|PN| loads a domain and a problem, asks a planner for a plan, and executes it. This page says
what PDDL each step supports. With Kairos, the default planner, everything below is loaded,
planned and executed.

A domain or a problem with something |PN| cannot represent fails when it is loaded, with a
message that says what (for instance ``PlanSys2 does not handle ...``). It is never loaded in
part.

What |PN| executes
******************

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Feature
     - How it is executed
   * - Types (also ``either``), constants, ``not``, ``and``, ``or``, ``imply``, ``exists``,
       ``forall``, ``=``
     - Conditions are checked in the knowledge of the problem expert before and while an
       action runs.
   * - Numeric fluents (``assign``, ``increase``, ``decrease``, ``scale-up``,
       ``scale-down``), metrics, action costs
     - Effects change the functions of the problem expert. Metrics and costs are for the
       planner.
   * - Object fluents (``(tool ?r)`` returns an object)
     - Kept as the object they hold; ``=`` compares by name, ``assign`` sets another object.
   * - Conditional effects (``when``, also inside ``forall``)
     - The condition is read in the state before the effect; the effect applies only if it
       holds.
   * - Durative actions, duration inequalities, ``?duration`` in effects
     - The performer gets the planned duration (``get_planned_duration()``); effects use it.
   * - Timed initial literals and fluents (``(at 10 (door_open d))``)
     - The problem expert applies them when they are due, counted from when it reads the
       problem.
   * - Derived predicates, also recursive
     - Recomputed from the facts, by strata, until nothing changes.
   * - PDDL+: processes, events, continuous effects (``#t``)
     - The problem expert simulates them while the plan runs (see below).
   * - Trajectory constraints (``always``, ``sometime``, ``within``, ``at-most-once``...)
     - Checked while the plan runs: a violation stops the plan and says which constraint.
   * - Preferences (``preference``, ``is-violated``)
     - For the planner: they never stop a plan.

Timed execution
***************

Some plans only make sense at their times: a timed initial literal happens at its time, a
process runs while time passes, a constraint has a deadline. The executor runs these plans
*timed*: each action starts at its time in the plan, not before, and a durative action lasts
its planned duration. It does so when the plan has timed initial literals or fluents,
processes, events, continuous effects or constraints with times, or always if its
``timed_execution`` parameter is true.

* The plan counts from when its problem was read (``Plan.stamp``, set by the planner), so the
  timed literals and the plan share their clock.
* If an action starts late, the rest of the plan shifts with it, so the planned intervals
  between actions hold. The execution trace says by how much and why (see
  :ref:`execution_trace`).

PDDL+ dynamics
**************

While a plan runs, the problem expert simulates the world every 50 ms: the active processes
and the continuous effects of the running actions change the functions for the time passed,
and then the events whose conditions hold happen.

* Parameter ``measured_functions`` of the problem expert: functions that the simulation does
  not change, because your perception updates them (for instance ``["level"]``).
* Parameter ``simulate_dynamics`` (default ``true``): set it to ``false`` when everything comes
  from perception.

What each planner plans
***********************

The planner gives a task only to the plan solvers that support what it uses: see
:ref:`plugins`. Kairos plans everything on this page. POPF plans STRIPS with typing,
equality, ``forall`` in conditions, numeric fluents, durative actions and timed initial
literals: a domain with conditional effects, derived predicates, object fluents, ``exists``,
preferences, PDDL+ or constraints is not sent to it.

Known limits
************

* An action waits for a timed initial literal it needs at most
  ``at_start_requirements_timeout`` seconds (executor, default 30): a literal further away
  makes the action fail.
* A ``when`` inside a ``forall`` is applied correctly, but the STN does not order other
  actions by it.
