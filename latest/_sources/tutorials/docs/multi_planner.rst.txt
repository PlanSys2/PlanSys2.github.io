.. _multi_planner:

Configuring Multiple Planners
******************************

- `Overview`_
- `Tutorial Steps`_

Overview
========

The Planner node does not have to be limited to a single PDDL solver. You can configure it with several plan solver
plugins at once, either different solvers (e.g. POPF and a third-party plugin such as OPTIC) or several instances of
the same solver configured with different command-line arguments. When more than one plugin is configured, PlanSys2
requests a plan from every plugin **in parallel** and keeps every plan that is found before the shared timeout expires.

This is useful because:

* Different solver configurations may succeed or fail on different problems, or find plans of different quality.
* Some solver arguments trade off search speed for plan quality (see `POPF arguments`_ below); running several
  configurations in parallel lets you get the fastest *usable* plan without having to guess the best configuration
  in advance.
* Applications that implement their own replanning strategy (for example, choosing the best candidate plan with an
  LLM, as described in `Peña-Narváez et al., 2026 <https://doi.org/10.3390/robotics15040080>`_) need access to every
  candidate plan, not just one.

Tutorial Steps
==============

1- The ``plan_solver_plugins`` parameter
-----------------------------------------

The ``planner`` node's ``plan_solver_plugins`` parameter takes a list of arbitrary **solver IDs**. Each ID must have
a corresponding block in the same YAML file declaring, at least, which plugin class to load:

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_timeout: 5.0
       plan_solver_plugins: ["POPF"]
       POPF:
         plugin: "plansys2/POPFPlanSolver"

* ``plan_solver_timeout`` [double]: Maximum time, in seconds, given to **each** configured plugin to find a plan. All
  plugins run concurrently, so this is not a cumulative budget: with 5 plugins configured, the total time to get a
  result is still bounded by this same timeout, not five times it.
* ``plan_solver_plugins`` [vector<string>]: The list of solver IDs to load. If left empty, PlanSys2 falls back to a
  single default solver named ``POPF``.
* For each ID in ``plan_solver_plugins``, a block named after that ID must declare at least the ``plugin`` key with
  the fully-qualified plugin class name (see :ref:`plugins` for the available planner plugins).

2- Running the same solver with different arguments
------------------------------------------------------

The ID does not have to match the underlying plugin's name — it is just a label you choose. This lets you configure
the **same** plugin several times with different options. For POPF (``plansys2/POPFPlanSolver``), two extra
per-solver parameters are available:

* ``<id>.arguments`` [string]: Extra command-line flags passed to the ``popf`` executable. Defaults to ``""``.
* ``<id>.output_dir`` [string]: Directory where the ``domain.pddl``, ``problem.pddl`` and ``plan.pddl`` files for
  this solver are written. Defaults to the system's temporary directory. Each namespaced node gets its own
  subdirectory under this path, so several PlanSys2 instances (e.g. simulating several robots) can safely share the
  same ``output_dir``.

For example, the following configuration runs four instances of POPF with progressively more aggressive search
settings, plus a fifth, differently-configured solver plugin, all queried in parallel every time a plan is requested:

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_timeout: 5.0
       plan_solver_plugins: ["POPF1", "POPF2", "POPF3", "POPF4", "OPTICS"]
       POPF1:
         plugin: "plansys2/POPFPlanSolver"
         arguments: "-h -E -A"
         output_dir: "/tmp/POPF1"
       POPF2:
         plugin: "plansys2/POPFPlanSolver"
         arguments: "-h -E"
         output_dir: "/tmp/POPF2"
       POPF3:
         plugin: "plansys2/POPFPlanSolver"
         arguments: "-h"
         output_dir: "/tmp/POPF3"
       POPF4:
         plugin: "plansys2/POPFPlanSolver"
         arguments: ""
         output_dir: "/tmp/POPF4"
       OPTICS:
         plugin: "plansys2/OPTICPlanSolver"
         arguments: "-N"
         output_dir: "/tmp/OPTICS"

.. note:: Unlike POPF, which ships with PlanSys2, ``OPTICPlanSolver`` (and other solvers such as ``TFDPlanSolver``)
  are separate, third-party plugin packages that must be installed independently. See :ref:`plugins` for the list of
  known planner plugins and where to get them. If a plugin package referenced in ``plan_solver_plugins`` is not
  installed, the planner node will fail to configure.

POPF arguments
^^^^^^^^^^^^^^

The most commonly used POPF flags, from fastest/greediest to most thorough, are:

* ``-h``: Disable helpful-action pruning.
* ``-E``: Skip Enforced Hill-Climbing (EHC) and go straight to best-first search.
* ``-A``: Use better actions in the heuristic, using h-add costs.
* ``""`` (no arguments): Run POPF with its default search strategy.

Combining these flags (as in the ``POPF1``-``POPF4`` example above) trades search completeness for speed, so running
several of them in parallel is a simple way to hedge between a fast, possibly suboptimal plan and a slower, more
thorough search, without having to pick a single strategy up front.

3- Requesting plans from all configured solvers
---------------------------------------------------

Once several solvers are configured, two client calls are available:

.. code-block:: c++

     std::optional<plansys2_msgs::msg::Plan> getPlan(const std::string & domain, const std::string & problem);
     plansys2_msgs::msg::PlanArray getPlanArray(const std::string & domain, const std::string & problem);

* ``getPlan()`` queries every configured solver in parallel and returns only the **shortest** plan found (the plans
  are sorted by number of actions before picking the first one). This is the call used internally whenever PlanSys2
  needs a single plan, for example from the terminal's ``get plan`` and ``run`` commands.
* ``getPlanArray()`` returns **every** plan found by every solver, sorted from shortest to longest, as a
  ``plansys2_msgs::msg::PlanArray``. Use this call if your application wants to choose among the candidates itself
  (for instance, to prefer the plan that best matches the plan currently being executed, or to delegate the choice
  to an external evaluator).

See the *Plan Repair and Multi-Planner Replanning* section of the :ref:`design` page for how this combines with
PlanSys2's automatic plan repair when a new plan is sent for execution while another one is still running.
