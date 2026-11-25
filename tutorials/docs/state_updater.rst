.. _state_updater_tutorial:

Keep the problem in sync with the sensors
*****************************************

- `Overview`_
- `Tutorial Steps`_

Overview
========

In the other tutorials, the knowledge of the problem comes from the terminal or from the
application. On a real robot, much of it is what the sensors say: the battery, a leak, the mode
of the vehicle, the waypoints that can be reached. The state updater writes those facts in the
problem by itself, from rules in a YAML file, with no code: each rule reads one field of one
topic, transforms it, and writes one fact.

In this tutorial we run |PN| with the example of ``plansys2_state_updater``, publish some sensor
messages from the shell, and watch how the problem follows them. :ref:`state_updater` explains
every option.

Tutorial Steps
==============

0- Requisites
-------------

Build |PN| from source (see :ref:`getting_started`); ``plansys2_state_updater`` is one of its
packages. The example is installed with it:

.. code-block:: bash

   CFG=$(ros2 pkg prefix --share plansys2_state_updater)/config
   ls $CFG
   # example.yaml  example_domain.pddl  example_problem.pddl

The domain has a battery (``(energy)``), a leak (``(leak_detected)``), modes (``(mode_is ?m)``),
waypoints (``(reachable ?w)``, and ``(location)``, an object fluent), and an action ``inspect``
that needs no leak while it runs:

.. code-block:: none

   (:durative-action inspect
     :parameters ()
     :duration (= ?duration 10)
     :condition (and (over all (not (leak_detected))) (at start (>= (energy) 20)))
     :effect (and (decrease (energy) (* #t 0.5)))
   )

The problem starts with ``(= (energy) 100)`` and ``(mode_is normal)``.

1- The rules
------------

``example.yaml`` has one rule for each kind of fact. These are the ones we use:

.. code-block:: yaml

   state_updater:
     rules:
       # Function, corrected only when 2 or more away from the model (#t)
       - name: battery
         topic: /robot/battery
         type: sensor_msgs/msg/BatteryState
         field: percentage
         transform: {linear: {scale: 100.0, round: 1}}
         target: {function: "(energy)"}
         policy: {mode: correct, min_change: 2.0, sample_period: 0.5, stale_after: 10.0,
                  on_stale: "(sensor_stale battery)"}

       # Predicate held 2 s before it is written: a sensor glitch does not abort the action
       - name: leak
         topic: /robot/leak
         type: std_msgs/msg/Bool
         field: data
         target: {predicate: "(leak_detected)"}
         policy: {debounce: 2.0, stale_after: 5.0, on_stale: "(sensor_stale leak)"}

       # Set: the waypoints in the list are reachable
       - name: reachable
         topic: /reachable
         type: rcl_interfaces/msg/ParameterValue
         field: string_array_value
         target: {set: "(reachable {})"}

       # Enum: one mode holds at a time; lost after 5 s of silence
       - name: mode
         topic: /robot/mode
         type: std_msgs/msg/Int32
         field: data
         transform: {map: {values: {2: normal, 3: degraded}, default: lost}}
         target: {enum: "(mode_is {})", values: [normal, degraded, lost]}
         policy: {debounce: 0.5, stale_after: 5.0, on_stale: {value: lost}}

       # Object fluent: (location) takes a waypoint name
       - name: location
         topic: /robot/waypoint
         type: std_msgs/msg/String
         field: data
         target: {function: "(location)"}

2- Running |PN| with the state updater
--------------------------------------

* Open a terminal and launch |PN| with the domain, the problem and the rules
  (``state_updater_config``):

  .. code-block:: bash

     CFG=$(ros2 pkg prefix --share plansys2_state_updater)/config
     ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
       model_file:=$CFG/example_domain.pddl problem_file:=$CFG/example_problem.pddl \
       state_updater_config:=$CFG/example.yaml

  The rules are checked against the domain and the problem; if one is wrong, the problem expert
  does not configure, and says why. Then it waits for the publishers of each topic, to take the
  QoS they offer:

  .. code-block:: text

     [problem_expert.state_updater]: 8 rules on 7 topics
     [problem_expert]: Updater state_updater (plansys2_state_updater/RulesUpdater) configured
     [problem_expert.state_updater]: Waiting for a publisher of '/robot/battery', to take the qos it offers

* Open a second terminal and watch the writes, each one with its reason:

  .. code-block:: bash

     ros2 topic echo /state_updater/events plansys2_msgs/msg/StateUpdate --field reason

* Open a third terminal and watch the problem: the TUI (tab *Overview*, panel *Knowledge*), or

  .. code-block:: bash

     ros2 topic echo /problem_expert/knowledge

3- The sensors speak
--------------------

Open a fourth terminal for the sensors. Each ``ros2 topic pub`` keeps publishing once a second
until you stop it with ``Ctrl+C``.

* **The battery.** The sensor says 60 %, the problem says 100: 40 away, more than
  ``min_change``, so it is written.

  .. code-block:: bash

     ros2 topic pub -r 1 /robot/battery sensor_msgs/msg/BatteryState "{percentage: 0.6}"

  .. code-block:: text

     battery: 100 → 60, deviation 40 ≥ 2

  The log of the launch says how it subscribed. The publisher of ``ros2 topic pub`` is reliable,
  so the subscription is too:

  .. code-block:: text

     Subscribed to '/robot/battery' (sensor_msgs/msg/BatteryState; reliability reliable, durability volatile, depth 5)

  Stop it. 10 s later the battery is stale, and ``(sensor_stale battery)`` holds; publish again,
  and it goes.

* **The mode.** ``3`` maps to ``degraded``: ``(mode_is normal)`` goes and
  ``(mode_is degraded)`` holds, never both at once.

  .. code-block:: bash

     ros2 topic pub -r 1 /robot/mode std_msgs/msg/Int32 "{data: 3}"

  Stop it: 5 s later the mode is ``lost`` (``on_stale: {value: lost}``).

* **The reachable waypoints.** The list is the set of facts that hold:

  .. code-block:: bash

     ros2 topic pub -r 1 /reachable rcl_interfaces/msg/ParameterValue \
       "{type: 9, string_array_value: [wp1, wp3]}"

  ``(reachable wp1)`` and ``(reachable wp3)`` hold. Publish ``[wp3]`` instead, and
  ``(reachable wp1)`` goes.

* **The location**, an object fluent:

  .. code-block:: bash

     ros2 topic pub -r 1 /robot/waypoint std_msgs/msg/String "{data: wp2}"

  The problem has ``(= (location) wp2)``.

* **A leak.** It must hold 2 s (``debounce``) before it is written. Publish ``true`` for less
  than 2 s and stop: nothing is written. Publish it for longer:

  .. code-block:: bash

     ros2 topic pub -r 2 /robot/leak std_msgs/msg/Bool "{data: true}"

  ``(leak_detected)`` holds. An ``inspect`` running now would be cut by the executor: its over all
  condition no longer holds.

4- While it runs
----------------

* The state of every rule, once a second, as JSON:

  .. code-block:: bash

     ros2 topic echo /state_updater/status --field data

* Disable some rules (their facts stay as they are), and enable them again:

  .. code-block:: bash

     ros2 param set /problem_expert state_updater.disabled_rules "['battery', 'leak']"
     ros2 param set /problem_expert state_updater.disabled_rules "[]"

* Change ``example.yaml`` (copy it somewhere first, and launch with the copy), and load it
  without stopping |PN|. If the new rules are not valid, the old ones stay:

  .. code-block:: bash

     ros2 service call /state_updater/reload std_srvs/srv/Trigger

  .. code-block:: text

     success=True, message='8 rules on 7 topics, 8 kept as they were'

5- Next
-------

* ``ros2 run plansys2_state_updater replanning_mission.py`` is a mission that plans again when
  the state updater changes the world (see :ref:`state_updater`).
* For your robot, write the rules of your own topics and facts. A value the transforms cannot
  compute is a transform plugin; something the rules cannot do at all is an updater plugin of
  your own.
