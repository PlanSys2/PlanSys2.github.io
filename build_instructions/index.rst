.. _build-instructions:

Build and Install
#################

Releases and distributions
**************************

|PN| 4.0.0 is the same version for Humble, Jazzy, Kilted and Lyrical: the same features and the same
code in every distribution. It brings Kairos as the default planner, the execution of
everything Kairos plans (conditional effects, timed literals, PDDL+, constraints...) and the
execution trace. The earlier releases are documented in version 3.1.0 of this site (version
selector, bottom left).

.. only:: latest

   This is the documentation of the ``rolling`` branch, what comes after 4.0.0. The version
   selector (bottom left) shows the one of 4.0.0.

.. only:: not latest

   Install the binaries
   ********************

   1. Install ROS 2 as described in the `official documentation <https://docs.ros.org>`_.
   2. Install |PN|:

      .. code-block:: bash

         sudo apt install ros-<distro>-plansys2-*

   .. note::

      4.0.0 reaches the binaries of each distribution with its next sync: until then, ``apt``
      installs the previous release.

Build from source
*****************

.. only:: latest

   This is the development documentation: what it describes is in the ``rolling`` branches
   and is not released yet. Build it from source.

|PN| needs some packages that are built with it from source:

* `Kairos <https://github.com/PlanSys2/kairos>`_, the default planner.
* `cascade_lifecycle <https://github.com/fmrico/cascade_lifecycle>`_ and
  `popf <https://github.com/fmrico/popf>`_, from the branches that match |PN|.
* For the examples, `EasyNav <https://github.com/EasyNavigation/EasyNavigation>`_ and
  `yaets <https://github.com/fmrico/yaets>`_.

Each repository lists them in ``dependency_repos.repos``; ``vcs`` clones them:

.. code-block:: bash

   sudo apt install python3-vcstool
   mkdir -p ~/plansys2_ws/src
   cd ~/plansys2_ws/src
   git clone -b rolling https://github.com/PlanSys2/ros2_planning_system.git
   vcs import < ros2_planning_system/dependency_repos.repos

   # The examples, optional
   git clone -b rolling https://github.com/PlanSys2/ros2_planning_system_examples.git
   vcs import --skip-existing < ros2_planning_system_examples/dependency_repos.repos

   cd ~/plansys2_ws
   rosdep install -y -r -q --from-paths src --ignore-src
   colcon build --symlink-install

* For a released distribution, use ``<distro>-devel`` instead of ``rolling`` for |PN| (for
  instance ``jazzy-devel``) and ``<distro>`` for the examples (for instance ``jazzy``).
* The examples' ``dependency_repos.repos`` also lists |PN| itself: ``--skip-existing`` keeps
  the one cloned above.

Choosing the middleware
***********************

|PN| works with any ROS 2 middleware (RMW), but we recommend **Zenoh**, the one we develop and
test |PN| with; Cyclone DDS works as well. Our stress tests start |PN|, its performers and the
controller of each example in every order (all at once, the controller first, |PN| minutes
later...), three times each. With Zenoh and Cyclone DDS they never failed. With Fast DDS, the
default RMW of ROS 2, some runs lost messages while everything was starting: the server side
gives up a response it cannot deliver (``failed to send response ... client will not receive
response``). |PN| sends such requests again, but the processes of an application (a navigation
performer and its navigation stack, for instance) may not.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - RMW
     - Runs that ended well
   * - Zenoh
     - 57 of 57
   * - Cyclone DDS
     - 57 of 57
   * - Fast DDS
     - 45 of 57

To use Zenoh, which also needs its router running (once per machine):

.. code-block:: bash

   sudo apt install ros-<distro>-rmw-zenoh-cpp
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ros2 run rmw_zenoh_cpp rmw_zenohd

To use Cyclone DDS:

.. code-block:: bash

   sudo apt install ros-<distro>-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

All the processes (|PN|, the performers, the controller) must use the same RMW.

If you stay with Fast DDS: processes killed with ``kill -9`` or crashed leave shared memory
segments behind in ``/dev/shm``, and later processes may send to them and lose messages.
``fastdds shm clean`` removes them.

Optional planners
*****************

`TFD <https://github.com/PlanSys2/plansys2_tfd_plan_solver>`_ is another plan solver plugin.
Install TFD first, as its repository explains, then clone the plugin into the workspace (it
has the branches ``rolling``, ``lyrical`` and ``humble``):

.. code-block:: bash

   cd ~/plansys2_ws/src
   git clone -b rolling https://github.com/PlanSys2/plansys2_tfd_plan_solver.git

See :ref:`plugins` to choose the planners PlanSys2 uses.
