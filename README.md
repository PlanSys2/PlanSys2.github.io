# ros2_planning_system.github.io
http://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io

This folder holds the source and configuration files used to generate the
[PlanSys2 documentation](http://intelligentroboticslab.gsyc.urjc.es/ros2_planning_system.github.io) web site.

Dependencies for Build: 
* [Sphinx](https://www.sphinx-doc.org/en/master/usage/installation.html)
* `sudo apt install python3-pip`
* `pip3 install breathe==4.30.0 sphinx_rtd_theme sphinxcontrib-plantuml`

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.

