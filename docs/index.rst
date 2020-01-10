Welcome to |project|'s documentation!
================================================

Description
-----------

|description|

The base_footprint is used as a frame for navigation. It represents the position and orientation of the whole robot.
A more elaborate description of this concept is described in REP120_.

Subscriptions 
~~~~~~~~~~~~~

``/walk_support_state`` -- std_msgs/Char

``/dynamic_kick_support_state`` -- std_msgs/Char

Publishes
~~~~~~~~~

nothing

TFs
~~~

``base_link`` --> ``base_footprint``



.. _REP120: https://www.ros.org/reps/rep-0120.html#base-footprint


.. toctree::
   :maxdepth: 2

   cppapi/library_root
   pyapi/modules


Indices and tables
==================

* :ref:`genindex`
* |modindex|
* :ref:`search`
