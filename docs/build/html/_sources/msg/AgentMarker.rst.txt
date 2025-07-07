AgentMarker
===========

Message Definition
-----------------

.. code-block:: none

    uint64            id
    bool              active
    geometry_msgs/Pose    pose
    geometry_msgs/Twist   velocity

Field Descriptions
-----------------

* ``id`` (uint64)
    Unique identifier for the agent marker.

* ``active`` (bool)
    Flag indicating if the agent is currently active.

* ``pose`` (geometry_msgs/Pose)
    The current pose (position and orientation) of the agent.

* ``velocity`` (geometry_msgs/Twist)
    The current velocity (linear and angular) of the agent.

Links
-----

* `geometry_msgs/Pose Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html>`_
* `geometry_msgs/Twist Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import AgentMarker
    from geometry_msgs.msg import Pose, Twist

    # Create an AgentMarker message
    marker = AgentMarker()
    marker.id = 1
    marker.active = True
    marker.pose = Pose()  # Set pose values
    marker.velocity = Twist()  # Set velocity values
