TrajectoryPoint
==============

Message Definition
-----------------

.. code-block:: none

    geometry_msgs/Pose     pose
    geometry_msgs/Twist    velocity
    duration              time_from_start

Field Descriptions
-----------------

* ``pose`` (geometry_msgs/Pose)
    The position and orientation at this trajectory point.

* ``velocity`` (geometry_msgs/Twist)
    The linear and angular velocity at this trajectory point.

* ``time_from_start`` (duration)
    Time from the start of the trajectory to this point.

Links
-----

* `geometry_msgs/Pose Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html>`_
* `geometry_msgs/Twist Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import TrajectoryPoint
    from geometry_msgs.msg import Pose, Twist

    # Create a TrajectoryPoint message
    point = TrajectoryPoint()
    
    # Set pose
    point.pose = Pose()
    point.pose.position.x = 1.0
    point.pose.position.y = 2.0
    point.pose.position.z = 0.0
    # Set orientation...
    
    # Set velocity
    point.velocity = Twist()
    point.velocity.linear.x = 0.5
    # Set other velocity components...
    
    # Set timing
    point.time_from_start = rospy.Duration(1.0)  # 1 second from start
