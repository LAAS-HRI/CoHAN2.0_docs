PointArray
==========

Message Definition
-----------------

.. code-block:: none

    Header              header
    geometry_msgs/Point[] points

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``points`` (geometry_msgs/Point[])
    Array of 3D points representing positions in space.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* `geometry_msgs/Point Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import PointArray
    from geometry_msgs.msg import Point

    # Create a PointArray message
    point_array = PointArray()
    
    # Set header
    point_array.header.stamp = rospy.Time.now()
    point_array.header.frame_id = "map"
    
    # Add points to the array
    point = Point()
    point.x = 1.0
    point.y = 2.0
    point.z = 0.0
    point_array.points.append(point)
    
    # Add more points as needed
