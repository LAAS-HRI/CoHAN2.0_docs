TrajectoryArray
==============

Message Definition
-----------------

.. code-block:: none

    Header            header
    Trajectory[]      trajectories

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``trajectories`` (Trajectory[])
    Array of trajectories.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`Trajectory Message <Trajectory>`

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import TrajectoryArray, Trajectory

    # Create a TrajectoryArray message
    traj_array = TrajectoryArray()
    
    # Set header
    traj_array.header.stamp = rospy.Time.now()
    traj_array.header.frame_id = "map"
    
    # Add trajectories
    trajectory = Trajectory()
    # Configure trajectory...
    traj_array.trajectories.append(trajectory)
    
    # Add more trajectories as needed
