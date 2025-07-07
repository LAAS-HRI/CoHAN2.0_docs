AgentPathArray
==============

Message Definition
-----------------

.. code-block:: none

    Header               header
    AgentPath[]         paths

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``paths`` (cohan_msgs/AgentPath[])
    Array of paths for multiple agents.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`AgentPath Message <AgentPath>`

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import AgentPathArray, AgentPath

    # Create an AgentPathArray message
    path_array = AgentPathArray()
    
    # Set header
    path_array.header.stamp = rospy.Time.now()
    path_array.header.frame_id = "map"
    
    # Add paths for multiple agents
    agent_path = AgentPath()
    # Configure agent_path...
    path_array.paths.append(agent_path)
    
    # Add more agent paths as needed
