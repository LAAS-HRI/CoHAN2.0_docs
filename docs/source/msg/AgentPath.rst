AgentPath
=========

Message Definition
-----------------

.. code-block:: none

    Header           header
    uint64          id
    nav_msgs/Path   path

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``id`` (uint64)
    Unique identifier for the agent.

* ``path`` (nav_msgs/Path)
    The planned or tracked path of the agent.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* `nav_msgs/Path Message <http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import AgentPath
    from nav_msgs.msg import Path

    # Create an AgentPath message
    agent_path = AgentPath()
    
    # Set header
    agent_path.header.stamp = rospy.Time.now()
    agent_path.header.frame_id = "map"
    
    # Set agent ID
    agent_path.id = 1
    
    # Set path
    agent_path.path = Path()
    agent_path.path.header = agent_path.header
    # Add poses to path as needed
