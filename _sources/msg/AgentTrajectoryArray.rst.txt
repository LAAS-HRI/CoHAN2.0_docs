AgentTrajectoryArray
===================

Message Definition
-----------------

.. code-block:: none

    AgentTrajectory[]    trajectories

Field Descriptions
-----------------

* ``trajectories`` (cohan_msgs/AgentTrajectory[])
    Array of trajectories for multiple agents.

Links
-----

* :doc:`AgentTrajectory Message <AgentTrajectory>`

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.msg import AgentTrajectoryArray, AgentTrajectory

    # Create an AgentTrajectoryArray message
    traj_array = AgentTrajectoryArray()
    
    # Add trajectories for multiple agents
    agent_traj = AgentTrajectory()
    # Configure agent_traj...
    traj_array.trajectories.append(agent_traj)
    
    # Add more trajectories as needed
