GetTrajectory
============

Service Definition
-----------------

.. code-block:: none

    # Request
    geometry_msgs/PoseStamped    robot_goal
    int64[]                      agents_ids
    ---
    # Response
    bool                         success
    string                       message
    geometry_msgs/Twist          cmd_vel
    Trajectory                   robot_trajectory
    AgentTrajectoryArray        human_trajectories

Request Fields
-------------

* ``robot_goal`` (geometry_msgs/PoseStamped)
    Goal pose for the robot.

* ``agents_ids`` (int64[])
    Array of agent IDs to consider in trajectory planning.

Response Fields
--------------

* ``success`` (bool)
    Indicates if the trajectory generation was successful.

* ``message`` (string)
    Status message or error description.

* ``cmd_vel`` (geometry_msgs/Twist)
    Computed velocity command for the robot.

* ``robot_trajectory`` (Trajectory)
    Planned trajectory for the robot.

* ``human_trajectories`` (AgentTrajectoryArray)
    Predicted trajectories for the human agents.

Links
-----

* `geometry_msgs/PoseStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html>`_
* `geometry_msgs/Twist Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>`_
* :doc:`../msg/Trajectory`
* :doc:`../msg/AgentTrajectoryArray`

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.srv import GetTrajectory
    from geometry_msgs.msg import PoseStamped

    # Create service proxy
    get_trajectory = rospy.ServiceProxy('get_trajectory', GetTrajectory)

    # Create request
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    # Set goal pose...

    agents = [1, 2]  # Example agent IDs

    try:
        # Call service
        response = get_trajectory(goal, agents)
        
        if response.success:
            # Process trajectories
            robot_traj = response.robot_trajectory
            human_trajs = response.human_trajectories
        else:
            rospy.logwarn("Failed to get trajectory: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
