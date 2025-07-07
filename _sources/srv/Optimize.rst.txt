Optimize
========

Service Definition
-----------------

.. code-block:: none

    # Get optimized timed elastic bands for given agents and robot plans

    # Request
    nav_msgs/Path              robot_plan
    AgentPathArray            agent_plan_array
    int64[]                   agents_ids
    ---
    # Response
    bool                      success
    string                    message
    geometry_msgs/Twist       cmd_vel
    Trajectory                robot_trajectory
    AgentTrajectoryArray     human_trajectories

Request Fields
-------------

* ``robot_plan`` (nav_msgs/Path)
    Initial path plan for the robot.

* ``agent_plan_array`` (AgentPathArray)
    Array of initial path plans for the agents.

* ``agents_ids`` (int64[])
    Array of agent IDs to consider in the optimization.

Response Fields
--------------

* ``success`` (bool)
    Indicates if the optimization was successful.

* ``message`` (string)
    Status message or error description.

* ``cmd_vel`` (geometry_msgs/Twist)
    Optimized velocity command for the robot.

* ``robot_trajectory`` (Trajectory)
    Optimized trajectory for the robot.

* ``human_trajectories`` (AgentTrajectoryArray)
    Optimized trajectories for the human agents.

Links
-----

* `nav_msgs/Path Message <http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html>`_
* `geometry_msgs/Twist Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>`_
* :doc:`../msg/AgentPathArray`
* :doc:`../msg/Trajectory`
* :doc:`../msg/AgentTrajectoryArray`

Example Usage
------------

.. code-block:: python

    # Python
    import rospy
    from cohan_msgs.srv import Optimize
    from nav_msgs.msg import Path
    from cohan_msgs.msg import AgentPathArray

    # Create service proxy
    optimize = rospy.ServiceProxy('optimize', Optimize)

    # Create request
    robot_plan = Path()
    # Set robot plan...

    agent_plans = AgentPathArray()
    # Set agent plans...

    agents = [1, 2]  # Example agent IDs

    try:
        # Call service
        response = optimize(robot_plan, agent_plans, agents)
        
        if response.success:
            # Process optimized trajectories
            robot_traj = response.robot_trajectory
            human_trajs = response.human_trajectories
            cmd_vel = response.cmd_vel
        else:
            rospy.logwarn("Optimization failed: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
