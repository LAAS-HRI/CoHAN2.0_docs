Install
=======

Preparing things (Native)
-------------------------

1. Install ROS `Melodic <https://wiki.ros.org/melodic/Installation/Ubuntu>`_ on Ubuntu 18.04 or `Noetic <https://wiki.ros.org/noetic/Installation/Ubuntu>`_ on Ubuntu 20.04. 
2. This installation assumes that the `ROS <http://wiki.ros.org/ROS/Installation>`_ is already installed along with the `2D navigation stack <http://wiki.ros.org/navigation>`_. Otherwise, please install them before continuing to the next steps. For ROS Noetic, follow these:
3. Install the requirements::

       sudo apt install python-pip python-catkin-tools python-is-python3

4. Clone the git repository::

       git clone https://github.com/LAAS-HRI/CoHAN2.0.git -b master ~
       cd ~/CoHAN2.0

5. Install the dependencies using rosdep::

       rosdep install --from-paths src --ignore-src --rosdistro=noetic -y

6. Follow the build instructions below.


Using Docker
------------

1. Clone the git repository::

       git clone https://github.com/LAAS-HRI/CoHAN2.0.git -b master ~
       cd ~/CoHAN2.0

2. Navigate to docker directory in the folder and build the image. For example::

       cd ~/CoHAN2.0/docker/noetic
       ./build-docker.sh

3. Activate the image and do the steps for building. For example::

       source ~/CoHAN2.0/docker/noetic/run-docker.sh


Building Cohan and Running Examples
-----------------------------------

1. Everything will be taken care of by the script. Just run compile.sh script::

       ./compile.sh

2. Once it is built, you need to source setup.bash file to use these packages along with other ROS packages inside the system::

       source devel/setup.bash

3. Now you are set to run use it. You can run some examples as::

       roslaunch cohan_sim_navigation cohan_sim_pr2.launch

   If everything is installed correctly, you should see rviz opening and you can move the robot by giving it a goal (using 2D Nav Goal tool). 

