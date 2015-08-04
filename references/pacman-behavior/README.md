ROS based implementation of intelligent pacman agent. (Graduation project)

--

To start working on this project run:

```bash
$ git clone https://github.com/tiagopms/pacman-behavior ./
```

And then you will have a copy of this project.

To run it, compile with ROS Indigo:

```bash
$ catkin_make --pkg pacman_msgs
$ catkin_make --pkg pacman_interface
$ catkin_make
```

And then use:

```bash
$ roscore
$ rosrun pacman_game pacman_game.py
$ rosrun bayesian_q_5_behaviors bayesian_q_learning_5_behaviors_node
```

In three different terminal windows.
