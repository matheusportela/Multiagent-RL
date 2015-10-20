Multiagent-RL
=============

Introduction
------------

This repository contains the code used in the undergraduate thesis in Mechatronics Engineering, at the [University of Brasilia](http://www.unb.br), entitled "Reinforcement Learning applied to behaviour selection in multiple autonomous agents".

The idea is to have multiple [robotic] agents learn interesting behaviors in a stochastic environment. The uncertainty of a state is handled through [Bayesian Programming](https://en.wikipedia.org/wiki/Bayesian_programming), and the he agents learn through [Q-learning](https://en.wikipedia.org/wiki/Q-learning). Currently, the approach is being tested in the predator-prey problem, a simplified multi-agent situation, to answer: __can the ghosts learn to get the Pac-Man?__

Code structure
--------------

* Programming language: [Python 2.7](http://www.python.org)
* Required packages: [The Pac-Man AI Projects](http://ai.berkeley.edu/project_overview.html), by UC Berkeley

Installation
------------

This assumes a GNU/Unix distribution (Ubuntu), but everything is in Pỳthon so the setup shouldn't be too different for other platforms. The Pac-Man AI Projects provides six Pac-Man-like simulators that are free to use for educational purposes. The one we will be using is [Project 5: Classification](http://ai.berkeley.edu/classification.html), which provides an arena mimicking the complete Pac-Man game, including various ghosts.

This project requires the following Python packages:

* *[Tkinter](https://docs.python.org/2/library/tkinter.html)*: graphical user interfaces
* *[ZeroMQ](http://zeromq.org/)*: interprocess communication
* *[Matplotlib](http://matplotlib.org/)*: graphics plotting
* *[Numpy](http://www.numpy.org/)*: numerical computation

Install by running the following commands.

```bash
sudo apt-get install python python-dev python-pip python-tk libzmq-dev python-matplotlib
sudo pip install pyzmq
```

Usage
-----

In order to run the system, two processes are necessary: one that implements agents intelligence, in our case `pacman_mas.py`, and another that provides an interface to the real agents. Since we are using the Pacman simulator, we interface it in the module `simulator.py`.

Hence, run the system my executing the following commands in two different terminals:

```bash
python pacman_mas.py
```

and

```bash
python simulator.py
```
