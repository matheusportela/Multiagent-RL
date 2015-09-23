# Multiagent-RL
## Introduction
This repository contains the code used in the undergraduate thesis in Mechatronics Engineering, at the
[University of Brasilia](http://www.unb.br), entitled "Reinforcement Learning applied to behaviour selection in multiple
autonomous agents".

## Code structure
* Programming language: [Python 2.7](http://www.python.org)
* Required packages: [The Pac-Man AI Projects](http://ai.berkeley.edu/project_overview.html), by UC Berkeley

## Installation
The Pac-Man AI Projects, by UC Berkeley, provides six Pac-Man-like simulators that are free to use for educational
purposes. The one we will be using is [Project 5: Classification](http://ai.berkeley.edu/classification.html), which
provides an arena mimicking the complete Pac-Man game, including various ghosts.

The project requires the following Python packages:

* *Tkinter*: graphics plotting
* *ZeroMQ*: interprocess communication

Install by running the following commands.

```
sudo apt-get install python python-pip python-tk libzmq-dev
sudo pip install pyzmq
```

## Usage

In order to run the system, two parts are necessary: one process that implements agents intelligence, in our case `pacman_mas.py`
and another that provides an interface to the real agents. Since we are using the Pacman simulator, we interface it in the module
`simulator.py`.

Hence, run the system my executing the following commands in two different terminals:

```
python pacman_mas.py
```

and

```
python simulator.py
```

## Extras
### Simulator standalone installation

This project already includes the simulator adapter to our purposes. However, you might want to give a look at the
original simulator by yourself.

Start installing by downloading the `.zip` file located [here](http://ai.berkeley.edu/classification.html). Not all
files are necessary for our purposes since most of them are evaluation features. A stripped-down version of the
simulator can be downloaded [here](https://mega.co.nz/#!otcGnJAb!IZ6MLmS2fMwu2MPyGWKCGhJMRul4SgiPelk08wEjOP4). Here is
the list of files that must be maintained, the others can be safely discarded.

* `layouts/` directory
* `game.py`
* `ghostAgents.py`
* `graphicsDisplay.py`
* `graphicsUtils.py`
* `keyboardAgents.py`
* `layout.py`
* `pacman.py`
* `pacmanAgents.py`
* `util.py`
* `textDisplay.py`

Simply run the simulator with the following command:

`python pacman.py`

Several aspects can be configured, such as Pac-Man and ghosts AI agents, number of ghosts, game layout etc. Visualize
all possible settings with the following command:

`python pacman.py -h`

For instance, to run the game with the `GreedyAgent` Pac-Man AI agent, `RandomGhost` ghost AI agent and `smallClassic`
layout, use the following:

`python pacman.py -p GreedyAgent -g RandomGhost -l smallClassic`