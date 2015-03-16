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

Start installing by downloading the `.zip` file located [here](http://ai.berkeley.edu/classification.html). Not all
files are necessary for our purposes since most of them are evaluation features. Here is the list of files that should
be maintained, the others can be safely discarded.

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

A stripped-down version of the simulator can be downloaded [here](https://mega.co.nz/#!otcGnJAb!IZ6MLmS2fMwu2MPyGWKCGhJMRul4SgiPelk08wEjOP4).

## Usage
Simply run the simulator with the following command:

`python pacman.py`

Several aspects can be configured, such as Pac-Man and ghosts AI agents, number of ghosts, game layout etc. Visualize
all possible settings with the following command:

`python pacman.py -h`

For instance, to run the game with the `GreedyAgent` Pac-Man AI agent, `RandomGhost` ghost AI agent and `smallClassic`
layout, use the following:

`python pacman.py -p GreedyAgent -g RandomGhost -l smallClassic`
