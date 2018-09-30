# Multiagent-RL

## Introduction

This repository contains the code used in the undergraduate thesis in Mechatronics Engineering, at the [University of Brasilia](http://www.unb.br), entitled ["Behavior selection for multiple autonomous agents with reinforcement learning in stochastic environments" (Portuguese only)](http://bdm.unb.br/bitstream/10483/15302/1/2015_MatheusVieiraPortela_tcc.pdf).

The idea is to have multiple simulated robotic agents learning to select appropriate behaviors in a stochastic environment. The uncertainty of a state is handled through [Bayesian Programming](https://en.wikipedia.org/wiki/Bayesian_programming), and the agents learn by applying [Q-learning](https://en.wikipedia.org/wiki/Q-learning) with [function approximation](http://people.csail.mit.edu/agf/Files/13FTML-RLTutorial.pdf).

Currently, the approach is tested in a predator-prey problem using a modified version of the [Pac-Man game](https://en.wikipedia.org/wiki/Pac-Man) with introduced uncertainties. Therefore, this simplified multi-agent situation aims to answer the following question: __can the ghosts learn to get the Pac-Man?__

## Installation

 The Pac-Man AI Projects provides six Pac-Man-like simulators that are free to use for educational purposes. The one we will be using is [Project 5: Classification](http://ai.berkeley.edu/classification.html), which provides an arena mimicking the complete Pac-Man game, including various ghosts.

This project requires the following Python packages:

* *[Tkinter](https://docs.python.org/2/library/tkinter.html)*: graphical user interfaces
* *[ZeroMQ](http://zeromq.org/)*: interprocess communication
* *[Matplotlib](http://matplotlib.org/)*: graphics plotting
* *[Numpy](http://www.numpy.org/)*: numerical computation

### GNU/Unix

This assumes a GNU/Unix distribution (Ubuntu), but everything is in Pỳthon so the setup shouldn't be too different for other platforms.

Install by running the following commands.

```bash
sudo apt-get install python python-dev python-pip python-tk libzmq-dev python-matplotlib
sudo pip install pyzmq
```

### Mac OS X

Installing on an OS X distribution requires a special setup using Homebrew and the XCode.

* *[Homebrew](http://brew.sh)*: package manager
* *[XCode](https://developer.apple.com/xcode/)*: IDE developed by Apple

XCode shall be downloaded from the provided link or in your App Store. Then you need to run the following commands.

```bash
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew install python --with-tcl-tk --enable-threads --with-x11
pip install matplotlib bumpy pyzmq
```

## Documentation

All documentation are kept in [our wiki](https://github.com/matheusportela/Multiagent-RL/wiki), with usage examples and modules explanations.

## Publications

* Undergraduate thesis: [PORTELA, M.V. - Behavior selection for multiple autonomous agents with reinforcement learning in stochastic environments - University of Brasilia - Brasilia, Brazil - 2015](http://bdm.unb.br/bitstream/10483/15302/1/2015_MatheusVieiraPortela_tcc.pdf)
* Conference paper: [PORTELA, M. V., RAMOS, G. N. - State estimation and reinforcement learning for behavior selection in stochastic multiagent systems - Proceedings of SBGames 2015 - Teresina, Brazil - 2015](http://www.sbgames.org/sbgames2015/anaispdf/computacao-short/147936.pdf)
