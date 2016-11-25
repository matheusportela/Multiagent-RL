# Multiagent RL

## Introduction

**Multiagent RL** is a framework to run reinforcement learning experiment with multiple agents. There are other simulators and frameworks for reinforcement learning, such as [OpenAI Gym](https://gym.openai.com), [Arcade Learning Environment](http://www.arcadelearningenvironment.org), [PyBrain](http://pybrain.org) and [Maja](http://mmlf.sourceforge.net), and they are great. However, they are mostly focused in single agent reinforcement learning scenarios and adapting them to multiagent systems isn't trivial.

This code was used in the undergraduate thesis in Mechatronics Engineering, at the [University of Brasilia](http://www.unb.br), entitled ["Behavior selection for multiple autonomous agents with reinforcement learning in stochastic environments"](https://db.tt/TIvw9aXx).

## Experiments

The [experiments directory](experiments/) contains code of simulations that have already been implemented with Multiagent RL. Take a look to better understand how this project can be useful in your case.

- [Windy World](experiments/windy/): single agent learning to reach a goal position without falling in the water
- [Pac-Man](experiments/pacman/): Pac-Man game where ghosts learn to capture the Pac-Man

## Installation

This project requires the following Python packages:

* *[Tkinter](https://docs.python.org/2/library/tkinter.html)*: graphical user interfaces
* *[ZeroMQ](http://zeromq.org/)*: interprocess communication
* *[Matplotlib](http://matplotlib.org/)*: graphics plotting
* *[Numpy](http://www.numpy.org/)*: numerical computation

### GNU/Unix

This assumes a GNU/Unix distribution (Ubuntu), but everything is in Python so the setup shouldn't be too different for other platforms.

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
pip install matplotlib numpy pyzmq
```

## Documentation

All documentation are kept in [our wiki](https://github.com/matheusportela/Multiagent-RL/wiki), with usage examples and modules explanations.

## Publications

* Undergraduate thesis: [PORTELA, M.V. - Behavior selection for multiple autonomous agents with reinforcement learning in stochastic environments - University of Brasilia - Brasilia, Brazil - 2015](https://db.tt/TIvw9aXx)
* Conference paper: [PORTELA, M. V., RAMOS, G. N. - State estimation and reinforcement learning for behavior selection in stochastic multiagent systems - Proceedings of SBGames 2015 - Teresina, Brazil - 2015](http://www.sbgames.org/sbgames2015/anaispdf/computacao-short/147936.pdf)
