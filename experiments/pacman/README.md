# Pac-Man Experiment

This experiment contains the code used in the undergraduate thesis in Mechatronics Engineering, at the [University of Brasilia](http://www.unb.br), entitled ["Behavior selection for multiple autonomous agents with reinforcement learning in stochastic environments"](https://db.tt/TIvw9aXx).

The idea is to have multiple simulated robotic agents learning to select appropriate behaviors in a stochastic environment. The uncertainty of a state is handled through [Bayesian Programming](https://en.wikipedia.org/wiki/Bayesian_programming), and the agents learn by applying [Q-learning](https://en.wikipedia.org/wiki/Q-learning) with [function approximation](http://people.csail.mit.edu/agf/Files/13FTML-RLTutorial.pdf).

Currently, the approach is tested in a predator-prey problem using a modified version of the [Pac-Man game](https://en.wikipedia.org/wiki/Pac-Man) with introduced uncertainties. Therefore, this simplified multi-agent situation aims to answer the following question: *can the ghosts learn to get the Pac-Man?*

## Installation

The Pac-Man AI Projects provides six Pac-Man-like simulators that are free to use for educational purposes. The one we will be using is [Project 5: Classification](http://ai.berkeley.edu/classification.html), which provides an arena mimicking the complete Pac-Man game, including various ghosts. This scenario can be considered both competitive (Pac-Man vs ghosts) and cooperative (ghosts helping each other).

When learning with the Pac-Man, it is expected it will be able to learn how to eat food and capsules more effectively when acquiring experience, even learning to run from ghosts. When ghosts are learning, though, they should be able to learn how to chase de Pac-Man faster.

# Modules

- `adapter.py`: Contains the experiment, connecting the Pac-Man simulator to our `multiagentrl` package.
- `agents.py`: All agents available to be used in the experiment.
- `behaviors.py`: Behaviors when learning behavior selection, instead of action selection.
- `features.py`: Features for function approximation.
- `plot.py`: Plot experiment results for later analysis.
- `state.py`: Simulation game state.