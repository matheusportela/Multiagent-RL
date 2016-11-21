# Pac-Man Experiment

This experiment investigates Reinforcement Learning in a Multiagent System. In order to do such, a [Pac-Man simulator](http://ai.berkeley.edu/classification.html), provided by UC Berkeley, is used to simulate the game. This scenario can be considered both competitive (Pac-Man vs ghosts) and cooperative (ghosts helping each other).

When learning with the Pac-Man, it is expected it will be able to learn how to eat food and capsules more effectively when acquiring experience, even learning to run from ghosts. When ghosts are learning, though, they should be able to learn how to chase de Pac-Man faster.

# Modules

- `adapter.py`: Contains the experiment, connecting the Pac-Man simulator to our `multiagentrl` package.
- `agents.py`: All agents available to be used in the experiment.
- `behaviors.py`: Behaviors when learning behavior selection, instead of action selection.
- `features.py`: Features for function approximation.
- `plot.py`: Plot experiment results for later analysis.
- `state.py`: Simulation game state.