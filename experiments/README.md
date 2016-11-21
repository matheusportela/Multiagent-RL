# Experiments

This directory contains all experiments compatible with Multiagent RL, which must contain the following modules:

- `adapter.py`: Connects the Multiagent RL framework to the simulator.
- `agents.py`: Reinforcement learning agents that will learn with the given experiment.

Also, an experiment must connect somehow to a simulator, be it implementing the simulation code itself or sending messages to third-party simulators.