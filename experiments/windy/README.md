# Windy World Experiment

Toy experiment, original from [SUTTON R.S., BARTO A.G. - Reinforcement Learning, a introduction](https://webdocs.cs.ualberta.ca/~sutton/book/the-book.html), where a single agent must move from the initial position and achieve a goal. Along the way, there is a small lake and, whenever the agent falls in the water, it receives negative rewards. The agent can use a bridge, although random wind may push it into the water.

```
* * * W W * * * * *
S * * * * * * G * *
* * * W W * * * * *
* * * W W * * * * *
* * * * * * * * * *
* * A * * * * * * *
* * * * * * * * * *

where:

S: Initial state
W: Water that gives penalty
G: Goal state
A: Agent current position
*: Free cell
```

It is expected that greedy agents, seeking immediate rewards, will risk going through the bridge whereas conservative ones will go through a further but safer path.

# Modules

- `adapter.py`: Contains the experiment, connecting the Pac-Man simulator to our `multiagentrl` package.
- `agents.py`: All agents available to be used in the experiment.
- `behaviors.py`: Behaviors when learning behavior selection, instead of action selection.
- `features.py`: Features for function approximation.
- `plot.py`: Plot experiment results for later analysis.
- `state.py`: Simulation game state.