# Cart Pole Experiment

Toy experiment, original from [SUTTON R.S., BARTO A.G., ANDERSON C.W. - Neuronline Adaptive Elements That Can Solve Difficult Learning Control Problem](http://www.derongliu.org/adp/adp-cdrom/Barto1983.pdf), where a cart must control a pole to remain stationary.

# Installation

```bash
pip install gym
```

# Modules

- `adapter.py`: Contains the experiment, connecting the [OpenAI Gym](https://gym.openai.com/envs/CartPole-v0#barto83) simulator to our `multiagentrl` package.
- `agents.py`: All agents available to be used in the experiment.
- `plot.py`: Plot experiment results for later analysis.

# Example run

```bash
python simulation.py cart_pole -l 500 -t 50 -a ai -o cart_pole_ai.res -p cart_pole_ai.pol -g --main-thread
```