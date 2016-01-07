# Multiagent-RL

## Introduction

This repository contains the code used in the undergraduate thesis in Mechatronics Engineering, at the [University of Brasilia](http://www.unb.br), entitled ["Reinforcement Learning applied to behaviour selection in multiple autonomous agents"](https://db.tt/TIvw9aXx).

The idea is to have multiple simulated robotic agents learning to select appropriate behaviors in a stochastic environment. The uncertainty of a state is handled through [Bayesian Programming](https://en.wikipedia.org/wiki/Bayesian_programming), and the agents learn by applying [Q-learning](https://en.wikipedia.org/wiki/Q-learning) with [function approximation](http://people.csail.mit.edu/agf/Files/13FTML-RLTutorial.pdf).

Currently, the approach is tested in a predator-prey problem using a modified version of the [Pac-Man game](https://en.wikipedia.org/wiki/Pac-Man) with introduced uncertainties. Therefore, this simplified multi-agent situation aims to answer the following question: __can the ghosts learn to get the Pac-Man?__

## Code structure

The entire system was programmed in [Python 2.7](http://www.python.org) and used [The Pac-Man AI Projects](http://ai.berkeley.edu/project_overview.html), by UC Berkeley, as the game simulator.

Several modules compose the system, which are presented below with their defined roles.

### Controller

The `controller` script implements logic to control learning and action selection for each agent by receiving simulator messages, routing them to the appropriate agent, and sending agents' actions to the simulator. In order to communicate with the simulator module via messages, the script instantiates a server object.

### Simulator

The `simulator` script executes the Pac-Man simulator, extracts the state from the current game state, communicates the current state with the `controller` process through a client instance, receives actions from `controller`, and save experiment results. This script is tightly coupled to the Pac-Man simulator and must be modified to use this project in new scenarios.

### Agents

The `agents` module contains agents implementation for action selection. By implementing the `choose_action` method, the agent instance must return a valid action according to its execution environment. For instance, the following agent always walk North assuming that `'North'` is a valid representation of an action that moves it upwards in a particular simulator.

```python
class NorthAgent(object):
    def choose_action(self):
        return 'North'
```

A more complex agent may use a learning algorithm to select the best possible action for the given state, such as the following example.

```python
class LearningAgent(object):
    def __init__(self, learning_algorithm):
        self.learning_algorithm = learning_algorithm
    def choose_action(self, state):
        return learning_algorithm.act(state)
```

The module also contains behaviors, which are pre-defined reactive action selection processes. For instance, the flee behavior always select the action that moves the agent away from its enemies. On the other hand, a random behavior, such as presented below, randomly selects any legal action for the given state.

An agent can, therefore, use behaviors to select actions and even learn to select the most appropriate behavior for the given state.
```python
class RandomBehavior(object):
    def __call__(self, state, legal_actions):
        return random.choice(legal_actions)

class BehaviorAgent(object):
    def __init__(self):
        self.behavior = RandomBehavior()

    def choose_action(self, state, legal_actions):
        self.behavior(state, legal_actions)
```

### Learning

The `learning` module stores general-purpose reinforcement learning algoriths. Every RL algorithm must inherit from the `LearningAlgorithm` class and implement two methods:

* `learn(self, state, action, reward)`: Adapts according to the current state representation, the last performed action, and a numerical reward value.
* `act(self, state)`: Selects an action for the current state.

### Communication

The `communication` module implement two classes: `Server` and `Client`. By using the `ZeroMQ` package, client-server architecture is easily incorporated into the decision process cycle using `recv` and `send` methods to receive and send strings.

A server, configured with TCP/IP address, may receive and answer toany number of clients messages. However, a client can only connect to a single server. Due to a ZeroMQ restriction, in this architecture, the client must send a message first and, in sequence, receive a server reply. Should the server not be able to reply the client, communication is lost.

The following code implements a client-server architecture where the client sends `Client data` and the server replies `Server data`:

```python
# Server-side script
import communication as comm

server = comm.Server()

recv_data = server.recv()
print 'Received "{}"'.format(recv_data)

send_data = 'Server data'
server.send(send_data)
print 'Sent "{}"'.format(send_data)
```

```python
# Client-side script
import communication as comm

client = comm.Client()

send_data = 'Client data'
client.send(send_data)
print 'Sent "{}"'.format(send_data)

recv_data = client.recv()
print 'Received "{}"'.format(recv_data)
```

Server output:

```
Received "Client data"
Sent "Server data"
```

Client output:

```
Sent "Client data"
Received "Server data"
```

### Messages

The `messages` module stores all kinds of messages used in the Pac-Man application. All messages inherit from `BaseMessage` and have a respective type.

For instance, `AckMessage` is used to communicate the server received the client message but has no special reply.

```python
ACK = 'Ack'

class AckMessage(BaseMessage):
    def __init__(self):
        super(AckMessage, self).__init__(msg_type=ACK)
```

### State

The `state` module contains the `GameState` class, which holds information about the Pac-Man simulation current state.

In order to incorporate stochastic information, the `Map` class stores probabilities in each cell and allow Bayesian approaches with `observe` and `predict` methods, according to Bayesian Programming theory. `observe` incorporates new measurements into the map probabilities, whereas `predict` updates the probability without using sensor measurements. `Map` also implements access graphs to take obstacles into account when predicting movements and calculating distances.

### Plot

The `plot` script allows visualization for simulation data. It plots the scores, probability of selecting each behavior, and game duration.

## Installation

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

## Usage

In order to run the system, two processes are necessary: one that implements agents intelligence, in our case `controller.py`, and another that provides an interface to the real agents. Since we are using the Pacman simulator, we interface it in the module `simulator.py`.

First, it's necessary to run the `controller.py` script, which will launch a server that listens to simulator messages, process them by learning with new information, and returns actions for each agent.

```bash
python controller.py
```

Next, we can launch the Pac-Man simulation with default settings by invoking the command below.

```bash
python simulator.py
```

The simulation may be customized with several implemented flags. Check all available settings by executing:

```bash
python simulator.py -h
```

If configured to save results into a file, the `plot.py` script can be used to visualize simulation results. For instance, the following command plots all graphs for a `results.txt` file.

```bash
python plot.py -i results.txt
```
