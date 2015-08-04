import time

class System(object):
    """Multiagent system central management."""
    def __init__(self):
        self.agents = []
        self.messages = {}
        self.is_running = True

    @property
    def num_agents(self):
        return len(self.agents)

    @property
    def num_messages(self):
        return len(self.messages)

    def add_agent(self, class_):
        agent = class_()
        self.agents.append(agent)
        agent.on_start()
        return agent

    def remove_agent(self, agent):
        agent.on_stop()
        self.agents.remove(agent)

    def send_message(self, message):
        if message.receiver not in self.messages:
            self.messages[message.receiver] = []
        self.messages[message.receiver].append(message.data)

    def _route_messages_from_agent(self, agent):
        for message in agent._unsent_messages:
            self.send_message(message)
        agent._clear_unsent_messages()

    def _route_messages_to_agent(self, agent):
        if agent.id in self.messages:
            for message in self.messages[agent.id]:
                agent.on_receive_message(message)
            del self.messages[agent.id]

    def stop(self):
        self.is_running = False

    def run(self, sleep=0):
        while self.is_running:
            for agent in self.agents:
                self._route_messages_to_agent(agent)
                agent.on_run()
                self._route_messages_from_agent(agent)

                if not agent.is_running:
                    self.remove_agent(agent)

            if self.num_agents == 0:
                self.stop()

            time.sleep(sleep)


class Agent(object):
    """Agent capable of starting, running, and stoping execution."""
    num_agents = 0

    def __init__(self):
        self.id = Agent.num_agents
        self.is_running = True
        self._unsent_messages = []
        Agent.num_agents += 1

    def __eq__(self, other):
        return (self.id == other.id)

    def __ne__(self, other):
        return not (self.id == other.id)

    def on_start(self):
        pass

    def on_run(self):
        pass

    def stop(self):
        self.is_running = False

    def on_stop(self):
        pass

    def on_receive_message(self, message):
        pass

    def send_message(self, message):
        self._unsent_messages.append(message)

    def _clear_unsent_messages(self):
        self._unsent_messages = []


class Message(object):
    """Communication packet used by agents."""
    def __init__(self, receiver=None, data=None):
        self.receiver = receiver
        self.data = data

    def __str__(self):
        return 'Message: %s' % self.data