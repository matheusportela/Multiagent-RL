#!/usr/bin/env python

import unittest
import pymas


class TestAgent(unittest.TestCase):
    def test_agent_id(self):
        agent0 = pymas.Agent()
        agent1 = pymas.Agent()
        agent2 = pymas.Agent()

        self.assertEqual(agent0.id, 0)
        self.assertEqual(agent1.id, 1)
        self.assertEqual(agent2.id, 2)


class TestSystem(unittest.TestCase):
    def test_add_agent(self):
        system = pymas.System()

        system.add_agent(pymas.Agent)
        self.assertEqual(system.num_agents, 1)

        system.add_agent(pymas.Agent)
        self.assertEqual(system.num_agents, 2)

        agent = system.add_agent(pymas.Agent)
        self.assertTrue(agent in system.agents)

    def test_remove_agent(self):
        system = pymas.System()

        system.add_agent(pymas.Agent)
        agent = system.add_agent(pymas.Agent)
        system.add_agent(pymas.Agent)

        system._remove_agent(agent)

        self.assertEqual(system.num_agents, 2)
        self.assertTrue(agent not in system.agents)

    def test_run(self):
        class TestAgent(pymas.Agent):
            def on_run(self):
                self.stop()

        system = pymas.System()

        system.add_agent(TestAgent)
        system.run()

        self.assertEqual(system.num_agents, 0)

    def test_messages(self):
        class TestAgent(pymas.Agent):
            def on_run(self):
                self.stop()

        system = pymas.System()
        agent = system.add_agent(TestAgent)
        message = pymas.Message(receiver=agent.id)

        system._send_message(message)
        self.assertEqual(system.num_messages, 1)

        system.run()
        self.assertEqual(system.num_messages, 0)


if __name__ == '__main__':
    unittest.main()
