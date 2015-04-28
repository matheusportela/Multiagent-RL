import unittest
import learn


class TestLearn(unittest.TestCase):
    def test_learn_method_raises_not_implemented_error(self):
        state = None
        action = None
        reward = None
        l = learn.Learner()

        with self.assertRaises(NotImplementedError):
            l.learn(state, action, reward)

    def test_select_method_raises_not_implemented_error(self):
        state = None
        l = learn.Learner()

        with self.assertRaises(NotImplementedError):
            l.act(state)


class TestQValues(unittest.TestCase):
    def test_default_num_states(self):
        qv = learn.QValues()

        self.assertEqual(qv.num_states, 0)

    def test_default_num_actions(self):
        qv = learn.QValues()

        self.assertEqual(qv.num_actions, 0)

    def test_initial_num_states(self):
        qv = learn.QValues(num_states=1)

        self.assertEqual(qv.num_states, 1)

    def test_initial_num_actions(self):
        qv = learn.QValues(num_actions=1)

        self.assertEqual(qv.num_actions, 1)

    def test_get_default_q_value(self):
        state = 0
        action = 0
        expected_q_value = 0
        qv = learn.QValues(num_states=1, num_actions=1)

        q_value = qv.get(state, action)

        self.assertEqual(q_value, expected_q_value)

    def test_set_q_value(self):
        state = 0
        action = 0
        q_value = 10
        qv = learn.QValues(num_states=1, num_actions=1)

        qv.set(state, action, q_value)
        set_q_value = qv.get(state, action)

        self.assertEqual(set_q_value, q_value)

    def test_get_best_q_value(self):
        q_values = [[1, 2], [4, 3]]
        qv = learn.QValues(num_states=2, num_actions=2)

        for state, actions in enumerate(q_values):
            for action, value in enumerate(actions):
                qv.set(state, action, value)

        best_q_values = [qv.get_max_value(state) for state in [0, 1]]

        self.assertEqual(best_q_values, [2, 4])

    def test_get_max_action_index(self):
        q_values = [[1, 2], [4, 3]]
        qv = learn.QValues(num_states=2, num_actions=2)

        for state, actions in enumerate(q_values):
            for action, value in enumerate(actions):
                qv.set(state, action, value)

        actions = [qv.get_max_action(state) for state in [0, 1]]

        self.assertEqual(actions, [1, 0])


class TestQLearn(unittest.TestCase):
    def test_default_current_state(self):
        expected_current_state = 0

        ql = learn.QLearner()

        self.assertEqual(ql.current_state, expected_current_state)

    def test_default_discount_factor(self):
        expected_discount_factor = 1

        ql = learn.QLearner()

        self.assertEqual(ql.discount_factor, expected_discount_factor)

    def test_default_learning_rate(self):
        expected_learning_rate = 1

        ql = learn.QLearner()

        self.assertEqual(ql.learning_rate, expected_learning_rate)

    def test_initial_state(self):
        expected_current_state = 2

        ql = learn.QLearner(initial_state=2)

        self.assertEqual(ql.current_state, expected_current_state)

    def test_initial_discount_factor(self):
        expected_discount_factor = 0.8

        ql = learn.QLearner(discount_factor=0.8)

        self.assertEqual(ql.discount_factor, expected_discount_factor)

    def test_initial_learning_rate(self):
        expected_learning_rate = 0.8

        ql = learn.QLearner(learning_rate=0.8)

        self.assertEqual(ql.learning_rate, expected_learning_rate)

    def test_change_state_after_learning(self):
        state = 1
        action = 0
        reward = 0
        ql = learn.QLearner(num_states=3, num_actions=2)

        ql.learn(state, action, reward)

        self.assertEqual(ql.current_state, state)

    def test_learn_with_1_state_action_q_value(self):
        state = 0
        action = 0
        reward = 10
        ql = learn.QLearner(num_states=1, num_actions=1)

        ql.learn(state, action, reward)
        q_value = ql.q_values.get(state, action)

        self.assertEqual(q_value, 10)

    def test_learn_with_several_state_action_q_values(self):
        current_state = 2
        next_state = 4
        action = 3
        reward = 10
        ql = learn.QLearner(initial_state=current_state, num_states=5,
            num_actions=4)

        ql.learn(next_state, action, reward)
        q_value = ql.q_values.get(current_state, action)

        self.assertEqual(q_value, 10)

    def test_learn_two_rewards(self):
        state = 0
        action = 0
        rewards = [5, 10]
        expected_q_value = 15
        ql = learn.QLearner(num_states=1, num_actions=1)

        for reward in rewards:
            ql.learn(state, action, reward)
        q_value = ql.q_values.get(state, action)

        self.assertEqual(q_value, expected_q_value)

    def test_learn_with_discount_factor(self):
        state = 0
        action = 0
        expected_q_value = 15
        rewards = [10, 10]
        ql = learn.QLearner(num_states=1, num_actions=1, discount_factor=0.5)

        for reward in rewards:
            ql.learn(state, action, reward)
        q_value = ql.q_values.get(state, action)

        self.assertEqual(q_value, expected_q_value)

    def test_learn_with_learning_rate(self):
        state = 0
        action = 0
        expected_q_value = 10
        rewards = [10, 10]
        ql = learn.QLearner(num_states=1, num_actions=1, learning_rate=0.5)

        for reward in rewards:
            ql.learn(state, action, reward)
        q_value = ql.q_values.get(state, action)

        self.assertEqual(q_value, expected_q_value)

    def test_select_action_after_one_learning_step(self):
        state, action, reward = 1, 1, 10
        ql = learn.QLearner(num_states=2, num_actions=2, learning_rate=0.5)

        ql.learn(state, action, reward)
        selected_action = ql.act(0)

        self.assertEqual(action, selected_action)

    def test_select_action_after_several_learning_steps(self):
        ql = learn.QLearner(num_states=2, num_actions=2, learning_rate=0.5,
            discount_factor=0.5)
        steps = [(1, 1, 10),
                 (0, 0, 100),
                 (1, 1, 5)]
        excepted_actions = [(0, 1),
                            (1, 0)]

        for state, action, reward in steps:
            ql.learn(state, action, reward)

        for state, excepted_action in excepted_actions:
            action = ql.act(state)
            self.assertEqual(action, excepted_action)


class TestSystemAdapter(unittest.TestCase):
    def test_run_method_raises_not_implemented_error(self):
        measurements = None
        sa = learn.SystemAdapter()

        with self.assertRaises(NotImplementedError):
            sa.run(measurements)


class TestPacmanMeasurements(unittest.TestCase):
    def test_pack_measurements(self):
        measurements = learn.PacmanMeasurements(
            pacman_position=(10, 10),
            ghosts_positions=[
                (0, 0),
                (0, 1),
                (2, 5),
            ],
            reward=5,
        )

        self.assertEqual(measurements.pacman_position, (10, 10))
        self.assertEqual(measurements.ghosts_positions, [(0, 0), (0, 1), (2, 5)])
        self.assertEqual(measurements.reward, 5)


class TestPacmanActions(unittest.TestCase):
    def test_pack_actions(self):
        actions = learn.PacmanActions(
            pacman_action='North',
            ghosts_actions=[
                'South',
                'West',
                'East',
            ],
        )

        self.assertEqual(actions.pacman_action, 'North')
        self.assertEqual(actions.ghosts_actions, ['South', 'West', 'East'])


if __name__ == '__main__':
    unittest.main()