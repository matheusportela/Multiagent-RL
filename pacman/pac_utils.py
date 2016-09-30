#  -*- coding: utf-8 -*-
##    @package pac_utils.py
#      @author Guilherme N. Ramos (gnramos@unb.br)
#
#


class Logger(object):
    '''A simple logger.'''
    def __init__(self, owner):
        self.name = owner.__class__.__name__

    def log(self, msg):
        '''Logs the given message.'''
        print('[{}] {}'.format(self.name, msg))


class Experiment(object):
    def __init__(self):
        self.logger = Logger(self)

    def cleanup(self):
        '''Cleanup conditions for starting an experiment.'''
        raise NotImplementedError('Experiment must be cleaned up.')

    def execute_step(self):
        '''Executes one step in the experiment.'''
        raise NotImplementedError('Experiment must execute a step.')

    def run(self, steps):
        '''Setups the experiment, runs the given number of steps and then
        cleans up after itself.
        '''
        self.setup()
        for x in range(steps):
            self.logger.log('Run #{} (of {})'.format(x + 1, steps))
            self.execute_step()
        self.cleanup()

    def setup(self):
        '''Setup conditions for starting an experiment.'''
        raise NotImplementedError('Experiment must be setup.')
