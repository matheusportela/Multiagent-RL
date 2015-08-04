#!/usr/bin/env python

import pymas


class HelloWorldAgent(pymas.Agent):
    def on_start(self):
        print '%d Starting' % self.id

    def on_run(self):
        print '%d Running' % self.id

        msg = pymas.Message(receiver=self.id+1, data='Greetings from %d' % self.id)
        self.send_message(msg)
        print 'Sent message to %d' % (self.id+1)
        self.stop()

    def on_receive_message(self, message):
        print 'Received message:'
        print str(message)

    def on_stop(self):
        print '%d Stopped' % self.id


if __name__ == '__main__':
    system = pymas.System()
    system.add_agent(HelloWorldAgent)
    system.add_agent(HelloWorldAgent)
    system.add_agent(HelloWorldAgent)
    system.add_agent(HelloWorldAgent)
    system.run()