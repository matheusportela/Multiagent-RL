"""Plot simulation results using the plot module in the experiment package.

It assumes the experiment has a package located at "experiments/" containing a
module called "adapter.py". For example, an experiment called "pacman" must
have at least the structure:
experiments/
  pacman/
      __init__.py
      plot.py
"""

import argparse
from importlib import import_module
import sys

if __name__ == '__main__':
    # Parse module name
    parser = argparse.ArgumentParser(
        description='Plot simulation results.',
        add_help=False)
    parser.add_argument(
        'module', help='module to execute the simulation (e.g. "pacman")')

    if len(sys.argv) < 2 or sys.argv[1] in ['-h', '--help']:
        parser.print_help()
        exit(1)

    args, unknown = parser.parse_known_args()

    # Parse module-specific arguments
    plot_module = import_module('experiments.' + args.module + '.plot')
    plot_parser = plot_module.build_parser()
    plot_args = plot_parser.parse_args(unknown)
    plot_module.plot(plot_args)
