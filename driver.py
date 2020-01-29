"""Computational trial driver for trilevel network interdiction game.

Drives the computational trial processing by reading a list of problem file
locations and calling the main solver driver for each. Also handles writing the
result logs.
"""

# Note: The current version of this program is equipped only to handle networks
# with parent arcs, since parent nodes require some restructuring when the
# network object is created.

import solver

### This main driver should instantiate a different TrialSolver object for each
### trial instance, calling each of its methods one-at-a-time.

# Handle all of the file accessing and writing. Refer to an external list with
# the names of all files to be read, since this list can have elements
# removed from it as trials are completed
