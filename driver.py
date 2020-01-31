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

# The main solver method should ask for a .txt file which lists a set of
# NETGEN files. The driver then calls solver.py for each of the listed files in
# turn, using all of its solver methods to apply each of the solution
# algorithms, and then printing the results to a specified set of output .txt
# files.
# Include the following outputs (also include an option to overwrite):
# summary.txt
# file, nodes, arcs, int, type, defense, attack, milp_cp_time, milp_obj,
# lp_cp_time, lp_dual_time, lp_obj, lp_milp_obj
# milp_cp_sol.txt
# file, [sol]
# lp_cp_sol.txt
# file, [sol]
# lp_dual_sol.txt
# file, [sol]
