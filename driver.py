"""Computational trial driver for trilevel network interdiction game.

Drives the computational trial processing by reading a list of problem file
locations and calling the main solver driver for each.
"""

import solver
import solver.upper_cp as ucp
import solver.method.milp_cp as milpcp
import solver.method.lp_cp as lpcp
import solver.method.lp_dual as lpdual
import solver.method.network.network as net

# Structural plans:
# Main driver reads project files and organizes all of the things that need to
# be done with it. All of the subroutines come into play only after a specific
# trial instance has been decided on.
# Main solver is called by the driver and handles calling all of the solution
# subroutines.
# Cutting plane MILP solver.
# Cutting plane LP solver.
# Duality LP solver.

# Get a bit more of a placeholder skeleton going and then develop each solver
# on its own branch.

# This program (the main trial driver) needs to process the example files one-
#   at-a-time, drive the complete set of trials for each, and output the
#   results to a central file.
# The main solver driver (solver.py) should store the information related to
#   the current problem instance, including handling the read-in and the
#   network object. It will also call each of the solution methods, time them,
#   and get the results.
# The upper-level solver (upper_cp.py) drives the upper level cutting plane
#   method common to all solution methods and trials. The lower-level portion
#   can be treated as a black box that just produces an objective value and an
#   attack vector.
# All three lower-level solvers (milp_cp.py, lp_cp.py, and lp_dual.py) should
#   act the same way in that they should initialize a Cplex object when they
#   start, modify that Cplex object based on the upper-level defensive vector,
#   and then solve their respective version of the model to return the
#   objective and attack vector.
