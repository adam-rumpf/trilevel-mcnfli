"""Driver for the various solution methods of the trilevel game.

Calls each of the solution algorithms for a given problem instance, and also
applies the LP relaxation solution to the MILP to evaluate the optimality gap.

The functions in this module can be used to call a particular solution method
once, and are used by the main driver to call all available solution methods
for comparison.
"""

if __name__ == "__main__":
    import solver.upper_cp as ucp
    import solver.method.milp_cp as milpcp
    import solver.method.lp_cp as lpcp
    import solver.method.lp_dual as lpdual
    import solver.method.network.network as net

# Include a Solver class that loads a Network object and includes methods to
# call each individual solver in turn (by calling the upper_cp solver with
# different options). Each of these individual solves should be handled by an
# object that is instantiated and then deleted in order to free up memory (use
# gc.collect()). Be sure to use scope to remove things from memory.

# Include an overall timer, but expect timers to be returned by the upper-
# level solver.
