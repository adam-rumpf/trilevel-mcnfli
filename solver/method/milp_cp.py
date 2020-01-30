"""Cutting plane solution algorithm for the lower-level bilevel program (MILP).

Includes a MILPCuttingPLane class which applies the cutting plane solution
method given a protection vector. Returns the objective value and attack vector
obtained from the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the MILP
binary interdependence model.
"""

### Currently consits entirely of placeholders.

#==============================================================================
class MILPCuttingPlane:
    """Class to implement the cutting plane method for the MILP lower model.

    This class also includes a local Cplex object to represent the lower-level
    program. The object remains active for the entirety of the trilevel
    solution algorithm as the lower-level program is repeatedly solved and re-
    solved, which saves time since previous results may be retained.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in):
        """MILP cutting plane solution object constructor.

        Ininitializes the Cplex object associated with the lower-level program.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
        """

        ### Include optional keyword arguments for CP loop limits.

        self.Net = net_in # set reference to network object

    #--------------------------------------------------------------------------
    def solve(self, defend, cutoff=100, gap=0.01, cplex_epsilon=0.001):
        """Lower-level program solution method.

        The main loop of the cutting plane algorithm proceeds until either
        reaching an iteration cutoff or achieving a sufficiently small
        optimality gap, both of which can be adjusted.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Accepts the following optional keyword arguments:
            cutoff -- Iteration cutoff for the overall cutting plane main loop.
                Defaults to 100.
            gap -- Optimality gap tolerance for the overall cutting plane main
                loop. Defaults to 0.01.
            cplex_epsilon -- Epsilon value for CPLEX solver's cleanup method.
                Values generated by the solver falling below this absolute
                value are deleted between solves. Defaults to 0.001.

        Returns a tuple containing the following elements:
            objective -- Objective value of the lower-level bilevel program.
            destroy -- Vector of destroyed arcs, as a boolean list.
            status -- Numerical code to describe the results of the solution
                process, including the following:
                    0: Successful exit with finite objective value.
                    1: Successful exit with infinite objective value.
                    2: Exit due to error.
            iterations -- Number of iterations of main cutting plane loop.
        """

        ### Placeholder output
        return (0.0, [False for i in range(len(self.Net.arcs))], 0, 0)
