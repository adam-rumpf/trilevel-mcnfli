"""Duality solution algorithm for the lower-level bilevel program (LP).

Includes an LPDuality class which applies the duality solution method given a
protection vector. Returns the objective value and attack vector obtained from
the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the LP
relaxation of the binary interdependence model.
"""

### Currently consits entirely of placeholders.

#==============================================================================
class LPDuality:
    """Class to implement the duality method for the LP lower model.

    This class maintains all attributes relevant only to the lower-level
    program.

    Its main public solve() method accepts and upper-level defense vector and
    solves the corresponding version of the lower-level bilevel model,
    returning the objective value and the attack decision vector.

    This class also includes a local Cplex object to represent the lower-level
    program. The object remains active for the entirety of the trilevel
    solution algorithm as the lower-level program is repeatedly solved and re-
    solved, which saves time since previous results may be retained.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in):
        """LP duality solution object constructor.

        Ininitializes the Cplex object associated with the lower-level program.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
        """

        self.Net = net_in # set reference to network object

    #--------------------------------------------------------------------------
    def solve(self, defend, cutoff=None, gap=None, cplex_epsilon=0.001):
        """Lower-level program solution method.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Accepts the following optional keyword arguments:
            cutoff -- Placeholder which does nothing in the duality algorithm,
                used to standardize input format for all solvers.
            gap -- Placeholder which does nothing in the duality algorithm,
                used to standardize input format for all solvers.
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
            zero -- Placeholder value of 0, which for the other solution
                algorithms would be the number of cutting plane iterations.
        """

        ### Placeholder output
        return (0.0, [False for i in range(len(self.Net.arcs))], 0, 0)
