"""Upper level cutting plane driver for trilevel solution algorithm.

Drives the upper-level portion of the cutting plane solution algorithm, which
consists of solving a bilevel program whose lower level is the bilevel program
formed by the lower two levels.

All solution algorithms include exactly the same upper level, and differ only
in their solutions of the lower-level problem. Different methods are included
here to select which solution method to use for the lower-level.
"""

import cplex
import time

if __name__ == "__main__":
    import method.milp_cp as milpcp
    import method.lp_cp as lpcp
    import method.lp_dual as lpd
else:
    import solver.method.milp_cp as milpcp
    import solver.method.lp_cp as lpcp
    import solver.method.lp_dual as lpd

#==============================================================================
class UpperLevel:
    """Class to drive the upper-level cutting plane algorithm.

    When an upper-level object is created it must be given a specified type of
    lower-level solution method. The upper-level object then initializes the
    appropriate type of lower-level object and refers to it whenever the main
    cutting plane loop requires solving the lower-level bilevel program. This
    choice also implicitly determines whether the underlying network uses
    binary or linear interdependencies.

    Includes a public method solve_lower() for solving the lower-level problem
    associated with a given upper-level decision. This is used repeatedly
    within the overall cutting plane solution process, but it can also be
    called on its own to evaluate a single defensive deicsion, for example to
    evaluate the optimality gap of a heuristically-chosen defense.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in, method, big_m=1.0e16):
        """Solution object constructor.

        Initializes a lower-level model object corresponding to the chosen
        solution method.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
            method -- Integer argument to indicate which lower-level method to
                apply, indexed as follows:
                    1: MILP cutting plane
                    2: LP cutting plane
                    3: LP duality

        Accepts the following optional keyword arguments:
            big_m -- Large constant for use in the big-M method. Should be
                chosen to be significantly larger than the largest objective
                allowed to be returned by the lower-level submodel. Defaults to
                1.0e16.
        """

        self.Net = net_in # set network reference

        # Initialize the chosen type of lower-level solver
        if method == 1:
            self.LowerLevel = milpcp.MILPCuttingPlane(self.Net)
        elif method == 2:
            self.LowerLevel = lpcp.LPCuttingPlane(self.Net)
        elif method == 3:
            self.LowerLevel = lpd.LPDuality(self.Net)

        # Initialize Cplex object
        self._cplex_setup()

    #--------------------------------------------------------------------------
    def _cplex_setup(self):
        """Initializes Cplex object for the top-level relaxed master problem.

        The top level of the trilevel program's main cutting plane method is a
        relaxed master problem consisting of the defender's minimization MILP.
        It includes an expanding set of constraints based on previously-
        calculated lower level solutions.

        The MILP is initialized with no constraints on the objective value. A
        constraint is added after each solution of the lower-level program.
        """

        # Initialize Cplex object
        self.TopModel = cplex.Cplex()

        # Silence CPLEX output streams
        self.TopModel.set_log_stream(None)
        self.TopModel.set_results_stream(None)
        self.TopModel.set_error_stream(None)
        self.TopModel.set_warning_stream(None)

        # Set as minimization
        self.TopModel.objective.set_sense(
                self.TopModel.objective.sense.minimize)

        ####################################################################################
        ### Include indicator constraints instead of big-M constraints.







    #--------------------------------------------------------------------------
    def solve_upper(self, cutoff=100, lower_cutoff=100, gap=0.01,
                    lower_gap=0.01):
        """Main upper-level cutting plane solution algorithm.

        The main loop of the cutting plane algorithm proceeds until either
        reaching an iteration cutoff or achieving a sufficiently small
        optimality gap, both of which can be adjusted.

        Accepts the following optional keyword arguments:
            cutoff -- Iteration cutoff for the overall cutting plane main loop.
                Defaults to 100.
            gap -- Optimality gap tolerance for the overall cutting plane main
                loop. Defaults to 0.01.
            lower_cutoff -- Iteration cutoff for the lower-level cutting plane
                main loop (if applicable). Defaults to 100.
            lower_gap -- Optimality gap tolerance for the lower-level cutting
                plane main loop (if applicable). Defaults to 0.01.

        Returns a tuple containing the following elements:
            objective -- Objective value of the trilevel program.
            defend -- Vector of defended arcs, as a boolean list.
            destroy -- Vector of destroyed arcs, as a boolean list.
            times -- Tuple of cumulative times spent on various parts of the
                solution process, including: (upper level, lower level)
            iterations -- Tuple of iterations of each cutting plane loop (upper
                level then lower level), with -1 if not applicable (for example
                in the case of the duality lower-level program)
        """

        ### Needs to feed lower_cutoff and lower_gap to submodel solver
        ### (unless its method is type 3)

        ### Placeholder output
        return (0.0, [False for i in range(len(self.Net.arcs))],
                [False for i in range(len(self.Net.arcs))], (0.0, 0.0), (0, 0))

    #--------------------------------------------------------------------------
    def solve_lower(self, defend):
        """Calculates the lower-level solution for a given defensive decision.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Returns a tuple containing the following elements:
            objective -- Objective value of the lower-level bilevel program.
            destroy -- Vector of destroyed arcs, as a boolean list.
            status -- Numerical code to describe the results of the solution
                process, including the following:
                    0: Successful exit with finite objective value.
                    1: Successful exit with infinite objective value.
                    2: Exit due to error.
            iterations -- Number of iterations of the lower-level algorithm's
                cutting plane loop (0 if not applicable).
        """

        # Simply call local lower-level solver with given defense vector

        return self.LowerLevel.solve(defend)

    #--------------------------------------------------------------------------
    def end(self):
        """Closes all internal Cplex models.

        This should be called before the UpperLevel object is discarded. It
        also ends the Cplex models of the lower-level objects.
        """

        self.TopModel.end()
        self.LowerLevel.end()

###############################################################################
### For testing (delete later)

if __name__ == "__main__":
    import method.network.network as net
    TestNet = net.Network("../problems/smallnet.min")
    TestSolver = UpperLevel(TestNet, 2)

    TestSolver.end()
