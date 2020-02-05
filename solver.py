"""Driver for the various solution methods of the trilevel game.

Calls each of the solution algorithms for a given problem instance, and also
applies the LP relaxation solution to the MILP to evaluate the optimality gap.

The functions in this module can be used to call a particular solution method
once, and are used by the main driver to call all available solution methods
for comparison.
"""

import gc
import time

import upper.upper_cp as ucp
import upper.lower.milp_lp_cp as milpcp
import upper.lower.network.network as net

#==============================================================================
class TrialSolver:
    """Class for driving the computational trial process.

    Loads the given trial network file upon initialization. Includes public
    methods for running each individual required test, each of which returns
    the objective, defender/attacker decisions, and times and iteration counts
    for use by the overall trial driver.

    There is also an internal attribute that stores the value of the most
    recent defensive vector returned by one of the solvers.
    """

    #--------------------------------------------------------------------------
    def __init__(self, file):
        """Trial evaluation object constructor.

        Requires the following positional arguments:
            file -- Complete file path for the NETGEN file containing the
                network definition.
        """

        # Build the network for the given trial file
        self.Net = net.Network(file)

        # Initialize memory object for most recent upper-level solution
        self.last_defense = [False for i in range(len(self.Net.arcs))]

    #--------------------------------------------------------------------------
    def solve_milp_cutting_plane(self, upper_cutoff=100, lower_cutoff=100,
                                 upper_gap=0.01, lower_gap=0.01, big_m=1.0e16,
                                 small_m=1.0e10):
        """Solves the trilevel model using the MILP cutting plane algorithm.

        Creates an upper-level solver object associated with the problem
        network, and equipped to solve the model version with binary
        interdependencies via the cutting plane lower-level algorithm.

        Accepts the following optional keyword arguments:
            upper_cutoff -- Iteration cutoff for the upper-level cutting plane
                main loop. Defaults to 100.
            upper_gap -- Optimality gap tolerance for the upper-level cutting
                plane main loop. Defaults to 0.01.
            lower_cutoff -- Iteration cutoff for the lower-level cutting plane
                main loop (if applicable). Defaults to 100.
            lower_gap -- Optimality gap tolerance for the lower-level cutting
                plane main loop (if applicable). Defaults to 0.01.
            big_m -- Large constant for use in the big-M method. Should be
                chosen to be significantly larger than the largest objective
                allowed to be returned by the lower-level submodel. Defaults to
                1.0e16.
            small_m -- Big-M constant for use in the lower-level model. Should
                still be larger than any reasonable values produced by the
                solution algorithm, but significantly smaller than big_m.
                Defaults to 1.0e10.

        Returns a tuple containing the following elements:
            objective -- Objective value of the trilevel program.
            defend -- Vector of defended arcs, as a boolean list.
            destroy -- Vector of destroyed arcs, as a boolean list.
            times -- Tuple of cumulative times spent on various parts of the
                solution process, including: (total, upper level, lower level)
            iterations -- Tuple of iterations of each cutting plane loop (upper
                level then lower level), with -1 if not applicable (for example
                in the case of the duality lower-level program)
        """

        # Collect garbage
        gc.collect()

        # Initialize a temporary upper-level solver object
        Up = ucp.UpperLevel(self.Net, 1, big_m=big_m, small_m=small_m)

        # Apply solver, save defense, and time entire process
        tot = time.time()
        (obj, self.last_defense, attack, times, iterations, status) = Up.solve(
                cutoff=upper_cutoff, lower_cutoff=lower_cutoff, gap=upper_gap,
                lower_gap=lower_gap)
        tot = time.time() - tot

        # End solver
        Up.end()

        # Return results
        return (obj, self.last_defense, attack, (tot, times[0], times[1]),
                iterations)

    #--------------------------------------------------------------------------
    def solve_lp_cutting_plane(self, upper_cutoff=100, lower_cutoff=100,
                               upper_gap=0.01, lower_gap=0.01, big_m=1.0e16,
                               small_m=1.0e10):
        """Solves the trilevel model using the LP cutting plane algorithm.

        Creates an upper-level solver object associated with the problem
        network, and equipped to solve the model version with linear
        interdependencies via the cutting plane lower-level algorithm.

        Accepts the following optional keyword arguments:
            upper_cutoff -- Iteration cutoff for the upper-level cutting plane
                main loop. Defaults to 100.
            upper_gap -- Optimality gap tolerance for the upper-level cutting
                plane main loop. Defaults to 0.01.
            lower_cutoff -- Iteration cutoff for the lower-level cutting plane
                main loop (if applicable). Defaults to 100.
            lower_gap -- Optimality gap tolerance for the lower-level cutting
                plane main loop (if applicable). Defaults to 0.01.
            big_m -- Large constant for use in the big-M method. Should be
                chosen to be significantly larger than the largest objective
                allowed to be returned by the lower-level submodel. Defaults to
                1.0e16.
            small_m -- Big-M constant for use in the lower-level model. Should
                still be larger than any reasonable values produced by the
                solution algorithm, but significantly smaller than big_m.
                Defaults to 1.0e10.

        Returns a tuple containing the following elements:
            objective -- Objective value of the trilevel program.
            defend -- Vector of defended arcs, as a boolean list.
            destroy -- Vector of destroyed arcs, as a boolean list.
            times -- Tuple of cumulative times spent on various parts of the
                solution process, including: (total, upper level, lower level)
            iterations -- Tuple of iterations of each cutting plane loop (upper
                level then lower level), with -1 if not applicable (for example
                in the case of the duality lower-level program)
        """

        # Collect garbage
        gc.collect()

        # Initialize a temporary upper-level solver object
        Up = ucp.UpperLevel(self.Net, 2, big_m=big_m, small_m=small_m)

        # Apply solver, save defense, and time entire process
        tot = time.time()
        (obj, self.last_defense, attack, times, iterations, status) = Up.solve(
                cutoff=upper_cutoff, lower_cutoff=lower_cutoff, gap=upper_gap,
                lower_gap=lower_gap)
        tot = time.time() - tot

        # End solver
        Up.end()

        # Return results
        return (obj, self.last_defense, attack, (tot, times[0], times[1]),
                iterations)

    #--------------------------------------------------------------------------
    def solve_lp_duality(self, upper_cutoff=100, upper_gap=0.01, big_m=1.0e16):
        """Solves the trilevel model using the LP duality algorithm.

        Creates an upper-level solver object associated with the problem
        network, and equipped to solve the model version with linear
        interdependencies via the duality lower-level algorithm.

        Accepts the following optional keyword arguments:
            upper_cutoff -- Iteration cutoff for the upper-level cutting plane
                main loop. Defaults to 100.
            upper_gap -- Optimality gap tolerance for the upper-level cutting
                plane main loop. Defaults to 0.01.
            big_m -- Large constant for use in the big-M method. Should be
                chosen to be significantly larger than the largest objective
                allowed to be returned by the lower-level submodel. Defaults to
                1.0e16.

        Returns a tuple containing the following elements:
            objective -- Objective value of the trilevel program.
            defend -- Vector of defended arcs, as a boolean list.
            destroy -- Vector of destroyed arcs, as a boolean list.
            times -- Tuple of cumulative times spent on various parts of the
                solution process, including: (total, upper level, lower level)
            iterations -- Tuple of iterations of each cutting plane loop (upper
                level then lower level), with -1 if not applicable (for example
                in the case of the duality lower-level program)
        """

        # Collect garbage
        gc.collect()

        # Initialize a temporary upper-level solver object
        Up = ucp.UpperLevel(self.Net, 3, big_m=big_m)

        # Apply solver, save defense, and time entire process
        tot = time.time()
        (obj, self.last_defense, attack, times, iterations, status) = Up.solve(
                cutoff=upper_cutoff, gap=upper_gap)
        tot = time.time() - tot

        # End solver
        Up.end()

        # Return results
        return (obj, self.last_defense, attack, (tot, times[0], times[1]),
                iterations)

    #--------------------------------------------------------------------------
    def solve_milp_defend(self, defend, cutoff=100, gap=0.01, big_m=1.0e10):
        """Calculates the MILP solution for a given defensive decision.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Accepts the following optional keyword arguments:
            cutoff -- Iteration cutoff for the overall cutting plane main loop.
                Defaults to 100. Used only for cutting plane lower-level
                methods.
            gap -- Optimality gap tolerance for the overall cutting plane main
                loop. Defaults to 0.01. Used only for cutting plane lower-
                level methods.
            big_m -- Large constant for use in the big-M method. Should be
                chosen to be significantly larger than the largest objective
                allowed to be returned by the lower-level submodel. Defaults to
                1.0e10.

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

        # Collect garbage
        gc.collect()

        # Initialize a temporary upper-level solver object
        Up = ucp.UpperLevel(self.Net, 1, small_m=big_m)

        # Get lower level solution for specified defense
        out = Up.lower_solve(defend, cutoff=cutoff, gap=gap)

        # End solver
        Up.end()

        # Return solution
        return out

    #--------------------------------------------------------------------------
    def solve_milp_initial(self):
        """Calculates the initial MILP solution.

        Calculates the solution of the lower-level binary interdependency
        model with no defensive or attack decisions made.

        Returns a tuple containing the following elements:
            objective -- Objective value of the lower-level bilevel program.
            status -- Numerical code to describe the results of the solution
                process, including the following:
                    0: Successful exit with feasible MILP.
                    1: Successful exit with infeasible MILP.
                    2: Exit due to error.
        """

        # Initialize a temporary lower-level solver object
        Milp = milpcp.LLCuttingPlane(self.Net, 1)

        # Get lower level solution with no attacks made
        (obj, _, feas) = Milp.lower_solve(destroy=[])

        status = 0
        if feas == False:
            status = 1

        # End solver
        Milp.end()

        # Return solution
        return (obj, status)

###############################################################################
### For testing (delete later)

if __name__ == "__main__":
    TestSolver = TrialSolver("problems/smallnet.min")

    #print(TestSolver.solve_milp_cutting_plane())
    #print(TestSolver.solve_lp_cutting_plane())
    #print(TestSolver.last_defense)
    #print(TestSolver.solve_milp_defend(TestSolver.last_defense))
    #print(TestSolver.solve_milp_defend([False
    #                                    for a in TestSolver.Net.def_arcs]))
    #print(TestSolver.solve_milp_initial())
    print(TestSolver.solve_lp_duality())
