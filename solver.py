"""Driver for the various solution methods of the trilevel game.

Calls each of the solution algorithms for a given problem instance, and also
applies the LP relaxation solution to the MILP to evaluate the optimality gap.

The functions in this module can be used to call a particular solution method
once, and are used by the main driver to call all available solution methods
for comparison.
"""

import gc
import time

import solver.upper_cp as ucp
import solver.method.network.network as net

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
                                 upper_gap=0.01, lower_gap=0.01):
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
        Up = ucp.UpperLevel(self.Net, 1)

        # Apply solver, save defense, and time entire process
        tot = time.time()
        (obj, self.last_defense, attack, times, iterations) = Up.solve_upper(
                cutoff=upper_cutoff, lower_cutoff=lower_cutoff, gap=upper_gap,
                lower_gap=lower_gap)
        tot = time.time() - tot

        # Return results
        return (obj, self.last_defense, attack, (tot, times[0], times[1]),
                iterations)

    #--------------------------------------------------------------------------
    def solve_lp_cutting_plane(self, upper_cutoff=100, lower_cutoff=100,
                               upper_gap=0.01, lower_gap=0.01):
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
        Up = ucp.UpperLevel(self.Net, 2)

        # Apply solver, save defense, and time entire process
        tot = time.time()
        (obj, self.last_defense, attack, times, iterations) = Up.solve_upper(
                cutoff=upper_cutoff, lower_cutoff=lower_cutoff, gap=upper_gap,
                lower_gap=lower_gap)
        tot = time.time() - tot

        # Return results
        return (obj, self.last_defense, attack, (tot, times[0], times[1]),
                iterations)

    #--------------------------------------------------------------------------
    def solve_lp_duality(self, upper_cutoff=100, upper_gap=0.01):
        """Solves the trilevel model using the LP duality algorithm.

        Creates an upper-level solver object associated with the problem
        network, and equipped to solve the model version with linear
        interdependencies via the duality lower-level algorithm.

        Accepts the following optional keyword arguments:
            upper_cutoff -- Iteration cutoff for the upper-level cutting plane
                main loop. Defaults to 100.
            upper_gap -- Optimality gap tolerance for the upper-level cutting
                plane main loop. Defaults to 0.01.

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
        Up = ucp.UpperLevel(self.Net, 3)

        # Apply solver, save defense, and time entire process
        tot = time.time()
        (obj, self.last_defense, attack, times, iterations) = Up.solve_upper(
                cutoff=upper_cutoff, gap=upper_gap)
        tot = time.time() - tot

        # Return results
        return (obj, self.last_defense, attack, (tot, times[0], times[1]),
                iterations)

    #--------------------------------------------------------------------------
    def solve_milp_defend(self, defend):
        """Calculates the MILP solution for a given defensive decision.

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
        """

        # Collect garbage
        gc.collect()

        # Initialize a temporary upper-level solver object
        Up = ucp.UpperLevel(self.Net, 1)

        # Return lower level solution for specified defense
        return Up.solve_lower(defend)
