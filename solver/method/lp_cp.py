"""Cutting plane solution algorithm for the lower-level bilevel program (LP).

Includes an LPCuttingPLane class which applies the cutting plane solution
method given a protection vector. Returns the objective value and attack vector
obtained from the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the LP
relaxation of the binary interdependence model.
"""

import cplex
import random

#==============================================================================
class LPCuttingPlane:
    """Class to implement the cutting plane method for the LP lower model.

    This class also includes a local Cplex objects to represent the upper and
    lower problems of the bilevel submodel. These objects remain active for the
    entirety of the trilevel solution algorithm as the lower-level program is
    repeatedly solved and re-solved, which saves time since previous results
    may be retained.

    The end() method should be called before finishing work with this object in
    order to close the Cplex objects.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in):
        """LP cutting plane solution object constructor.

        Ininitializes the Cplex objects associated with the lower-level
        subproblem.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
        """

        self.Net = net_in # set reference to network object

        # Initialize attack vector and related lists
        self.attack = [False for a in self.Net.att_arcs]
        self.attack_rhs = [a.bound for a in self.Net.arcs]

        # Initialize Cplex objects
        self._upper_cplex_setup()
        self._lower_cplex_setup()

    #--------------------------------------------------------------------------
    def _upper_cplex_setup(self):
        """Initializes Cplex object for the relaxed master problem.

        The relaxed master problem is the attacker's maximization MILP which
        includes an expanding set of constraints based on previously-calculated
        lower level solutions.
        """

        pass

    #--------------------------------------------------------------------------
    def _lower_cplex_setup(self):
        """Initializes Cplex object for interdependent min-cost flow problem.

        The interdependent network problem is the defender's minimization LP
        in which they respond to the attacker's destruction to optimize the
        resulting network.
        """

        # Initialize object
        self.LowerModel = cplex.Cplex()

        # Silence CPLEX output streams
        ###self.LowerModel.set_log_stream(None)
        ###self.LowerModel.set_results_stream(None)
        ###self.LowerModel.set_error_stream(None)
        ###self.LowerModel.set_warning_stream(None)

        # Set as minimization
        self.LowerModel.objective.set_sense(
                self.LowerModel.objective.sense.minimize)

        # Define a list of variable names
        flow_vars = ["x("+str(a.id)+")" for a in self.Net.arcs]

        # Define objective coefficients
        flow_costs = [a.cost for a in self.Net.arcs]

        # Define flow bounds
        flow_lb = [0.0 for a in self.Net.arcs]
        flow_ub = [a.bound for a in self.Net.arcs]

        # Add variables to Cplex object
        self.LowerModel.variables.add(names=flow_vars, obj=flow_costs,
                                      lb=flow_lb, ub=flow_ub)

        # Define a list of constraint names (flow attack constraints need to
        # change during the solution process, so that list is saved)
        flow_con = ["c("+str(n.id)+")" for n in self.Net.nodes]
        self.flow_att = ["a("+str(a.id)+")" for a in self.Net.att_arcs]
        flow_int = ["i("+str(i)+")" for i in range(len(self.Net.int))]

        # Define sense strings for constraints (== for flow conservation, <=
        # for all others)
        flow_con_sense = "E"*len(flow_con)
        flow_att_sense = "L"*len(self.flow_att)
        flow_int_sense = "L"*len(flow_int)

        # Define constraint righthand sides
        flow_con_rhs = [n.supply for n in self.Net.nodes]
        flow_att_rhs = [a.bound for a in self.Net.att_arcs]
        flow_int_rhs = [0.0 for i in self.Net.int]

        # Define flow conservation constraints for each node
        flow_con_expr = [[[], []] for n in self.Net.nodes]
        i = 0
        for n in self.Net.nodes:

            # Get variable names of outgoing/incoming arcs
            var_out = [flow_vars[a.id] for a in n.out_arcs]
            var_in = [flow_vars[a.id] for a in n.in_arcs]

            # Set coefficients
            coef = [1 for i in var_out] + [-1 for i in var_in]

            # Update constraint list
            flow_con_expr[i] = [var_out+var_in, coef]

            i += 1

        # Define flow attack constraints for each arc (initially just bounds)
        flow_att_expr = [[[flow_vars[a.id]], [1.0]] for a in self.Net.att_arcs]

        # Define interdependency constraints (arranged with variables on LHS)
        flow_int_expr = [[[], []] for i in self.Net.int]
        i = 0
        for intd in self.Net.int:

            # Get parent/child arc names
            var_pair = [flow_vars[intd[1].id], flow_vars[intd[0].id]]

            # Set coefficients
            coef = [-(1.0*intd[0].bound/intd[1].bound), 1.0]

            # Update constraint list
            flow_int_expr[i] = [var_pair, coef]

            i += 1

        # Add constraints to Cplex object
        self.LowerModel.linear_constraints.add(names=flow_con,
                                               lin_expr=flow_con_expr,
                                               senses=flow_con_sense,
                                               rhs=flow_con_rhs)
        self.LowerModel.linear_constraints.add(names=self.flow_att,
                                               lin_expr=flow_att_expr,
                                               senses=flow_att_sense,
                                               rhs=flow_att_rhs)
        self.LowerModel.linear_constraints.add(names=flow_int,
                                               lin_expr=flow_int_expr,
                                               senses=flow_int_sense,
                                               rhs=flow_int_rhs)

        ### Write problem file as a test
        self.LowerModel.write("test_program.lp")

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

    #--------------------------------------------------------------------------
    def end(self):
        """Closes all internal Cplex models.

        This should be called before the lower-level solver is discarded.
        """

        self.LowerModel.end()
        self.UpperModel.end()

###############################################################################
### For testing
import network.network as net
TestNet = net.Network("../../problems/smallnet.min")
TestSolver = LPCuttingPlane(TestNet)
