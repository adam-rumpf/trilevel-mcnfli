"""Duality solution algorithm for the lower-level bilevel program (LP).

Includes an LLDuality class which applies the duality solution method given a
protection vector. Returns the objective value and attack vector obtained from
the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the LP
relaxation of the binary interdependence model.
"""

import cplex

import upper.lower.network.network as net

#==============================================================================
class LLDuality:
    """Class to implement the duality method for the LP lower model.

    This class also includes a local Cplex object to represent the lower-level
    program. The object remains active for the entirety of the trilevel
    solution algorithm as the lower-level program is repeatedly solved and re-
    solved, which saves time since previous results may be retained.

    The end() method should be called before finishing work with this object in
    order to close the Cplex objects.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in, big_m=1.0e10):
        """LP duality solution object constructor.

        Initializes the Cplex objects associated with the lower-level
        subproblem.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.

        Accepts the following optional keyword arguments:
            big_m -- Large constant for use in bounding certain variables in
                the CPLEX program. Defaults to 1.0e10.
        """

        self.Net = net_in # set reference to network object
        self.big_m = big_m # large constant bound

        # Initialize attack vector and related lists
        self.attack = [False for a in self.Net.att_arcs]
        self.attack_rhs = [a.bound for a in self.Net.arcs]

        # Initialize Cplex object
        self._cplex_setup()

    #--------------------------------------------------------------------------
    def _cplex_setup(self):
        """Initializes Cplex object for the duality method MILP.

        The duality method is based on replacing the lower-level problem with
        its dual and combining it with the upper-level problem to create a
        single MILP. The decision variables include the attacker's binary
        decision vector as well as several sets of lower-level dual variables.
        """

        # Initialize object
        self.DualModel = cplex.Cplex()

        # Silence CPLEX output streams
        self.DualModel.set_log_stream(None)
        self.DualModel.set_results_stream(None)
        self.DualModel.set_error_stream(None)
        self.DualModel.set_warning_stream(None)

        # Set as maximization
        self.DualModel.objective.set_sense(
            self.DualModel.objective.sense.maximize)

        # If using sink nodes as interdependency parents, the supply values are
        # relaxed, which leads to a slightly different dual program. We will
        # generate a list of supply values for use throughout the program.
        if self.Net.parent_type == 0:
            supply_nodes = []
            for n in self.Net.nodes:
                if n.supply > 0:
                    supply_nodes.append(n)

        # Note: In order to avoid problems with the behavior of CPLEX with
        # big-M constraints, for each attack variable we also define a
        # continuous penalty variable on [0, M] along with an indicator
        # constraint that forces it to be 0 whenever the attack variable is 0.
        # Within the constraint set, we use the penalty variables rather than a
        # product of M and an attack variable in order to avoid the possibility
        # of a very small nonzero attack decision being multiplied by M to
        # erroneously nullify one of the constraints.

        # Define a list of variable names
        self.att_vars = ["at("+str(a.id)+")" for a in self.Net.att_arcs]
        pen_vars = ["pt("+str(a.id)+")" for a in self.Net.att_arcs]
        node_vars = ["np("+str(n.id)+")" for n in self.Net.nodes]
        int_vars = ["in("+str(i)+")" for i in range(len(self.Net.int))]
        bound_vars = ["ub("+str(a.id)+")" for a in self.Net.arcs]
        if self.Net.parent_type == 0:
            # Additional dual variables are required for supply nodes in the
            # node parent model
            supply_vars = ["ns("+str(n.id)+")" for n in supply_nodes]

        # Define bounds of node dual variables
        node_vars_lb = [-cplex.infinity for n in self.Net.nodes]
        node_vars_ub = [cplex.infinity for n in self.Net.nodes]
        if self.Net.parent_type == 0:
            for n in supply_nodes:
                # The dual variables for supply nodes are nonnegative in the
                # node parent model
                node_vars_lb[n.id] = 0.0

        # Define coefficients of node dual variables
        node_vars_coef = [n.supply for n in self.Net.nodes]
        if self.Net.parent_type == 0:
            for n in supply_nodes:
                # Supply values are negated for supply nodes in the node parent
                # model
                node_vars_coef[n.id] = -n.supply

        # Add binary attack decision variables to Cplex object
        self.DualModel.variables.add(names=self.att_vars,
                                     types="B"*len(self.att_vars))

        # Add attack decision penalty variables to Cplex object
        self.DualModel.variables.add(names=pen_vars,
                                    lb=[0.0 for a in self.Net.att_arcs],
                                    ub=[self.big_m for a in self.Net.att_arcs])

        # Add node potential dual variables to Cplex object
        self.DualModel.variables.add(obj=node_vars_coef, names=node_vars,
                                     lb=node_vars_lb, ub=node_vars_ub)

        # Add extra dual variables for supply relaxation
        if self.Net.parent_type == 0:
            self.DualModel.variables.add(names=supply_vars,
                                     lb=[0.0 for n in supply_nodes],
                                     ub=[cplex.infinity for n in supply_nodes])

        # Add interdependency dual variables to Cplex object
        self.DualModel.variables.add(names=int_vars,
                         lb=[0.0 for i in range(len(self.Net.int))],
                         ub=[cplex.infinity for i in range(len(self.Net.int))])

        # Add arc capacity dual variables to Cplex object
        self.DualModel.variables.add(obj=[-a.bound for a in self.Net.arcs],
                                    names=bound_vars,
                                    lb=[0.0 for a in self.Net.arcs],
                                    ub=[cplex.infinity for a in self.Net.arcs])

        # Define a list of constraint names for all arcs
        arc_con = ["ac("+str(a.id)+")" for a in self.Net.arcs]

        # Create a list of variables and coefficients for each arc constraint.
        # All arc constraints include the node potentials of the endpoints and
        # the arc bound dual variable. Destructible arcs also receive a penalty
        # term while interdependent arcs also receive interdependency dual
        # variable terms.
        # In the arc parent formulation, there is a single type of free dual
        # variable used in each arc's constraint. In the node parent
        # formulation there are two nonnegative variables whose difference is
        # used for each supply node.

        # Common base constraints for flow bounds
        arc_con_vars = [[bound_vars[a.id]] for a in self.Net.arcs]
        arc_con_coef = [[1.0] for a in self.Net.arcs]

        # Determine endpoint node dual variables depening on model type
        if self.Net.parent_type == 1:
            # For arc parents, the dual variables of both endpoints appear
            arc_con_vars = [[node_vars[a.tail.id], node_vars[a.head.id]]
                            for a in self.Net.arcs]
            arc_con_coef = [[-1.0, 1.0] for a in self.Net.arcs]
        else:
            # For node parents, the dual variables of the endpoints depend on
            # whether each endpoint is a source node and whether it is defined.

            # Process all arc tails
            for a in self.Net.arcs:

                # Undefined tail
                if a.tail == None:
                    continue

                # Supply tail
                elif a.tail.supply > 0:
                    pass

                # Nonsupply tail
                else:
                    pass

            # Process all arc heads
            for a in self.Net.arcs:

                # Undefined head
                if a.head == None:
                    continue

                # Supply head
                elif a.head.supply > 0:
                    pass

                # Nonsupply head
                else:
                    pass

        # Destructible arcs receive a penalty term
        for i in range(len(self.Net.att_arcs)):
            a = self.Net.att_arcs[i]
            arc_con_vars[a.id].append(pen_vars[i])
            arc_con_coef[a.id].append(1.0)

        # Interdependent arcs receive an interdependency dual variable term
        for i in range(len(self.Net.int)):

            ap = self.Net.int[i][0] # parent arc
            ac = self.Net.int[i][1] # child arc

            # Parent arc constraint
            arc_con_vars[ap.id].append(int_vars[i])
            arc_con_coef[ap.id].append(-1.0*ac.bound/ap.bound)

            # Child arc constraint
            arc_con_vars[ac.id].append(int_vars[i])
            arc_con_coef[ac.id].append(1.0)

        # Define a list of attack variable constraint names for defensible arcs
        self.att_con = ["df("+str(a.id)+")" for a in self.Net.def_arcs]

        # Define a list of penalty variable indicator constraint names
        pen_con = ["ap("+str(a.id)+")" for a in self.Net.att_arcs]

        # Define sense string for attack constraints (all <=)
        att_sense = "L"*len(self.Net.def_arcs)

        # Define attack constraint righthand sides (all 1)
        att_rhs = [1 for a in self.Net.def_arcs]

        # Define attack constraints for each arc (initially just bounds)
        att_expr = [[["at("+str(a.id)+")"], [1]] for a in self.Net.def_arcs]

        # Define attack constraints to limit the total number of attacks
        att_lim_expr = [[[v for v in self.att_vars],
                         [1 for v in self.att_vars]]]

        # Define penalty variable constraints to limit value when activated
        pen_expr = [[[v], [1]] for v in pen_vars]

        # Add attack constraints to Cplex object, using equality for the attack
        # bound in order to reduce the number of feasible solutions
        self.DualModel.linear_constraints.add(names=self.att_con,
                                              lin_expr=att_expr,
                                              senses=att_sense, rhs=att_rhs)
        self.DualModel.linear_constraints.add(names=["ab"],
                                              lin_expr=att_lim_expr,
                                              senses=["E"],
                                              rhs=[self.Net.att_limit])

        # Add penalty variable indicator constraints to Cplex object
        for i in range(len(pen_con)):
            self.DualModel.indicator_constraints.add(name=pen_con[i],
                                                     indvar=self.att_vars[i],
                                                     complemented=1,
                                                     lin_expr=pen_expr[i],
                                                     sense="L",
                                                     rhs=0.0)

        # Add arc constraints to Cplex object
        self.DualModel.linear_constraints.add(names=arc_con,
                                          lin_expr=[[arc_con_vars[i],
                                                     arc_con_coef[i]]
                                          for i in range(len(self.Net.arcs))],
                                          senses="G"*len(self.Net.arcs),
                                          rhs=[-a.cost for a in self.Net.arcs])

        self.DualModel.write("DualModel.lp")###

    #--------------------------------------------------------------------------
    def solve(self, defend, cutoff=100, gap=0.01, cplex_epsilon=0.001):
        """Bilevel subproblem solution method.

        The duality formulation includes a single MILP that combines both the
        attacker's decision variables and (the dual version of) the defender's
        response. For this reason the submodel need only be solved once in
        order to obtain an attack vector and objective value.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Accepts the following optional keyword arguments:
            cutoff -- Dummy argument that does nothing for the duality method.
                Included only to give all methods the same argument set.
            gap -- Dummy argument that does nothing for the duality method.
                Included only to give all methods the same argument set.
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
            zero -- Placeholder value of 0. Included only to give all methods
                the same output set.
        """

        # Clean up the model
        self.DualModel.cleanup(cplex_epsilon)

        # Update constraints based on arc defense vector
        if len(defend) == len(self.Net.def_arcs):
            new_rhs = [1 for a in self.Net.def_arcs]
            for i in range(len(new_rhs)):
                if defend[i] == True:
                    new_rhs[i] = 0
            self.DualModel.linear_constraints.set_rhs([(self.att_con[i],
                           new_rhs[i]) for i in range(len(self.Net.def_arcs))])

        self.DualModel.write("DualModel.lp")###

        # Solve the MILP
        self.DualModel.solve()

        ###
        if self.DualModel.solution.is_primal_feasible() == True:
            print("Lower-level primal feasible.")
        else:
            print("Lower-level primal infeasible.")
        if self.DualModel.solution.is_dual_feasible() == True:
            print("Lower-level dual feasible.")
        else:
            print("Lower-level dual infeasible.")

        # Get the objective value
#        obj = self.DualModel.solution.get_objective_value()

        # Set unbounded objective value to infinity (CPLEX returns an objective
        # of 0.0 for unbounded problems)
        ###
#        if ((obj == 0.0 or obj >= 0.1*self.big_m) and
#            (self.DualModel.solution.is_primal_feasible() == True)):
#            obj = cplex.infinity
#            status = 1

        # Get objective value and solution vector
        if self.DualModel.solution.is_dual_feasible() == True:
            # If dual feasible then the primal is bounded and we can return the
            # objective and solution found by CPLEX
            status = 0
            obj = self.DualModel.solution.get_objective_value()
            destroy = [False for a in self.Net.att_arcs]
            for i in range(len(self.Net.att_arcs)):
                if self.DualModel.solution.get_values(self.att_vars[i]) == 1:
                    destroy[i] = True
        else:
            # Otherwise the primal is unbounded and we return an infinite
            # objective and an attack vector that is the exact inverse of the
            # input defense vector (this will prevent the defender from
            # repeating exactly the same defense that led to an infeasible
            # response problem)
            status = 1
            obj = cplex.infinity
            destroy = [True for a in self.Net.att_arcs]
            for i in range(len(self.Net.def_arcs)):
                if defend[i] == True:
                    destroy[i] = False

        return (obj, destroy, status, 0)

    #--------------------------------------------------------------------------
    def end(self):
        """Closes the internal Cplex model.

        This should be called before the LLDual object is discarded.
        """

        self.DualModel.end()

###############################################################################
### For testing (delete later)

if __name__ == "__main__":
    TestNet = net.Network("../../../problems/smallnet.min")
    TestSolver = LLDuality(TestNet, big_m=1.0e10)

    print(TestSolver.solve([False, True, True, False, True, False, False]))
    #print(TestSolver.solve([False, False, False, False, False, False, False]))

    #nms = TestSolver.DualModel.variables.get_names()
    #val = TestSolver.DualModel.solution.get_values()
    #for i in range(len(nms)):
    #    print(str(nms[i])+" = "+str(val[i]))

    TestSolver.DualModel.write("dual_program.lp")

    TestSolver.end()
