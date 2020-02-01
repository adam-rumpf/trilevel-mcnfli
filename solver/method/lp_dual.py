"""Duality solution algorithm for the lower-level bilevel program (LP).

Includes an LLDuality class which applies the duality solution method given a
protection vector. Returns the objective value and attack vector obtained from
the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the LP
relaxation of the binary interdependence model.
"""

import cplex

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
    def __init__(self, net_in):
        """LP duality solution object constructor.

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

        ###
        # Duality model:
        # binary attack decision psi for every destructible arc
        # free node potential lambda for each node
        # nonnegative interdependency variable mu for each interdependency
        # nonnegative flow bound variable eta for each destructible arc
        # Objective:
        # Sum of b_i lambda_i over all nodes, minus sum of u_ij eta_ij over all arcs
        # Constraints:
        # Each arc ij has a basic constraint of the form c_ij - lambda_i + lambda_j + eta_ij >= 0
        # Additional terms are added depending on the type of arc.
        # If destructible, add M psi_ij
        # If a parent, add -(u_kl/u_ij) mu_ij^kl
        # If a child, add mu_kl^ij

        parents = [a[0] for a in self.Net.int] # parent arc objects
        children = [a[1] for a in self.Net.int] # child arc objects

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

        # Add binary attack decision variables to Cplex object
        self.DualModel.variables.add(names=self.att_vars,
                                     types="B"*len(self.att_vars))

        # Add attack decision penalty variables to Cplex object
        self.DualModel.variables.add(names=pen_vars,
                                lb=[0.0 for a in self.Net.att_arcs],
                                ub=[cplex.infinity for a in self.Net.att_arcs])

        # Add node potential dual variables to Cplex object
        self.DualModel.variables.add(obj=[n.supply for n in self.Net.nodes],
                                  names=node_vars,
                                  lb=[-cplex.infinity for n in self.Net.nodes],
                                  ub=[cplex.infinity for n in self.Net.nodes])

        # Add interdependency dual variables to Cplex object
        self.DualModel.variables.add(names=int_vars,
                         lb=[0.0 for i in range(len(self.Net.int))],
                         ub=[cplex.infinity for i in range(len(self.Net.int))])

        # Add arc capacity dual variables to Cplex object
        self.DualModel.variables.add(obj=[-a.bound for a in self.Net.arcs],
                                    names=bound_vars,
                                    lb=[0.0 for a in self.Net.arcs],
                                    ub=[cplex.infinity for a in self.Net.arcs])

        ### Note: We will need some indicator constraints to force the penalty terms to be zero when the decision is 0.














#        # Define a list of attack variable constraint names for defensible arcs
#        self.att_con = ["df("+str(a.id)+")" for a in self.Net.def_arcs]
#
#        # Define a list of penalty variable indicator constraint names
#        pen_con = ["ap("+str(a.id)+")" for a in self.Net.att_arcs]
#
#        # Define sense string for attack constraints (all <=)
#        att_sense = "L"*len(self.Net.def_arcs)
#
#        # Define attack constraint righthand sides (all 1)
#        att_rhs = [1 for a in self.Net.def_arcs]
#
#        # Define attack constraints for each arc (initially just bounds)
#        att_expr = [[["at("+str(a.id)+")"], [1]] for a in self.Net.def_arcs]
#
#        # Define attack constraints to limit the total number of attacks
#        att_lim_expr = [[[v for v in self.att_vars],
#                         [1 for v in self.att_vars]]]
#
#        # Define penalty variable constraints to limit value when activated
#        pen_expr = [[[v], [1]] for v in self.pen_vars]
#
#        # Add attack constraints to Cplex object
#        self.UpperModel.linear_constraints.add(names=self.att_con,
#                                               lin_expr=att_expr,
#                                               senses=att_sense, rhs=att_rhs)
#        self.UpperModel.linear_constraints.add(names=["ab"],
#                                               lin_expr=att_lim_expr,
#                                               senses=["L"],
#                                               rhs=[self.Net.att_limit])
#
#        # Add penalty variable indicator constraints to Cplex object
#        self.UpperModel.indicator_constraints.add_batch(name=pen_con,
#                                       indvar=self.att_vars,
#                                       complemented=[1 for a in self.att_vars],
#                                       lin_expr=pen_expr,
#                                       sense=["L" for a in self.att_vars],
#                                       rhs=[0.0 for a in self.att_vars])

    #--------------------------------------------------------------------------
    def solve(self, defend, cplex_epsilon=0.001):
        """Bilevel subproblem solution method.

        The duality formulation includes a single MILP that combines both the
        attacker's decision variables and (the dual verion of) the defender's
        response. For this reason the submodel need only be solved once in
        order to obtain an attack vector and objective value.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Accepts the following optional keyword arguments:
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
        """

        # Clean up the model
        self.DualModel.cleanup(cplex_epsilon)













#        # Set local variables
#        obj_ub = -cplex.infinity # objective upper bound (upper-level problem)
#        obj_lb = cplex.infinity # objective lower bound (lower-level problem)
#        iteration= 1 # current iteration number
#        status = 0 # exit code
#
#        ###
#        print("\nInitializing P2-3' cutting plane search.\n")
#        print("-"*20+" Iteration 0 "+"-"*20)
#
#        # Solve the upper-level problem once for the given defense vector
#        (obj_ub, destroy) = self._upper_solve(defend=defend,
#                                              cplex_epsilon=cplex_epsilon)
#
#        ###
#        print("rho2 = "+str(obj_ub))
#
#        # Find the lower-level response for the given attack vector
#        (obj_lb, nonzero, feasible) = self._lower_solve(destroy=destroy,
#                                                   cplex_epsilon=cplex_epsilon)
#
#        ###
#        print("rho1 = "+str(obj_lb))
#
#        obj_gap = abs(obj_ub - obj_lb) # current optimality gap
#
#        ###
#        print("Optimality gap = "+str(obj_gap))
#
#        #----------------------------------------------------------------------
#        # Main cutting plane loop begin
#
#        while (iteration < cutoff) and (obj_gap > gap):
#
#            iteration += 1
#
#            ###
#            print("-"*20+" Iteration "+str(iteration-1)+" "+"-"*20)
#
#            # Add a constraint based on the nonzero flow vector
#            self._upper_add_constraint(obj_lb, nonzero)
#
#            # Re-solve the relaxed master problem
#            (obj_ub, destroy) = self._upper_solve(cplex_epsilon=cplex_epsilon)
#
#            ###
#            print("rho2 = "+str(obj_ub))
#
#            # Re-solve the lower-level response
#            (obj_lb, nonzero, feasible) = self._lower_solve(destroy=destroy,
#                                                   cplex_epsilon=cplex_epsilon)
#
#            ### Include the potential to break here if LL is infeasible.
#            if feasible == False:
#                print("Response problem infeasible.")
#                obj_ub = self.big_m
#                obj_lb = self.big_m
#                status = 1
#                break
#
#            ###
#            print("rho1 = "+str(obj_lb))
#
#            # Recalculate the optimality gap
#            obj_gap = abs(obj_ub - obj_lb)
#
#            ###
#            print("Optimality gap = "+str(obj_gap))
#
#            if (iteration >= cutoff) and (obj_gap > gap):
#                status = 3
#
#        # Main cutting plane loop end
#        #----------------------------------------------------------------------

        return (0, self.attack, 2)###((obj_ub+obj_lb)/2, destroy, status, iteration)

    #--------------------------------------------------------------------------
    def end(self):
        """Closes the internal Cplex model.

        This should be called before the LLDual object is discarded.
        """

        self.DualModel.end()

###############################################################################
### For testing (delete later)

if __name__ == "__main__":
    import network.network as net
    TestNet = net.Network("../../problems/smallnet.min")
    TestSolver = LLDuality(TestNet)

    print(TestSolver.solve([False, False, False, False, True, False, False]))

    TestSolver.DualModel.write("dual_program.lp")

    TestSolver.end()
