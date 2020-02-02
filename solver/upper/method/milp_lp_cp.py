"""Cutting plane solution algorithm for the lower-level bilevel MILP or LP.

Includes a LLCuttingPLane class which applies the cutting plane solution method
given a protection vector. Returns the objective value and attack vector
obtained from the lower-level bilevel maximization.

The class can be used to model either the bilevel interdependency MILP or its
LP relaxation.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the MILP
binary interdependence model.
"""

import cplex

#==============================================================================
class LLCuttingPlane:
    """Class to implement the cutting plane method for the lower LP or MILP.

    This class also includes a local Cplex object to represent the lower-level
    program. The object remains active for the entirety of the trilevel
    solution algorithm as the lower-level program is repeatedly solved and re-
    solved, which saves time since previous results may be retained.

    The end() method should be called before finishing work with this object in
    order to close the Cplex objects.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in, mode, big_m=1.0e10):
        """LP or MILP cutting plane solution object constructor.

        Ininitializes the Cplex objects associated with the lower-level
        subproblem, which is constructed either as the LP or the MILP version
        of the interdependent network flows program, depending on the selected
        option.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
            mode -- Selects whether to construct the lower-level program as the
                binary interdependency MILP or the linear interdependency LP.
                The numerical codes are as follows:
                    1: binary interdependency MILP
                    2: linear interdependency LP

        Accepts the following optional keyword arguments:
            big_m -- Large constant for use in the big-M method. Defaults to
                1.0e10.
        """

        self.Net = net_in # set reference to network object
        self.big_m = big_m # penalty value for use in relaxed master problem

        # Initialize attack vector and related lists
        self.attack = [False for a in self.Net.att_arcs]
        self.attack_rhs = [a.bound for a in self.Net.arcs]

        # Initialize Cplex objects
        self._upper_cplex_setup()
        self._lower_cplex_setup(mode)

    #--------------------------------------------------------------------------
    def _upper_cplex_setup(self):
        """Initializes Cplex object for the relaxed master problem.

        The relaxed master problem is the attacker's maximization MILP which
        includes an expanding set of constraints based on previously-calculated
        lower level solutions.

        The MILP is initialized with no constraints on the objective value. A
        constraint is added after each solution of the lower-level program.
        """

        # Initialize object
        self.UpperModel = cplex.Cplex()

        # Silence CPLEX output streams
        self.UpperModel.set_log_stream(None)
        self.UpperModel.set_results_stream(None)
        self.UpperModel.set_error_stream(None)
        self.UpperModel.set_warning_stream(None)

        # Set as maximization
        self.UpperModel.objective.set_sense(
            self.UpperModel.objective.sense.maximize)

        # Note: In order to avoid problems with the behavior of CPLEX with
        # big-M constraints, for each attack variable we also define a
        # continuous penalty variable on [0, M] along with an indicator
        # constraint that forces it to be 0 whenever the attack variable is 0.
        # Within the relaxed master problem's constraint set, we use the
        # penalty variables rather than a product of M and an attack variable
        # in order to avoid the possibility of a very small nonzero attack
        # decision being multiplied by M to erroneously nullify one of the
        # objective bound constraints.

        # Define a list of variable names
        self.obj_var = "ob"
        self.att_vars = ["at("+str(a.id)+")" for a in self.Net.att_arcs]
        self.pen_vars = ["pt("+str(a.id)+")" for a in self.Net.att_arcs]

        # Add objective bound variable to Cplex object
        self.UpperModel.variables.add(obj=[1.0], names=[self.obj_var],
                                      lb=[-cplex.infinity],
                                      ub=[cplex.infinity])

        # Add binary attack decision variables to Cplex object
        self.UpperModel.variables.add(names=self.att_vars,
                                      types="B"*len(self.att_vars))

        # Add penalty variables to Cplex object
        self.UpperModel.variables.add(names=self.pen_vars,
                                    lb=[0.0 for a in self.Net.att_arcs],
                                    ub=[self.big_m for a in self.Net.att_arcs])

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
        pen_expr = [[[v], [1]] for v in self.pen_vars]

        # Add attack constraints to Cplex object
        self.UpperModel.linear_constraints.add(names=self.att_con,
                                               lin_expr=att_expr,
                                               senses=att_sense, rhs=att_rhs)
        self.UpperModel.linear_constraints.add(names=["ab"],
                                               lin_expr=att_lim_expr,
                                               senses=["L"],
                                               rhs=[self.Net.att_limit])

        # Add penalty variable indicator constraints to Cplex object
        self.UpperModel.indicator_constraints.add_batch(name=pen_con,
                                       indvar=self.att_vars,
                                       complemented=[1 for a in self.att_vars],
                                       lin_expr=pen_expr,
                                       sense=["L" for a in self.att_vars],
                                       rhs=[0.0 for a in self.att_vars])

        # Keep track of the number of side constraints generated so far
        self.side_constraints = 0

    #--------------------------------------------------------------------------
    def _lower_cplex_setup(self, mode):
        """Initializes Cplex object for interdependent min-cost flow problem.

        The interdependent network problem is the defender's minimization MILP
        in which they respond to the attacker's destruction to optimize the
        resulting network.

        The MILP is initialized with all arcs intact. The constraints are
        updated before each solve to reflect the damage caused by the
        attacker's decisions.

        Requires the following positional arguments:
            mode -- Selects whether to construct the lower-level program as the
                binary interdependency MILP or the linear interdependency LP.
                The numerical codes are as follows:
                    1: binary interdependency MILP
                    2: linear interdependency LP
        """

        # Initialize object
        self.LowerModel = cplex.Cplex()

        # Silence CPLEX output streams
        self.LowerModel.set_log_stream(None)
        self.LowerModel.set_results_stream(None)
        self.LowerModel.set_error_stream(None)
        self.LowerModel.set_warning_stream(None)

        # Set as minimization
        self.LowerModel.objective.set_sense(
            self.LowerModel.objective.sense.minimize)

        # Define a list of variable names
        self.flow_vars = ["x("+str(a.id)+")" for a in self.Net.arcs]

        # Define objective coefficients
        flow_costs = [a.cost for a in self.Net.arcs]

        # Define flow bounds
        flow_lb = [0.0 for a in self.Net.arcs]
        flow_ub = [a.bound for a in self.Net.arcs]

        # Add variables to Cplex object
        self.LowerModel.variables.add(names=self.flow_vars, obj=flow_costs,
                                      lb=flow_lb, ub=flow_ub)

        # Define a list of common constraint names (flow attack constraints
        # need to change during the solution process, so that list is saved)
        flow_con = ["c("+str(n.id)+")" for n in self.Net.nodes]
        self.flow_att = ["a("+str(a.id)+")" for a in self.Net.att_arcs]

        # Define sense strings for common constraints (== for flow
        # conservation, <= for all others)
        flow_con_sense = "E"*len(flow_con)
        flow_att_sense = "L"*len(self.flow_att)

        # Define common constraint righthand sides
        flow_con_rhs = [n.supply for n in self.Net.nodes]
        flow_att_rhs = [a.bound for a in self.Net.att_arcs]

        # Define flow conservation constraints for each node
        flow_con_expr = [[[], []] for n in self.Net.nodes]
        i = 0
        for n in self.Net.nodes:

            # Get variable names of outgoing/incoming arcs
            var_out = [self.flow_vars[a.id] for a in n.out_arcs]
            var_in = [self.flow_vars[a.id] for a in n.in_arcs]

            # Set coefficients
            coef = [1 for i in var_out] + [-1 for i in var_in]

            # Update constraint list
            flow_con_expr[i] = [var_out+var_in, coef]

            i += 1

        # Define flow attack constraints for each arc (initially just bounds)
        flow_att_expr = [[[self.flow_vars[a.id]], [1.0]]
                         for a in self.Net.att_arcs]

        # Add common constraints to Cplex object
        self.LowerModel.linear_constraints.add(names=flow_con,
                                               lin_expr=flow_con_expr,
                                               senses=flow_con_sense,
                                               rhs=flow_con_rhs)
        self.LowerModel.linear_constraints.add(names=self.flow_att,
                                               lin_expr=flow_att_expr,
                                               senses=flow_att_sense,
                                               rhs=flow_att_rhs)

        # Add interdependencies for chosen model type
        if mode == 1:

            # MILP formulation

            # The binary interdependency formulation requires defining slack
            # variables and binary linking variables along with logical
            # constraints. For the CPLEX model we implement this by using
            # indicator constraints rather than the binary linking variables
            # in the original model.
            # We now use binary slack variables in a constraint of the form:
            #   u_ij s_ij^kl + x_ij >= u_ij
            # If x_ij = u_ij, then s_ij^kl is free.
            # If x_ij < u_ij, then s_ij^kl = 1.
            # We will include an indicator constraint that forces x_kl <= 0
            # when s_ij^kl = 1.

            parents = [a[0] for a in self.Net.int] # parent arc objects
            children = [a[1] for a in self.Net.int] # child arc objects

            # Add binary slack indicator variables to Cplex object
            slack_vars = ["sl("+str(a.id)+")" for a in parents]

            # Add variables to Cplex object
            self.LowerModel.variables.add(names=slack_vars,
                                          types="B"*len(parents))

            # Define slack constraint names, senses, and righthand sides
            slack_con = ["sc("+str(a.id)+")" for a in parents]
            slack_con_sense = "G"*len(parents)
            slack_con_rhs = [a.bound for a in parents]

            # Define slack constraint linear expressions
            slack_con_expr = [[[slack_vars[i], self.flow_vars[parents[i].id]],
                           [parents[i].bound, 1]] for i in range(len(parents))]

            # Add slack constraints to Cplex object
            self.LowerModel.linear_constraints.add(names=slack_con,
                                                   lin_expr=slack_con_expr,
                                                   senses=slack_con_sense,
                                                   rhs=slack_con_rhs)

            # Define indicator constraint names
            child_con = ["i("+str(a.id)+")" for a in children]

            # Define interdependency constraints
            child_expr = [[[self.flow_vars[a.id]], [1]] for a in children]

            # Add interdependency indicator constraints to Cplex object
            self.LowerModel.indicator_constraints.add_batch(name=child_con,
                                             indvar=slack_vars,
                                             complemented=[0 for a in parents],
                                             lin_expr=child_expr,
                                             sense=["L" for a in parents],
                                             rhs=[0.0 for a in children])

        elif mode == 2:

            # LP formulation

            # The liner interdependency formulation does not require any
            # variables beyond the existing flow variables. We simply need to
            # define flow bounds that link pairs of interdependent arcs.

            # Define constraint names, senses, and righthand sides
            flow_int = ["i("+str(i)+")" for i in range(len(self.Net.int))]
            flow_int_sense = "L"*len(flow_int)
            flow_int_rhs = [0.0 for i in self.Net.int]

            # Define interdependency constraints (with variables on LHS)
            flow_int_expr = [[[], []] for i in self.Net.int]
            i = 0
            for intd in self.Net.int:

                # Get parent/child arc names
                var_pair = [self.flow_vars[intd[0].id],
                            self.flow_vars[intd[1].id]]

                # Set coefficients
                coef = [-(1.0*intd[1].bound/intd[0].bound), 1.0]

                # Update constraint list
                flow_int_expr[i] = [var_pair, coef]

                i += 1

            # Add interdependencies to model
            self.LowerModel.linear_constraints.add(names=flow_int,
                                                   lin_expr=flow_int_expr,
                                                   senses=flow_int_sense,
                                                   rhs=flow_int_rhs)

    #--------------------------------------------------------------------------
    def solve(self, defend, cutoff=100, gap=0.01, cplex_epsilon=0.001):
        """Bilevel subproblem solution method.

        The main cutting plane loop consists of alternating between solving the
        upper-level relaxed master problem and solving the lower-level response
        problem. The lower-level problem is adjusted to reflect the upper-level
        attack decisions, and its objective value represents the objective
        value associated with that attack vector, which then becomes a
        constraint in the relaxed master problem.

        The main loop of the cutting plane algorithm proceeds until either
        reaching an iteration cutoff or achieving a sufficiently small
        optimality gap, both of which can be adjusted. We can also terminate
        the loop early if the lower-level response is infeasible, in which case
        the upper-level objective is infinite.

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
                    3: Exit due to iteration cutoff
            iterations -- Number of iterations of main cutting plane loop.
        """

        # Set local variables
        obj_ub = -cplex.infinity # objective upper bound (upper-level problem)
        obj_lb = cplex.infinity # objective lower bound (lower-level problem)
        iteration= 1 # current iteration number
        status = 0 # exit code

        ###
        print("\nInitializing P2-3' cutting plane search.\n")
        print("-"*20+" Iteration 0 "+"-"*20)

        # Solve the upper-level problem once for the given defense vector
        (obj_ub, destroy) = self._upper_solve(defend=defend,
                                              cplex_epsilon=cplex_epsilon)

        ###
        print("rho2 = "+str(obj_ub))

        # Find the lower-level response for the given attack vector
        (obj_lb, nonzero, feasible) = self.lower_solve(destroy=destroy,
                                                   cplex_epsilon=cplex_epsilon)

        ###
        print("rho1 = "+str(obj_lb))

        obj_gap = abs(obj_ub - obj_lb) # current optimality gap

        ###
        print("Optimality gap = "+str(obj_gap))

        #----------------------------------------------------------------------
        # Main cutting plane loop begin

        while (iteration < cutoff) and (obj_gap > gap):

            iteration += 1

            ###
            print("-"*20+" Iteration "+str(iteration-1)+" "+"-"*20)

            # Add a constraint based on the nonzero flow vector
            self._upper_add_constraint(obj_lb, nonzero)

            # Re-solve the relaxed master problem
            (obj_ub, destroy) = self._upper_solve(cplex_epsilon=cplex_epsilon)

            ###
            print("rho2 = "+str(obj_ub))

            # Re-solve the lower-level response
            (obj_lb, nonzero, feasible) = self.lower_solve(destroy=destroy,
                                                   cplex_epsilon=cplex_epsilon)

            ### Include the potential to break here if LL is infeasible.
            if feasible == False:
                print("Response problem infeasible.")
                obj_ub = self.big_m
                obj_lb = self.big_m
                status = 1
                break

            ###
            print("rho1 = "+str(obj_lb))

            # Recalculate the optimality gap
            obj_gap = abs(obj_ub - obj_lb)

            ###
            print("Optimality gap = "+str(obj_gap))

            if (iteration >= cutoff) and (obj_gap > gap):
                status = 3

        # Main cutting plane loop end
        #----------------------------------------------------------------------

        return ((obj_ub+obj_lb)/2, destroy, status, iteration)

    #--------------------------------------------------------------------------
    def _upper_solve(self, defend=[], cplex_epsilon=0.001):
        """Solves the upper-level relaxed master MILP.

        Uses the upper-level Cplex object to solve the MILP defined by the
        current defense vector. This process involves cleaning up the model,
        modifying the constraints, calling the CPLEX solver, and then
        interpreting and returning the results.

        Accepts the following optional keyword arguments:
            defend -- Vector of defended arcs, as a boolean list. Defaults to
                an empty list, in which case no constraints are updated.
            cplex_epsilon -- Epsilon value for CPLEX solver's cleanup method.
                Values generated by the solver falling below this absolute
                value are deleted between solves. Defaults to 0.001.

        Returns a tuple containing the following elements:
            objective -- Objective value upper-level program.
            destroy -- Vector of arc destruction decisions, as a boolean list.
        """

        # Clean up the model
        self.UpperModel.cleanup(cplex_epsilon)

        # Update constraints based on arc defense vector
        if len(defend) == len(self.Net.def_arcs):
            new_rhs = [1 for a in self.Net.def_arcs]
            for i in range(len(new_rhs)):
                if defend[i] == True:
                    new_rhs[i] = 0
            self.UpperModel.linear_constraints.set_rhs([(self.att_con[i],
                           new_rhs[i]) for i in range(len(self.Net.def_arcs))])

        # Solve the MILP
        self.UpperModel.solve()

        # Get the objective value
        obj = self.UpperModel.solution.get_objective_value()

        # Set unbounded objective value to infinity (CPLEX returns an objective
        # of 0.0 for unbounded primal problems)
        if ((obj == 0.0) and
            (self.UpperModel.solution.is_primal_feasible() == True)):
            obj = cplex.infinity

        # Get the solution vector
        destroy = [False for a in self.Net.att_arcs]
        for i in range(len(self.Net.att_arcs)):
            if self.UpperModel.solution.get_values(self.att_vars[i]) == 1:
                destroy[i] = True

        return (obj, destroy)

    #--------------------------------------------------------------------------
    def lower_solve(self, destroy=[], cplex_epsilon=0.001):
        """Solves the lower-level interdependent network flows LP or MILP.

        Uses the lower-level Cplex object to solve the LP or MILP defined by
        the current attack vector. This process involves cleaning up the model,
        modifying the constraints, calling the CPLEX solver, and then
        interpreting and returning the results.

        Accepts the following optional keyword arguments:
            destroy -- Vector of destroyed arcs, as a boolean list. Defaults to
                an empty list, in which case no constraints are updated.
            cplex_epsilon -- Epsilon value for CPLEX solver's cleanup method.
                Values generated by the solver falling below this absolute
                value are deleted between solves. Defaults to 0.001.

        Returns a tuple containing the following elements:
            objective -- Objective value lower-level program.
            nonzero -- Vector indicating nonzero flow values, as a boolean
                list.
            feasible -- Indicator of whether the lower-level program is
                feasible.
        """

        # Clean up the model
        self.LowerModel.cleanup(cplex_epsilon)

        # Update constraints based on arc destruction vector
        if len(destroy) == len(self.Net.att_arcs):
            new_rhs = [a.bound for a in self.Net.att_arcs]
            for i in range(len(new_rhs)):
                if destroy[i] == True:
                    new_rhs[i] = 0
            self.LowerModel.linear_constraints.set_rhs([(self.flow_att[i],
                new_rhs[i]) for i in range(len(self.flow_att))])

        # Solve the LP or MILP
        self.LowerModel.solve()

        # Set up containers for objective, nonzero flow indicator, and
        # feasibility status
        obj = cplex.infinity
        nonzero = [False for a in self.Net.arcs]
        status = self.LowerModel.solution.is_primal_feasible()

        # Update outputs if the problem is feasible (if infeasible, they will
        # retain their initialized values)
        if status == True:
            obj = self.LowerModel.solution.get_objective_value()
            for i in range(len(self.Net.arcs)):
                if self.LowerModel.solution.get_values(self.flow_vars[i]) > 0:
                    nonzero[i] = True

        return (obj, nonzero, status)

    #--------------------------------------------------------------------------
    def _upper_add_constraint(self, objective, arcs):
        """Adds a constraint to the relaxed master problem.

        The constraints added to the relaxed master problem during the course
        of the cutting plane algorithm bound the upper level objective
        variable. They are based on the solutions of the lower-level problem,
        and consist of the objective value plus a series of penalty terms for
        each nonzero flow arc.

        Requires the following positional arguments:
            objective -- Objective value from lower-level program.
            arcs -- Vector of arcs to include in the penalty term, as a boolean
                list. This should correspond to the arcs which carried nonzero
                flow for the solution in question, so that attack vectors which
                destroy such arcs ignore the corresponding objective bound.
        """

        # Define new constraint variables and coefficients
        new_con_vars = [self.obj_var]
        new_con_coef = [1]
        for i in range(len(self.Net.att_arcs)):
            if arcs[self.Net.att_arcs[i].id] == True:
                new_con_vars.append(self.pen_vars[i])
                new_con_coef.append(-1)

        # Add constraints to Cplex object
        self.UpperModel.linear_constraints.add(names=[
                "s("+str(self.side_constraints)+")"],
                lin_expr=[[new_con_vars, new_con_coef]],
                senses=["L"], rhs=[objective])
        self.side_constraints += 1

    #--------------------------------------------------------------------------
    def end(self):
        """Closes all internal Cplex models.

        This should be called before the LLCuttingPlane object is discarded.
        """

        self.LowerModel.end()
        self.UpperModel.end()

###############################################################################
### For testing (delete later)

if __name__ == "__main__":
    import network.network as net
    TestNet = net.Network("../../../problems/smallnet.min")
    TestSolver = LLCuttingPlane(TestNet, 1)
    PrintSolver = LLCuttingPlane(TestNet, 2)
    #print(TestSolver.lower_solve())
    #print(TestSolver.lower_solve(destroy=[True, False, True, True, False,
    #                                       False, True, False, False]))
    #print(TestSolver._upper_solve(defend=[False, True, False, False, True,
    #                                      False, True]))
    #print(TestSolver.UpperModel.solution.is_primal_feasible())

    print(TestSolver.solve([False, False, False, False, True, False, False],
                           cutoff=20))
    print(TestSolver.UpperModel.solution.get_values())
    #print(TestSolver.UpperModel.get_problem_type())
    #print(TestSolver.UpperModel.solution.MIP.get_best_objective())

    TestSolver.LowerModel.write("ll_program.lp")
    PrintSolver.LowerModel.write("ll_lp_program.lp")
    TestSolver.UpperModel.write("ul_program.lp")

    TestSolver.end()
    PrintSolver.end()
