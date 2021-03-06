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

import upper.lower.network.network as net
import upper.lower.milp_lp_cp as milpcp
import upper.lower.lp_dual as lpd

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
    called on its own to evaluate a single defensive decision, for example to
    evaluate the optimality gap of a heuristically-chosen defense.
    """

    #--------------------------------------------------------------------------
    def __init__(self, net_in, method, big_m=1.0e16, small_m=1.0e10):
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
            small_m -- Big-M constant for use in the lower-level model. Should
                still be larger than any reasonable values produced by the
                solution algorithm, but significantly smaller than big_m.
                Defaults to 1.0e10.
        """

        self.Net = net_in # set reference to network object
        self.big_m = big_m # penalty value for use in relaxed master problem
        self.med_m = 0.001*big_m # penalty value slightly less than big-M
        self.method = method # store method ID for reference

        # Initialize the chosen type of lower-level solver
        if self.method in {1, 2}:
            self.LowerLevel = milpcp.LLCuttingPlane(self.Net, self.method,
                                                    big_m=small_m)
        elif self.method == 3:
            self.LowerLevel = lpd.LLDuality(self.Net, big_m=small_m)

        # Initialize top-level Cplex object
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
        self.def_vars = ["df("+str(a.id)+")" for a in self.Net.def_arcs]
        self.pen_vars = ["pt("+str(a.id)+")" for a in self.Net.def_arcs]

        # Solve lower-level response model for an objective lower bound
        (obj_lb, _) = self.initial_solve()

        # Add objective bound variable to Cplex object, with a finite but large
        # lower bound in order to ensure dual feasibility
        self.TopModel.variables.add(obj=[1.0], names=[self.obj_var],
                                    lb=[obj_lb],
                                    ub=[cplex.infinity])

        # Add binary defense decision variables to Cplex object
        self.TopModel.variables.add(names=self.def_vars,
                                    types="B"*len(self.def_vars))

        # Add penalty variables to Cplex object
        self.TopModel.variables.add(names=self.pen_vars,
                                    lb=[0.0 for a in self.Net.def_arcs],
                                    ub=[self.big_m for a in self.Net.def_arcs])

        # Define a list of penalty variable indicator constraint names
        pen_con = ["dp("+str(a.id)+")" for a in self.Net.def_arcs]

        # Define defense constraints to limit the total number of defenses
        def_lim_expr = [[[v for v in self.def_vars],
                         [1 for v in self.def_vars]]]

        # Define penalty variable constraints to limit value when activated
        pen_expr = [[[v], [1]] for v in self.pen_vars]

        # Add defense constraints to Cplex object
        self.TopModel.linear_constraints.add(names=["db"],
                                             lin_expr=def_lim_expr,
                                             senses=["L"],
                                             rhs=[self.Net.def_limit])

        # Add penalty variable indicator constraints to Cplex object
        for i in range(len(pen_con)):
            self.TopModel.indicator_constraints.add(name=pen_con[i],
                                                    indvar=self.def_vars[i],
                                                    complemented=1,
                                                    lin_expr=pen_expr[i],
                                                    sense="L",
                                                    rhs=0.0)

        # Keep track of the number of side constraints generated so far
        self.side_constraints = 0

    #--------------------------------------------------------------------------
    def solve(self, cutoff=100, lower_cutoff=100, gap=0.01, lower_gap=0.01,
              cplex_epsilon=0.001):
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
            cplex_epsilon -- Epsilon value for CPLEX solver's cleanup method.
                Values generated by the solver falling below this absolute
                value are deleted between solves. Defaults to 0.001.

        Returns a tuple containing the following elements:
            objective -- Objective value of the trilevel program.
            defend -- Vector of defended arcs, as a boolean list.
            destroy -- Vector of destroyed arcs, as a boolean list.
            times -- Tuple of cumulative times spent on various parts of the
                solution process, including: (upper level, lower level)
            iterations -- Tuple of iterations of each cutting plane loop (upper
                level then lower level), with -1 if not applicable (for example
                in the case of the duality lower-level program)
            exit_status -- Numerical code to describe the results of the
                solution process, including the following:
                    0: Successful exit with finite objective value.
					1: Exit due to infeasible defense problem.
                    2: Exit due to error.
                    3: Exit due to iteration cutoff
        """

        # Set local variables
        obj_ub = -cplex.infinity # objective upper bound (lower-level problem)
        obj_lb = cplex.infinity # objective lower bound (upper-level problem)
        iteration= 1 # current iteration number
        upper_time = 0 # total time spent solving the upper-level models
        lower_time = 0 # total time spent solving the lower-level models
        exit_status = 0

        ###
        print("P1-3>\tIteration "+str(iteration-1))

        # Solve the upper-level problem for an initial solution
        timer = time.time()
        (obj_lb, defend) = self._upper_solve(cplex_epsilon=cplex_epsilon)
        upper_time += time.time() - timer

        ###
        print("  P1>\tobj = "+str(obj_lb))

        # Find the lower-level response for the given attack vector
        timer = time.time()
        (obj_ub, destroy, status, lower_iterations) = self.lower_solve(defend,
               cutoff=lower_cutoff, gap=lower_gap, cplex_epsilon=cplex_epsilon)
        lower_time += time.time() - timer

        # Bound returned objective by a large constant
        obj_ub = min(obj_ub, self.med_m)

        ###
        print("  P2>\t\tobj = "+str(obj_ub))

        obj_gap = abs(obj_ub - obj_lb) # current optimality gap

        ###
        print("P1-3>\tgap = "+str(obj_gap))

        #----------------------------------------------------------------------
        # Main cutting plane loop begin

        while (iteration < cutoff) and (obj_gap > gap):

            iteration += 1

            ###
            print("P1-3>\tIteration "+str(iteration-1))

            # Add a constraint based on the nonzero flow vector
            self._add_constraint(obj_ub, destroy)

            # Re-solve the relaxed master problem
            timer = time.time()
            (obj_lb, defend) = self._upper_solve(cplex_epsilon=cplex_epsilon)
            upper_time += time.time() - timer
			
            # Break if upper level cannot improve on the infeasibility bound
            if (obj_lb >= self.med_m) or (obj_lb >= obj_ub):
                obj_ub = cplex.infinity
                obj_lb = cplex.infinity
                exit_status = 1
                break

            ###
            print("  P1>\tobj = "+str(obj_lb))

            # Re-solve the lower-level response
            timer = time.time()
            (obj_ub, destroy, status, li) = self.lower_solve(defend,
               cutoff=lower_cutoff, gap=lower_gap, cplex_epsilon=cplex_epsilon)
            lower_time += time.time() - timer
            lower_iterations += li

            # Break in case of lower-level error
            if status == 2:
                exit_status = 2
                break

            # Bound returned objective by a large constant
            obj_ub = min(obj_ub, self.med_m)

            ###
            print("  P2>\t\tobj = "+str(obj_ub))

            # Recalculate the optimality gap
            obj_gap = abs(obj_ub - obj_lb)

            ###
            print("P1-3>\tgap = "+str(obj_gap))

            if (iteration >= cutoff) and (obj_gap > gap):
                # If ending due to iteration cutoff without reaching optimality
                # gap, use average of bounds as the best guess
                exit_status = 3
                obj_ub = (obj_ub+obj_lb)/2

        # Main cutting plane loop end
        #----------------------------------------------------------------------

        return (obj_ub, defend, destroy, (upper_time, lower_time), (iteration,
                lower_iterations), exit_status)

    #--------------------------------------------------------------------------
    def _upper_solve(self, cplex_epsilon=0.001):
        """Solves the upper-level relaxed master MILP.

        Uses the upper-level Cplex object to solve the MILP defined by the
        current relaxed master problem of the top level of the trilevel model.
        This process involves cleaning up the model, modifying the constraints,
        calling the CPLEX solver, and then interpreting and returning the
        results.

        Accepts the following optional keyword arguments:
            cplex_epsilon -- Epsilon value for CPLEX solver's cleanup method.
                Values generated by the solver falling below this absolute
                value are deleted between solves. Defaults to 0.001.

        Returns a tuple containing the following elements:
            objective -- Objective value upper-level program.
            defend -- Vector of arc defense decisions, as a boolean list.
        """

        # Clean up the model
        self.TopModel.cleanup(cplex_epsilon)

        # Solve the MILP
        self.TopModel.solve()

        # Get the objective value
        obj = self.TopModel.solution.get_objective_value()

        # Get the solution vector
        defend = [False for a in self.Net.def_arcs]
        for i in range(len(self.Net.def_arcs)):
            if self.TopModel.solution.get_values(self.def_vars[i]) == 1:
                defend[i] = True

        return (obj, defend)

    #--------------------------------------------------------------------------
    def lower_solve(self, defend, cutoff=100, gap=0.01, cplex_epsilon=0.001):
        """Calculates the lower-level solution for a given defensive decision.

        Requires the following positional arguments:
            defend -- Vector of defended arcs, as a boolean list.

        Accepts the following optional keyword arguments:
            cutoff -- Iteration cutoff for the overall cutting plane main loop.
                Defaults to 100. Used only for cutting plane lower-level
                methods.
            gap -- Optimality gap tolerance for the overall cutting plane main
                loop. Defaults to 0.01. Used only for cutting plane lower-
                level methods.
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
            iterations -- Number of iterations of the lower-level algorithm's
                cutting plane loop (0 if not applicable).
        """

        # Call the appropriate lower-level solver and return its results
        return self.LowerLevel.solve(defend, cutoff=cutoff, gap=gap,
                                     cplex_epsilon=cplex_epsilon)

    #--------------------------------------------------------------------------
    def _add_constraint(self, objective, arcs):
        """Adds a constraint to the relaxed master problem.

        The constraints added to the relaxed master problem during the course
        of the cutting plane algorithm bound the upper level objective
        variable. They are based on the solutions of the lower-level problem,
        and consist of the objective value plus a series of penalty terms for
        each destroyed arc.

        Requires the following positional arguments:
            objective -- Objective value from lower-level program.
            arcs -- Vector of arcs to include in the penalty term, as a boolean
                list. This should correspond to the arcs destroyed for the
                solution in question, so that defense vectors which defend such
                arcs ignore the corresponding objective bound.
        """

        # Define new constraint variables and coefficients
        new_con_vars = [self.obj_var]
        new_con_coef = [1]
        for i in range(len(self.Net.def_arcs)):
            if arcs[i] == True:
                new_con_vars.append(self.pen_vars[i])
                new_con_coef.append(1)

        # Set infinite objectives to big-M
        obj = min(objective, self.med_m)

        # Add constraints to Cplex object
        self.TopModel.linear_constraints.add(names=[
                "s("+str(self.side_constraints)+")"],
                lin_expr=[[new_con_vars, new_con_coef]],
                senses=["G"], rhs=[obj])
        self.side_constraints += 1

    #--------------------------------------------------------------------------
    def initial_solve(self):
        """Solves the initial lower-level network flows problem.

        The initial value of the final response problem gives a lower bound to
        the defender's objective value. We begin the solution process of the
        overall trilevel model by solving the underlying MILP or LP with no
        attack decisions made.

        Returns a tuple containing the following elements:
            objective -- Objective value of the lower-level bilevel program.
            status -- Numerical code to describe the results of the solution
                process, including the following:
                    0: Successful exit with feasible MILP.
                    1: Successful exit with infeasible MILP.
                    2: Exit due to error.
        """

        if self.method in {1, 2}:
            # If using the LP or MILP cutting plane method, we can use the
            # existing lower-level model object's own methods
            (obj, _, feas) = self.LowerLevel.lower_solve(destroy=[])
        elif self.method == 3:
            # Otherwise we instantiate a temporary LP solver object to
            # calculate the initial value
            Lp = milpcp.LLCuttingPlane(self.Net, 2)
            (obj, _, feas) = Lp.lower_solve(destroy=[])
            Lp.end()

        status = 0
        if feas == False:
            status = 1

        return (obj, status)

    #--------------------------------------------------------------------------
    def end(self):
        """Closes all internal Cplex models.

        This should be called before the UpperLevel object is discarded. It
        also ends the Cplex models of the lower-level objects.
        """

        self.TopModel.end()
        self.LowerLevel.end()
