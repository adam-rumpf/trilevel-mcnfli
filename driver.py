"""Computational trial driver for trilevel network interdiction game.

Drives the computational trial processing by reading a list of problem file
locations and calling the main solver driver for each. Also handles writing the
result logs.

This module contains several trial driver functions. Each requires a reference
to an output folder for printing output files. Options exist to process either
a single referenced input file or a list of references to input files. All
input files should follow the NETGEN format, modified to include
interdependencies and interdiction game parameters.

Most of the output files consist of rows of solutions arranged as tab-separated
rows, with a file name comment in the first column. The summary file includes
a list of results for each trial network test as a separate row. The columns
are arranged as follows:
    [0] File name of trial instance
    [1] Number of nodes
    [2] Number of arcs
    [3] Number of interdependencies
    [4] Type of interdependency (0 for sink nodes, 1 for arcs)
    [5] Number of allowed defenses
    [6] Number of allowed attacks
    [7] Initial objecive value of underlying MILP
    [8] Objective value of MILP after attacks with no defense
    [9] Solution time for MILP via cutting plane
    [10] Upper trilevel iterations for MILP via cutting plane
    [11] Lower trilevel iterations for MILP via cutting plane
    [12] Objective value of MILP game
    [13] Solution time for LP via cutting plane
    [14] Upper trilevel iterations for LP via cutting plane
    [15] Lower trilevel iterations for LP via cutting plane
    [16] Solution time for LP via duality
    [17] Upper trilevel iterations for LP via duality
    [18] Objective value of LP game
    [19] Objective value of MILP game solved with LP defensive decisions
"""

import gc
import random

import solver.solver as sl

#==============================================================================
def single_trial(input_file, output_directory, overwrite=False):
    """Processes a single trial instance.

    Runs a collection of tests on a single trial instance and writes the
    results to a collection of output files.

    Requires the following positional arguments:
        input_file -- Reference to an input NETGEN .min file to process.
        output_directory -- Reference to an output directory, as a destination
            for the output files.

    Accepts the following optional keyword arguments:
        overwrite -- Selects whether to overwrite any existing output files in
            the output directory. If True, new output files will be created and
            existing output files will be deleted. If False, new results will
            be appended to existing output files. Defaults to False.
    """

    # Initialize summary line contents
    results = [0 for i in range(20)]
    results[0] = input_file

    # Process trials in a semirandomized order in a loop
    order = [i for i in range(2, 5)]
    random.shuffle(order)
    order = [0, 1] + order + [5]

    # Initialize LP defensive decisions
    lp_sol = []

    ###
    order = [0, 1]

    # Main trial loop
    for i in order:

        # Collect garbage
        gc.collect()

        # Preliminary MILP solve (always first)
        if i == 0:

            # Initialize temporary solver
            Model = sl.TrialSolver(input_file)

            # Get network statistics
            results[1] = len(Model.Net.nodes)
            results[2] = len(Model.Net.arcs)
            results[3] = len(Model.Net.int)
            results[4] = Model.Net.parent_type
            results[5] = Model.Net.def_limit
            results[6] = Model.Net.att_limit

            # Solve initial MILP
            (obj, feas) = Model.solve_milp_initial()

            # Break if the model is infesible
            if feas != 0:
                break

            # Record objective
            results[7] = obj

        # Defenseless MILP solve (always second)
        elif i == 1:

            # Initialize temporary solver
            Model = sl.TrialSolver(input_file)

            # Solve bilevel submodel with no defense
            (obj, _, _, _) = Model.solve_milp_defend([])

            # Record objective
            results[8] = obj

        # MILP cutting plane solve
        elif i == 2:

            # Initialize temporary solver
            Model = sl.TrialSolver(input_file)

            # Solve trilevel model
            (obj, sol, _, times, itera) = Model.solve_milp_cutting_plane()

            # Record trial statistics
            results[9] = times[0]
            results[10] = itera[0]
            results[11] = itera[1]
            results[12] = obj

            # Write solution file
            for i in range(len(sol)):
                if sol[i] == True:
                    sol[i] = 1
                else:
                    sol[i] = 0
            _write_sol(output_directory+"milp_cp_sol.txt", input_file, sol,
                       overwrite=overwrite)

        # LP cutting plane solve
        elif i == 3:

            # Initialize temporary solver
            Model = sl.TrialSolver(input_file)

            # Solve trilevel model
            (obj, sol, _, times, itera) = Model.solve_lp_cutting_plane()
            lp_sol = sol # save solution for comparison with MILP

            # Record trial statistics
            results[13] = times[0]
            results[14] = itera[0]
            results[15] = itera[1]
            results[18] = obj

            # Write solution file
            for i in range(len(sol)):
                if sol[i] == True:
                    sol[i] = 1
                else:
                    sol[i] = 0
            _write_sol(output_directory+"lp_cp_sol.txt", input_file, sol,
                       overwrite=overwrite)

        # LP duality solve
        elif i == 4:

            # Initialize temporary solver
            Model = sl.TrialSolver(input_file)

            # Solve trilevel model
            (_, sol, _, times, itera) = Model.solve_lp_duality()

            # Record trial statistics
            results[16] = times[0]
            results[17] = itera[0]

            # Write solution file
            for i in range(len(sol)):
                if sol[i] == True:
                    sol[i] = 1
                else:
                    sol[i] = 0
            _write_sol(output_directory+"lp_dual_sol.txt", input_file, sol,
                       overwrite=overwrite)

        # Relaxed solution optimality gap solve (always last)
        elif i == 5:

            # Initialize temporary solver
            Model = sl.TrialSolver(input_file)

            # Solve bilevel model with LP defense
            (obj, _, _, _) = Model.solve_milp_defend(lp_sol)

            # Record objective
            results[19] = obj

    # Write summary file
    line = str(results[0])
    for i in range(1, len(results)):
        line += "\t" + str(results[i])
    _write_summary(output_directory+"summary.txt", line, overwrite=overwrite)

#==============================================================================
def trial_list(input_list, output_directory, overwrite=False):
    """Processes a list of trial instances.

    Runs a collection of tests on a list of trial instances and writes the
    results to a collection of output files. This works by calling the single
    trial processer iteratively for every file on the list.

    Requires the following positional arguments:
        input_list -- Reference to a list of files to process. The list should
            include a complete file path on each line.
        output_directory -- Reference to an output directory, as a destination
            for the output files.

    Accepts the following optional keyword arguments:
        overwrite -- Selects whether to overwrite any existing output files in
            the output directory. Defaults to False. If True, any existing
            output files will be overwritten before beginning the trial set.
            All remaining trials in the set will be added to the existing
            output files.
    """

    over = overwrite # initial overwrite setting

    # Read input file into a list
    trials = []
    with open(input_list, 'r') as f:
        for line in f:
            trials.append(line[:-1])

    # Process each trial on the list
    for i in range(len(trials)):
        single_trial(trials[i], output_directory, overwrite=over)
        over = False # stop overwriting after first trial

    print("\nAll trials processed!")

#==============================================================================
def _write_summary(file_name, line, overwrite=False, end=None):
    """Writes a single line to the summary output file.

    Requires the following positional arguments:
        file_name -- Reference to the output file name.
        line -- String to write to the output file.

    Accepts the following optional keyword arguments:
        overwrite -- Selects whether to overwrite the existing output file. If
            True, creates a new file with a new comment line and deletes any
            existing copy. If False, appends line to an existing file. Defaults
            to False.
        end -- Overrides the end-of-line character when printing lines.
    """

    # If overwriting, set file writing mode and write comment lines
    if overwrite == True:
        with open(file_name, mode='w') as f:
            print("file\tnodes\tarcs\tint\ttype\tdefense\tattack\t"+
                  "milp_obj_init\tmilp_obj_nodef\tmilp_cp_time\t"+
                  "milp_cp_iter\tmilp_cp_iter_lower\tmilp_obj\tlp_cp_time\t"+
                  "lp_cp_iter\tlp_cp_iter_lower\tlp_dual_time\tlp_dual_iter\t"+
                  "lp_obj\tlp_milp_obj", file=f)

    # Print new line
    with open(file_name, mode='a') as f:
        if end == None:
            print(line, file=f)
        else:
            print(line, file=f, end=end)

#==============================================================================
def _write_sol(file_name, trial_name, vector, overwrite=False):
    """Writes a single line to a solution output file.

    Requires the following positional arguments:
        file_name -- Reference to the output file name.
        trial_name -- String to use as a row label.
        vector -- Vector representing the solution to be recorded.

    Accepts the following optional keyword arguments:
        overwrite -- Selects whether to overwrite the existing output file. If
            True, creates a new file with a new comment line and deletes any
            existing copy. If False, appends line to an existing file. Defaults
            to False.
    """

    # Set file writing mode based on ovewrite option
    m = 'a'
    if overwrite == True:
        m = 'w'

    # Convert solution to a string
    if len(vector) > 0:
        line = trial_name + "\t" + str(vector[0])
        for i in range(1, len(vector)):
            line += "\t" + str(vector[i])
    else:
        line = ""

    # Print new line
    with open(file_name, mode=m) as f:
        print(line, file=f)

#==============================================================================
def refresh_files(directory):
    """Generates empty output files in an output directory.

    Generates (and overwrites) the summary file and method solution files in a
    specified directory.

    Requires the following positional arguments:
        directory -- Name of output directory.
    """

    # Write empty files
    _write_summary(directory+"summary.txt", "", overwrite=True, end="")
    with open(directory+"milp_cp_sol.txt", 'w'):
        pass
    with open(directory+"lp_cp_sol.txt", 'w'):
        pass
    with open(directory+"lp_dual_sol.txt", 'w'):
        pass

###############################################################################
### For testing (delete later)

#single_trial("problems/smallnet.min", "results/", overwrite=True)
#refresh_files("results/")
#trial_list("trial_list.txt", "results/", overwrite=True)

testfiles = ["problems/smalltest.min", "problems/bigtest.min"]
for tf in testfiles:
    print("\n"+"#"*60+"\nTesting "+tf+"\n"+"#"*60+"\n")
    single_trial(tf, "results/", overwrite=True)
