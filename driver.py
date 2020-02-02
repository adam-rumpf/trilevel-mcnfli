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
    [4] Type of interdependency ('a' for arcs, 'n' for sink nodes)
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

### Note: The current version of this program is equipped only to handle networks
### with parent arcs, since parent nodes require some restructuring when the
### network object is created.

import random
import solver

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
    order = [i for i in range(5)]
    random.shuffle(order)

    # Main trial loop
    for i in order:

        # Preliminary MILP solve
        if i == 0:
            pass

        # Defenseless MILP solve
        elif i == 1:
            pass

        # MILP cutting plane solve
        elif i == 2:

            sol = [0, 1, 1, 0]###

            # Write solution file
            _write_sol(output_directory+"milp_cp_sol.txt", input_file, sol,
                       overwrite=overwrite)

        # LP cutting plane solve
        elif i == 3:

            sol = [0, 1, 1, 0]###

            # Write solution file
            _write_sol(output_directory+"lp_cp_sol.txt", input_file, sol,
                       overwrite=overwrite)

        # LP duality solve
        elif i == 4:

            sol = [0, 1, 1, 0]###

            # Write solution file
            _write_sol(output_directory+"lp_dual_sol.txt", input_file, sol,
                       overwrite=overwrite)

    # Write summary file
    line = str(results[0])
    for i in range(1, len(results)):
        line += "\t" + str(results[i])
    _write_summary(output_directory+"summary.txt", line, overwrite=overwrite)

#==============================================================================
def _write_summary(file_name, line, overwrite=False):
    """Writes a single line to the summary output file.

    Requires the following positional arguments:
        file_name -- Reference to the output file name.
        line -- String to write to the output file.

    Accepts the following optional keyword arguments:
        overwrite -- Selects whether to overwrite the existing output file. If
            True, creates a new file with a new comment line and deletes any
            existing copy. If False, appends line to an existing file. Defaults
            to False.
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
        print(line, file=f)

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
    line = trial_name + "\t" + str(vector[0])
    for i in range(1, len(vector)):
        line += "\t" + str(vector[i])

    # Print new line
    with open(file_name, mode=m) as f:
        print(line, file=f)

###############################################################################
### For testing (delete later)

#single_trial("problems/smallnet.min", "results/")

### This main driver should instantiate a different TrialSolver object for each
### trial instance, calling each of its methods one-at-a-time.

# Handle all of the file accessing and writing. Refer to an external list with
# the names of all files to be read, since this list can have elements
# removed from it as trials are completed

# The main solver method should ask for a .txt file which lists a set of
# NETGEN files. The driver then calls solver.py for each of the listed files in
# turn, using all of its solver methods to apply each of the solution
# algorithms, and then printing the results to a specified set of output .txt
# files.
# Include the following outputs (also include an option to overwrite):
# summary.txt
# file, nodes, arcs, int, type, defense, attack, milp_obj_init, milp_obj_nodef,
# milp_cp_time, milp_cp_iter, milp_cp_iter_lower, milp_obj lp_cp_time,
# lp_cp_iter, lp_cp_iter_lower, lp_dual_time, lp_dual_iter, lp_obj, lp_milp_obj
# milp_cp_sol.txt
# file, [sol]
# lp_cp_sol.txt
# file, [sol]
# lp_dual_sol.txt
# file, [sol]

single_trial("problems/smallnet.min", "results/", overwrite=True)
#_write_summary("results/test.txt", "Test string...", overwrite=True)
