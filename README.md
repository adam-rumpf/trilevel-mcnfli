# Trilevel Network Interdiction Game with Linear Interdependencies

A collection of programs used to generate computational results for a research project of mine involving a trilevel network interdiction game on an interdependent network. This project is an extension of the MCNFLI project referenced [here](https://github.com/adam-rumpf/mcnfli-trials) and uses the same NETGEN code for random trial generation. I would not expect these programs to be of use to anyone outside of my research group, but they are provided here for anyone interested.

This module makes use of the [CPLEX Python API](https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/GettingStarted/topics/set_up/Python_setup.html) for solving the various linear and mixed-integer programs involved in the trilevel game.

## Module Structure

This module includes a variety of submodules for running various aspects of the computational trial evaluation process. They are organized as follows:

* `base/`
  * `solver/`
    * `method/`
      * `network/`
        * `network.py`: Network object module for use in storing the network defined by the NETGEN input file as well as additional problem info.
      * `lp_dual.py`: Duality algorithm for the lower-level bilevel program with the linear interdependence model as its lower level.
      * `milp_lp_cp.py`: Cutting plane algorithm for the lower-level bilevel program with either the binary or the linear interdependence model as its lower level (class constructor method selects whether to use LP or MILP model).
    * `upper_cp.py`: Driver for the upper-level cutting plane algorithm.
  * `solver.py`: Solution algorithm driver. Used to call each solution algorithm in turn and store its results.
* `driver.py`: Main driver. Processes each computational trial file one-by-one to apply all available solution methods and record the results.

## Input Format

The solution algorithm driver, `solver.py`, requires a reference to a modified NETGEN `.min` file to define a problem instance. The main driver, `driver.py`, requires a reference to a folder of `.min` files and uses `solver.py` to process each of them in turn as a batch.

Note: The current version of this module includes only support for the arc parent interdependency model.

## Output Format

The main driver, `driver.py`, writes the results of the trials to a collection of output files in a specified folder. All output files include a row for each trial.

The result summary file `summary.txt` includes the following tab-separated columns:

* `file`: Name of the trial's input file.
* `nodes`: Number of nodes in the input network.
* `arcs`: Number of arcs in the input network.
* `int`: Number of interdependencies in the input network.
* `type`: Type of interdependency (`a` for arc parents, `n` for node parents).
* `defense`: Number of arcs that may be defended.
* `attack`: Number of arcs that may be attacked.
* `milp_obj_init`: Initial objective value of the binary interdependence model before any attacks are made.
* `milp_obj_nodef`: Objective value of the binary interdependence model if it is attacked with no defensive decisions made.
* `milp_cp_time`: Total time spent to solve binary interdependence model via cutting plane.
* `milp_cp_iter`: Number of cutting plane iterations required for the overall binary interdependence model.
* `milp_cp_iter_lower`: Number of cutting plane iterations required for the lower bilevel binary interdependence model.
* `milp_obj`: Objective value of binary interdependence model.
* `lp_cp_time`: Total time spent to solve linear interdependence model via cutting plane.
* `lp_cp_iter`: Number of cutting plane iterations required for the overall linear interdependence model with cutting plane bilevel submodel.
* `lp_cp_iter_lower`: Number of cutting plane iterations required for the lower bilevel linear interdependence model.
* `lp_dual_time`: Total time spent to solve linear interdependence model via duality.
* `lp_dual_iter`: Number of cutting plane iterations required for the overall linear interdependence model with duality bilevel submodel.
* `lp_obj`: Objective value of linear interdependence model.
* `lp_milp_obj`: Objective value of binary interdependence model resulting from the linear interdependence model's defensive decisions.

There is also an output file for each of the three solution methods (`milp_cp_sol.txt`, `lp_cp_sol.txt`, and `lp_dual_sol.txt`) which lists the complete defense solution vector. Each of these files includes a column to indicate the input file name, followed by tab-separated columns to indicate the complete binary solution vector arranged in order of arc index.
