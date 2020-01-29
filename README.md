# Trilevel Network Interdiction Game with Linear Interdependencies

A collection of programs used to generate computational results for a research project of mine involving a trilevel network interdiction game on an interdependent network. This project is an extension of the MCNFLI project referenced [here](https://github.com/adam-rumpf/mcnfli-trials) and uses the same NETGEN code for random trial generation. I would not expect these programs to be of use to anyone outside of my research group, but they are provided here for anyone interested.

## Module Structure

This module includes a variety of submodules for running various aspects of the computational trial evaluation process. They are organized as follows:

* `solver/`
  * `method/`
    * `network/`
      * `network.py`: Network object module for use in storing the network defined by the NETGEN input file as well as additional problem info.
    * `lp_cp.py`: Cutting plane algorithm for the lower-level bilevel program with the linear interdependence model as its lower level.
    * `lp_dual.py`: Duality algorithm for the lower-level bilevel program with the linear interdependence model as its lower level.
    * `milp_cp.py`: Cutting plane algorithm for the lower-level bilevel program with the binary interdependence model as its lower level.
  * `upper_cp.py`: Driver for the upper-level cutting plane algorithm.
* `driver.py`: Main driver. Processes each computational trial file one-by-one to apply all available solution methods and record the results.
* `solver.py`: Solution algorithm driver. Used to call each solution algorithm in turn and store its results.
