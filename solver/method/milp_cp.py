"""Cutting plane solution algorithm for the lower-level bilevel program (MILP).

Includes a MILPCuttingPLane class which applies the cutting plane solution
method given a protection vector. Returns the objective value and attack vector
obtained from the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the MILP
binary interdependence model.
"""

if __name__ == "__main__":
    import network.network as net

#==============================================================================
class MILPCuttingPlane:
    """Class to implement the cutting plane method for the MILP lower model."""

    #--------------------------------------------------------------------------
    def __init__(self, net_in):
        """MILP cutting plane solution object constructor.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
        """

        pass
