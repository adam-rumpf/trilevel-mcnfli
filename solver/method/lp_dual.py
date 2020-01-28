"""Duality solution algorithm for the lower-level bilevel program (LP).

Includes an LPDuality class which applies the duality solution method given a
protection vector. Returns the objective value and attack vector obtained from
the lower-level bilevel maximization.

Requires a reference to the main Network object, which includes all needed
information to define the trilevel network interdiction game based on the LP
relaxation of the binary interdependence model.
"""

if __name__ == "__main__":
    import network.network as net

#==============================================================================
class LPDuality:
    """Class to implement the duality method for the LP lower model."""

    #--------------------------------------------------------------------------
    def __init__(self, net_in):
        """LP duality solution object constructor.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
        """

        pass
