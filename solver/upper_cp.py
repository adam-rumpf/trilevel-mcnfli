"""Upper level cutting plane driver for trilevel solution algorithm.

Drives the upper-level portion of the cutting plane solution algorithm, which
consists of solving a bilevel program whose lower level is the bilevel program
formed by the lower two levels.

All solution algorithms include exactly the same upper level, and differ only
in their solutions of the lower-level problem. Different methods are included
here to select which solution method to use for the lower-level.
"""

if __name__ == "__main__":
    import method.milp_cp as milpcp
    import method.lp_cp as lpcp
    import method.lp_dual as lpd
    import method.network.network as net

#==============================================================================
class UpperLevel:
    """Class to drive the upper-level cutting plane algorithm."""

    #--------------------------------------------------------------------------
    def __init__(self, net_in, method):
        """Solution object constructor.

        Requires the following positional arguments:
            net_in -- Reference to the Network object that defines the problem
                instance.
            method -- Integer argument to indicate which lower-level method to
                apply, indexed as follows:
                    1: MILP cutting plane
                    2: LP cutting plane
                    3: LP duality
        """

        pass

# Include separate timers for the upper-level cutting plane portion and the
# lower-level solves, and have them increment separate counts.
