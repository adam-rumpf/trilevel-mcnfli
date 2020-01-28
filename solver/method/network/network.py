"""A simple network class for use in setting up network flows problems.

The classes in this module are mostly meant as containers that can store node-
level and arc-level attributes in a convenient way for the purposes of building
objective functions and constraints for network flows functions.

This module is meant for use in storing modified network flows problems defined
in the NETGEN file format, and so most methods used for defining network
features are written under the assumption that the input file is being read
line-by-line.
"""

#==============================================================================
class Network:
    """A class for representing a flow network defined by a NETGEN file.

    The constructor requires an input file for defining the network, which is
    assumed to remain constant for the lifespan of the Network object. This
    data should be treated as read-only, and there are no methods for modifying
    the structure of the network after it is created.

    Methods are included to return sets needed for constructing the objective
    and constraints of a network flows problem.
    """

    #--------------------------------------------------------------------------
    def __init__(self, file):
        """Network object constructor for reading a specified NETGEN file.

        A Network object is meant to store the information defined in a NETGEN
        file. The constructor reads the contents of the file and generates all
        necessary Node, Arc, and other objects.

        Requires the following positional arguments:
            file -- Complete file path for the NETGEN file containing the
                network definition.
        """

        pass

#==============================================================================
class _Node:
    """A private class for representing nodes within a Network object."""

    #--------------------------------------------------------------------------
    def __init__(self):
        """Node object constructor."""

        pass

#==============================================================================
class _Arc:
    """A private class for representing arcs within a Network object."""

    #--------------------------------------------------------------------------
    def __init__(self):
        """Arc object constructor."""

        pass
