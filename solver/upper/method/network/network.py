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
        """Network object constructor for initializing containers.

        A Network object is meant to store the information defined in a NETGEN
        file. The constructor initializes its internal attributes and then
        calls a private method for reading the contents of the NETGEN file.

        We assume that the problem is always a minimization and so ignore the
        sense listed in the NETGEN file.

        Requires the following positional arguments:
            file -- Complete file path for the NETGEN file containing the
                network definition.
        """

        # Initialize internal lists
        self.nodes = [] # node object set
        self.arcs = [] # arc object set
        self.int = [] # list of parent/child Arc tuples
        self.def_arcs = [] # defensible arc set
        self.att_arcs = [] # destructible arc set

        # Initialize constants
        self.def_limit = 0 # number of allowed arc defenses
        self.att_limit = 0 # number of allowed arc attacks
        self.parent_type = 0 # 0 for sink node parents, 1 for arc parents

        # Read input file
        self._read_netgen(file)

    #--------------------------------------------------------------------------
    def _read_netgen(self, file):
        """Reads the contents of a NETGEN file to fill internal containers.

        The expected format of NETGEN file is the modified format which
        includes interdependency definitions and defender/attacker decision
        constraints.

        Note that the NETGEN file format indexes its nodes and arcs beginning
        with 1. The indices used here will be automatically reduced by 1 to
        correspond to list positions.

        Requires the following positional arguments:
            file -- Complete file path for the NETGEN file containing the
                network definition.
        """

        with open(file, 'r') as f:

            aid = 0 # current arc ID

            # Read the file line-by-line
            for line in f:

                # Decide what to do based on the line prefix

                # Comment line
                if line[0] == 'c':
                    # Skip
                    continue

                # Problem info
                elif line[0] == 'p':
                    # p sense #nodes #arcs #int int_type #defenses #attacks
                    # We always assume that the sense is minimization

                    ls = line.split()
                    if ls[5] == 'a':
                        self.parent_type = 1
                    self.def_limit = int(ls[6])
                    self.att_limit = int(ls[7])

                    # Initialize all nodes as transshipment (in case the NETGEN
                    # file lists only nonzero supply values)
                    self.nodes = [_Node(i, 0.0) for i in range(int(ls[2]))]

                # Node
                elif line[0] == 'n':
                    # n ID supply

                    # All nodes have already been defined, so update existing
                    # supply values

                    ls = line.split()
                    self.nodes[int(ls[1])-1].supply = float(ls[2])

                # Arc
                elif line[0] == 'a':
                    # a tail head LB UB cost

                    ls = line.split()
                    tail = self.nodes[int(ls[1])-1]
                    head = self.nodes[int(ls[2])-1]
                    self.arcs.append(_Arc(aid, tail, head, float(ls[4]),
                                          float(ls[5])))
                    aid += 1

                # Interdependency
                elif line[0] == 'i':
                    # i parent child

                    ls = line.split()
                    self.int.append((self.arcs[int(ls[1])-1],
                                     self.arcs[int(ls[2])-1]))

                # Defensible arc
                elif line[0] == 'd':
                    # d arc

                    ls = line.split()
                    self.def_arcs.append(self.arcs[int(ls[1])-1])

                    # All defensible arcs are assumed to be destructible
                    self.att_arcs.append(self.arcs[int(ls[1])-1])

                # Destructible arc
                elif line[0] == 'r':
                    # r arc

                    ls = line.split()
                    self.att_arcs.append(self.arcs[int(ls[1])-1])

            # If no defensible or destructible arcs were listed, we assume
            # that all arcs are available

            if len(self.def_arcs) == 0:
                self.def_arcs[:] = self.arcs[:]

            if len(self.att_arcs) == 0:
                self.att_arcs[:] = self.def_arcs[:]

#==============================================================================
class _Node:
    """A private class for representing nodes within a Network object.

    Nodes possess the following internal attributes:
        id: Position in main node list.
        supply: Supply value (positive for supply, negative for demand).
        out_arcs: List of references to outgoing Arc objects.
        in_arcs: List of references to incoming Arc objects.
    """

    #--------------------------------------------------------------------------
    def __init__(self, index, supply):
        """Node object constructor initializes internal attributes.

        Requires the following positional arguments:
            index -- Node ID. We assume here that indices have already been set
                to correspond to list position, beginning with 0.
            supply -- Supply value of node (positive for supply, negative for
                demand).
        """

        self.id = index # node ID
        self.supply = supply # supply value

        # Initialize outgoing and incoming Arc object lists
        self.out_arcs = []
        self.in_arcs = []

    #--------------------------------------------------------------------------
    def __str__(self):
        """String method returns Node ID."""

        return str(self.id)

#==============================================================================
class _Arc:
    """A private class for representing arcs within a Network object.

    Arcs possess the following internal attributes:
        id: Position in the main arc list.
        head: Reference to head Node object.
        tail: Reference to tail Node object.
        bound: Upper flow bound.
        cost: Unit flow cost.

    All arcs are assumed to have a lower flow bound of 0.
    """

    #--------------------------------------------------------------------------
    def __init__(self, index, tail, head, bound, cost):
        """Arc object constructor initializes internal attributes.

        Arc objects are instantiated with their internal attributes. They
        require references to Node objects for their endpoints, and so must be
        defined after the Node objects.

        The constructor also automatically adds the Arc to the outgoing and
        incoming arc lists of its endpoints.

        Requires the following positional arguments:
            index -- Arc ID. We assume here that indices have already been set
                to correspond to list position, beginning with 0.
            tail -- Tail Node object reference.
            head -- Head Node object reference.
            bound -- Upper flow bound.
            cost -- Unit flow cost.
        """

        self.id = index # arc ID
        self.tail = tail # tail Node object
        self.head = head # head Node object
        self.bound = bound # upper bound
        self.cost = cost # flow cost

        # Add self as an outgoing/incoming arc of the tail/head
        tail.out_arcs.append(self)
        head.in_arcs.append(self)

    #--------------------------------------------------------------------------
    def __str__(self):
        """String method returns Arc ID and endpoint Node IDs."""

        return str(self.id)+" ("+str(self.tail.id)+", "+str(self.head.id)+")"
