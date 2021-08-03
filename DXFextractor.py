import ezdxf
import numpy as np
from vector import Vector
# https://www.ae.msstate.edu/vlsm/truss/statically_det_indet_trusses/statically_det_indet_trusses.htm

# https://pypi.org/project/ezdxf/


class Member:
    def __init__(self, line):
        if type(line) == ezdxf.entities.line.Line:  # dxf line
            # dxf_line is a different type of object with x, y, z
            self.start = Vector(*round_list(list(line.dxf.start)[:2]))
            self.end = Vector(*round_list(list(line.dxf.end)[:2]))
        else:  # two tuples or whatever
            self.start = Vector(*round_list(line[0][:2]))  # kinda jank
            self.end = Vector(*round_list(line[1][:2]))

    def __str__(self):
        return f"{self.start}  ->  {self.end}"


def round_list(input, degree=6):
    output = []
    for i in range(len(input)):
        output.append(round(input[i], degree))
    return output


def extract_from_file(file_name):
    """ Gets the line data from the dxf file
    converts dxf lines to Members

    :param file_name: file name of the dxf
    :return: list of Members, list of two Vectors
    """
    doc = ezdxf.readfile(file_name)
    model_space = doc.modelspace()
    raw_lines = list(model_space.query('LINE[linetype=="Continuous"]'))
    roots = [Vector(*round_list(list(node.dxf.location)[:2])) for node in model_space.query('POINT')]

    return [Member(l) for l in raw_lines], roots  # converts to a member object


def get_nodes_from_lines(lines):
    """ Gets the nodes that connect the lines
    does this by collecting all of the endpoints from a line in a dictionary
    key is the node position, value is an array of indicies of the members that meet at that point

    :param lines: list of Members
    :return: dictionary of nodes key: Vector, value: list indices of lines relating to lines
    """
    nodes = {}
    for i, line in enumerate(lines):  # build the node dictionary
        if line.start in nodes:
            nodes[line.start].append(i)  # vector class contains a dunder hash function
        else:
            nodes[line.start] = [i]

        if line.end in nodes:
            nodes[line.end].append(i)
        else:
            nodes[line.end] = [i]

    return nodes


def get_floor_nodes(nodes):
    floor_nodes = []
    # get the y=0 nodes, in the form of dictionary indices
    for i, key in enumerate(nodes.keys()):
        if key[1] == 0:
            floor_nodes.append(key)
    floor_nodes.sort(key=lambda e: e[0])
    return floor_nodes


def solve_truss(lines, A, B):
    train_dist = 2.5  # kN/m, half of 5 since one side of two
    # get a list of all the nodes, this will be the basis for filling the coefficient matrix
    # structure: key is the node position, value is an array of indicies of the members that meet at that point
    nodes = get_nodes_from_lines(lines)
    #print(nodes)

    if 2*len(nodes) != len(lines) + 3:
        print("nodes:", len(nodes))
        print("lines:", len(lines))
        print('bad system:')
        # system is indeterminate
        return None

    # 2 times for x and y, + 3 for the 3 reaction forces
    coefficient_matrix = np.zeros((2*len(nodes), len(lines) + 3))
    constant_matrix = np.zeros((2*len(nodes)))

    '''populate the coefficient matrix'''
    # assume all members are in COMPRESSION (tension is -ve), force going into node
    # remember that members in compression push at the node (members in tension pull a node)
    # therefor assume all forces are pushing into the node
    # coefficient coordinates +ve x ->,     +ve y ^
    for member, key in enumerate(nodes.keys()):  # members equations go down the matrix
        current = Vector(*key)

        # add reaction coefficients to the two places that are anchors
        if current == A:
            coefficient_matrix[2*member][len(lines)] = 1  # assume Ax going to the left
            coefficient_matrix[2*member+1][len(lines)+1] = 1  # assume Ay going up
        if current == B:
            coefficient_matrix[2*member+1][len(lines)+2] = 1  # assume By going up

        for index in nodes[key]:  # indices go across the matrix
            if lines[index].start == current:
                other = lines[index].end
            else:
                other = lines[index].start

            delta = (current - other).normalize()  # sign might be backwards?

            coefficient_matrix[2*member][index] = delta[0]  # x
            coefficient_matrix[2*member+1][index] = delta[1]  # y

    '''get reaction forces'''
    # remember constant_matrix goes [-m1x, -m1y, -m2x, -m2y, ... -mnx, -mny]
    # where the Ms are external forces acting on the system
    # NOTE THE NEGATIVE SIGNS
    reaction_positions = get_floor_nodes(nodes)

    reactions = np.zeros((len(reaction_positions),))
    for i in range(len(reaction_positions) - 1):
        force = train_dist * (reaction_positions[i+1] - reaction_positions[i]).norm() / 2
        reactions[i] += force
        reactions[i+1] += force

    '''final matrices construction'''
    # plug the reaction force into the constant matrix
    for i, key in enumerate(nodes.keys()):
        if key in reaction_positions:
            constant_matrix[2*i + 1] = -reactions[reaction_positions.index(key)]  # negative due to formula

    '''solve'''
    # solve system - F1, F2, F3, F4, ..., Ax, Ay, By
    return np.linalg.solve(coefficient_matrix, constant_matrix)  # doesn't work for non simple trusses (non square)



'''
Use a matrix to solve forces

https://www.youtube.com/watch?v=ukPw8xh31n8
method of joints using matrix methods


'''