from vector import Vector
import numpy as np
import math
import random
from DXFextractor import *

# copied functions
def calculate_parallel(force):
    if force < 0:  # tension
        return max(1, min(math.ceil(force / min_force), 3))  # number of members stacked x2 or x3
    return max(1, min(math.ceil(force / max_force), 3))


def calculate_cost(lines, forces):
    gusset = 5
    member = 15
    cost = gusset * len(get_nodes_from_lines(lines))
    for line, force in zip(lines, forces):
        cost += member * (line.end - line.start).norm() * calculate_parallel(force)
    return cost


def is_valid(lines, forces, A, B):
    floor_nodes = get_floor_nodes(get_nodes_from_lines(lines))
    if B-A != Vector(12, 0):
        return False  # "Supports A & B invalid"

    for i in range(len(floor_nodes) - 1):
        if (Vector(*floor_nodes[i]) - Vector(*floor_nodes[i+1])).norm() > 3.5:
            return False  # "Floor beams too long"

    for line in lines:
        if (line.end - line.start).norm() < 1:
            return False  # "Members too short"

    for force in forces:
        if force < min_force * calculate_parallel(force) or force > max_force * calculate_parallel(force):
            return False  # "Force exceeded"

    return True  # "Design Valid"


def get_adjacency_matrix(lines, node_positons):  # this truss is bassically just a graph use DSA of graph
    """ constructs an adjacency matrix of node positions where each connection represents a line
    refer to matrix representation of a graph from MTE 140 :P

    :param lines: this is a list of member objects representing the truss
    :param node_positons: this is the list of nodes that make up the truss, corresponds to the matrix
    :return: returns the adjacency matrix for the nodes, in the order specified in node_positions
    """
    matrix = np.zeros((len(node_positons),)*2)
    node_hash = {key: value for value, key in enumerate(node_positons)}
    for line in lines:
        i = node_hash[line.start]
        j = node_hash[line.end]
        matrix[i][j] = 1
        matrix[j][i] = 1
    return matrix


def reconstruct_lines(nodes, adjacency_matrix):
    # only have to traverse the upper or lower triangle only, this is an undirected graph
    lines = []
    for row in range(adjacency_matrix.shape[0]):
        for col in range(row):
            if adjacency_matrix[row][col]:  # if an adjacency exists
                lines.append(Member((nodes[row], nodes[col])))
    return lines


def randomize_positions(node_positions, A, B, selection_rate, radius):
    for i, node in enumerate(node_positions):
        if node == A or node == B:  # dont change these bad boys
            continue
        if node == Vector(6.0, 0):  # midpoint symmetry
            continue

        if random.random() < selection_rate:  # if true, perform the randomization
            rand_y = 0
            rand_x = 0

            if node[1] != 0:  # randomize the y direction
                rand_y = 2 * radius * (random.random() - 0.5)
            if node[0] != 6:
                rand_x = 2 * radius * (random.random() - 0.5)
            vec = Vector(rand_x, rand_y)
            node_positions[i] += vec
    return node_positions


file_name = 'bridge3.DXF'
min_force = -9  # tension
max_force = 6  # compression

# base
lines, (A, B) = extract_from_file(file_name)
forces = np.round(solve_truss(lines, A, B), decimals=4)

# parent stuff
cost = calculate_cost(lines, forces)
validity = is_valid(lines, forces[:-3], A, B)
print("Original:", cost, validity)

nodes = get_nodes_from_lines(lines)  # value indices are line indices NOT node indices
node_keys = list(nodes.keys())
node_keys.sort(key=lambda e: e[0])
adjacency_matrix = get_adjacency_matrix(lines, node_keys)  # adjacency matrix will not change for a particular topology
'''
here modify the nodes, then test to see if it is cheaper etc
'''
lowest_cost = cost
optimal = node_keys[:]
# optimal = [(0.0, 0.0), (2.53557, 0.0), (2.535582, 5.00096), (6.0, 0.0), (9.46543, 0.0), (9.46543, 4.999009), (12.0, 0.0)]
lowest_nodes = [Vector(*i) for i in optimal]
lowest_nodes.sort(key=lambda e: e[0])

t1 = reconstruct_lines(lowest_nodes, adjacency_matrix)
t2 = np.round(solve_truss(t1, A, B), decimals=4)
print(calculate_cost(t1, t2), is_valid(t1, t2[:-3], A, B))

for j in range(30):
    for i in range(2000):
        new_node_positions = randomize_positions(lowest_nodes[:], A, B, 0.1, 0.0005)

        lines2 = reconstruct_lines(new_node_positions, adjacency_matrix)
        forces2 = np.round(solve_truss(lines2, A, B), decimals=4)

        cost2 = calculate_cost(lines2, forces2)
        validity2 = is_valid(lines2, forces2[:-3], A, B)
        if validity2 and cost2 < lowest_cost:
            lowest_cost = cost2
            lowest_nodes = new_node_positions
    print(lowest_cost)

print()
print()
print(lowest_cost)
print(node_keys)
print(lowest_nodes)
# [(0.0, 0.0), (2.5000583399605594, 0.0), (6.0, 0.0), (2.243839200260678, 2.3806116098316443), (5.993252074993606, 2.636416645606335), (12.0, 0.0), (9.5, 0.0), (9.77253585845522, 2.375205636183138)]
# [(0.0, 0.0), (2.5000041257786654, 0.0), (6.0, 0.0), (2.243839200260678, 2.3806116098316443), (5.993033329747108, 2.636136034062877), (12.0, 0.0), (9.5, 0.0), (9.77253585845522, 2.375205636183138)]
# [(0.0, 0.0), (2.5000041257786654, 0.0), (6.0, 0.0), (2.243839200260678, 2.3806116098316443), (5.993033329747108, 2.636136034062877), (12.0, 0.0), (9.5, 0.0), (9.770614792551388, 2.3753239267825688)]
# [(0.0, 0.0), (2.5000041257786654, 0.0), (6.0, 0.0), (2.243839200260678, 2.3806116098316443), (5.993033329747108, 2.636136034062877), (12.0, 0.0), (9.5, 0.0), (9.770614792551388, 2.3753239267825688)]
# [(0.0, 0.0), (2.5000015825793733, 0.0), (6.0, 0.0), (2.243839200260678, 2.3806116098316443), (5.992086979218171, 2.6360562540072983), (12.0, 0.0), (9.5, 0.0), (9.767794294514948, 2.3764152446146776)]
# [(0.0, 0.0), (2.5000022511249576, 0.0), (6.0, 0.0), (2.2451420595697615, 2.3810837610754216), (5.988987395931139, 2.6359902635049712), (12.0, 0.0), (9.5, 0.0), (9.770109275921689, 2.3754979810549313)] 1138.8401242423797
# [(0.0, 0.0), (2.5000007095280403, 0.0), (6.0, 0.0), (2.252194002946471, 2.383645276968861), (5.992592442684665, 2.6368750818073634), (12.0, 0.0), (9.5, 0.0), (9.758625049206765, 2.379725442898572)] 1138.779322031384
# [(0.0, 0.0), (2.5000007095280403, 0.0), (6.0, 0.0), (2.2528925918009053, 2.383930871817669), (5.998183663966511, 2.6373234721819596), (12.0, 0.0), (9.5, 0.0), (9.748895779883933, 2.3832064421597003)] 1138.7422926443408

# start working towards forcing symmetry:
# [(0.0, 0.0), (2.5000007095280403, 0.0), (6.0, 0.0), (2.2682412604939737, 2.3894868597638546), (6, 2.638867852113163), (12.0, 0.0), (9.5, 0.0), (9.732035578440929, 2.389360248933249)] 1138.643585116853
# [(0.0, 0.0), (2.5000007095280403, 0.0), (6.0, 0.0), (2.269115155178546, 2.389857275176385), (6, 2.639037481497605), (12.0, 0.0), (9.499991871372792, 0.0), (9.730110879269246, 2.3901145902589613)] 1138.642332787612
# [(0.0, 0.0), (2.5000002389593505, 0.0), (6.0, 0.0), (2.2699077337999385, 2.3901110500821936), (6, 2.639037481497605), (12.0, 0.0), (9.499992588749873, 0.0), (9.730110879269246, 2.3901145902589613)] 1138.6373301620729
# [(0.0, 0.0), (2.5000002389593505, 0.0), (6.0, 0.0), (2.2699077337999385, 2.3901110500821936), (6, 2.639037481497605), (12.0, 0.0), (9.499997019393831, 0.0), (9.730210035927495, 2.390055528271489)] 1138.6363669895395
# [(0.0, 0.0), (2.5000002389593505, 0.0), (6.0, 0.0), (2.269782434192665, 2.3900309672618163), (6, 2.6390149238204677), (12.0, 0.0), (9.500000356751286, 0.0), (9.730210035927495, 2.390055528271489)] 1138.6344459232346
# [(0.0, 0.0), (2.5000002389593505, 0.0), (6.0, 0.0), (2.26981609714102, 2.3900390108175706), (6, 2.639010046475992), (12.0, 0.0), (9.499998392323253, 0.0), (9.730490991063117, 2.389933663767139)] 1138.6338993801478

# taking a break now
# \/ stored in 'cheapest.dxf'
# [(0.0, 0.0), (2.500000, 0.0), (6.0, 0.0), (2.26978, 2.390031), (6, 2.639015), (12.0, 0.0), (9.500000, 0.0), (9.730210, 2.390056)] 1138.64

'''
this file is even more dirty.
another improvement could be to automatically save the cheapest design to file
'''
# [(0.0, 0.0), (2.5331268275349337, 0.0), (2.533122890329575, 4.9962244442042465), (6.0, 0.0), (9.461643766229226, 0.0), (9.461640549323715, 5.003764288480279), (12.0, 0.0)] 1137.787311947202
# [(0.0, 0.0), (2.5333978982135, 0.0), (2.533398639469747, 4.996983795398753), (6.0, 0.0), (9.46225378582953, 0.0), (9.462251241774709, 5.0029703157958), (12.0, 0.0)] 1137.7882558033878
# [(0.0, 0.0), (2.5334, 0.0), (2.5334, 4.997), (6.0, 0.0), (9.4666, 0.0), (9.4666, 4.997), (12.0, 0.0)] 1241.42
# [(0.0, 0.0), (2.535578867554359, 0.0), (2.5355828034598775, 5.000965708809768), (6.0, 0.0), (9.465439909794387, 0.0), (9.465434437926309, 4.9990093762109655), (12.0, 0.0)] 1137.7996083638873
# [(0.0, 0.0), (2.53626300245894, 0.0), (2.5362803558770404, 5.001012125997316), (6.0, 0.0), (9.464980213316887, 0.0), (9.464979367684418, 4.999006981878498), (12.0, 0.0)] 1137.7911569518865


'''
961.DXF
'''