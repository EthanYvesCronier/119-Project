import pygame
import pygame.freetype
import os
import math
import tkinter
import tkinter.filedialog
import ezdxf
import numpy as np
from vector import Vector
from DXFextractor import *


def draw_truss_body(lines, forces, mouse_pos):
    """ draws the lines to the screen after transforming the coordinates to match screen

    :param lines: list of Member objects
    :return: None
    """
    color = (100, 100, 100)
    for member, force in zip(lines, forces):
        num_parallel = calculate_parallel(force)

        if num_parallel != 2:  # when equal to 0, 1, 3 (zero force member was disappearing)
            pygame.draw.line(screen, color, transform(member.start), transform(member.end), 1)
        if num_parallel == 2 or num_parallel == 3:
            offset = 0.03*(member.end - member.start).rotate(90).normalize()
            pygame.draw.line(screen, color, transform(member.start + offset), transform(member.end + offset), 1)
            pygame.draw.line(screen, color, transform(member.start - offset), transform(member.end - offset), 1)

        start_color = (100, 0, 0) if (transform(member.start) - mouse_pos).norm() < r else (0, 0, 0)
        end_color = (100, 0, 0) if (transform(member.end) - mouse_pos).norm() < r else (0, 0, 0)

        pygame.draw.circle(screen, start_color, round(transform(member.start)), 5)
        pygame.draw.circle(screen, end_color, round(transform(member.end)), 5)


def calculate_parallel(force):
    if force < 0:  # tension
        return min(math.ceil(force / min_force), 3)  # number of members stacked x2 or x3
    return min(math.ceil(force / max_force), 3)


def write_forces(lines, forces):
    for i, line in enumerate(lines):
        render_str = f"{round(forces[i], 2)} kN {'(T)' if forces[i] < 0 else '(C)'}"
        textsurface, _ = font.render(render_str, (0, 0, 0))
        midpoint = line.start + 0.5*(line.end - line.start)
        screen.blit(textsurface, round(transform((midpoint)) - Vector(40, 10)))


def transform(vector):
    return scale*vector.matrix_mult([[1, 0], [0, -1]]) + Vector(x_offset, y_offset)


def inverse_transform(vector):
    return (1/scale)*(vector - Vector(x_offset, y_offset)).matrix_mult([[1, 0], [0, -1]])


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
        return "Supports A & B invalid"

    for i in range(len(floor_nodes) - 1):
        if (Vector(*floor_nodes[i]) - Vector(*floor_nodes[i+1])).norm() > 3.5:
            return "Floor beams too long"

    for line in lines:
        if (line.end - line.start).norm() < 1:
            return "Members too short"

    for force in forces:
        if force < min_force * calculate_parallel(force) or force > max_force * calculate_parallel(force):
            return "Force exceeded"

    return "Design Valid"


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


def save_file(lines, A, B):
    top = tkinter.Tk()
    top.withdraw()
    file_name = tkinter.filedialog.asksaveasfilename(filetypes=[('DXF Files', '*.DXF')], defaultextension=[('DXF Files', '*.DXF')])
    top.destroy()
    if file_name == '':
        return False
    doc = ezdxf.new('R2010')
    msp = doc.modelspace()
    for line in lines:
        msp.add_line(tuple(line.start), tuple(line.end), dxfattribs={"linetype": "Continuous"})

    for anchor in (A, B):
        msp.add_point(tuple(anchor))

    doc.saveas(file_name)



pygame.init()
pygame.freetype.init()
font = pygame.freetype.SysFont("", 11)
screen = pygame.display.set_mode((1500, 800))
pygame.display.set_caption("Why are you running?")
clock = pygame.time.Clock()

# transform consts
r = 5  # radius to snap to node
scale = 100
x_offset = 100
y_offset = 300

np.set_printoptions(linewidth=200)
# file_name = 'cheapest.DXF'
file_name = 'bridgeEC.DXF'
lines, (A, B) = extract_from_file(file_name)
moddate = os.stat(file_name)[8]
forces = np.round(solve_truss(lines, A, B), decimals=4)
member_forces = forces[:-3]
Ax, Ay, By = forces[-3:]
min_force = -9  # tension
max_force = 6  # compression

nodes = get_nodes_from_lines(lines)  # value indices are line indices NOT node indices
node_keys = list(nodes.keys())
node_keys.sort(key=lambda e: e[0])
adjacency_matrix = get_adjacency_matrix(lines, node_keys)  # adjacency matrix will not change for a particular topology

current_node_index = None

running = True
while running:
    clock.tick(30)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            for i, node in enumerate(node_keys):
                if (transform(node) - Vector(*pygame.mouse.get_pos())).norm() < r:
                    current_node_index = i
                    break
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s and pygame.key.get_mods() & pygame.KMOD_CTRL:
                # save the current truss
                save_file(lines, A, B)

        if event.type == pygame.MOUSEBUTTONUP:
            current_node_index = None

    if pygame.mouse.get_pressed()[0] and current_node_index is not None:
        if node_keys[current_node_index][1] == 0:
            if node_keys[current_node_index][0] != 0 and node_keys[current_node_index][0] != 12:  # at equality this is the end of the road, do not touch
                # floor node can only be manipulated in x
                new = inverse_transform(Vector(*pygame.mouse.get_pos())).matrix_mult([[1, 0], [0, 0]])
                node_keys[current_node_index] = new

        elif node_keys[current_node_index] == A:
            A = inverse_transform(Vector(*pygame.mouse.get_pos())).matrix_mult([[0, 0], [0, 1]])
            A = round(A, 4)
            node_keys[current_node_index] = A
            B_index = node_keys.index(B)
            B = A + Vector(12, 0)
            node_keys[B_index] = B
        elif node_keys[current_node_index] == B:
            B = inverse_transform(Vector(*pygame.mouse.get_pos())).matrix_mult([[0, 0], [0, 1]]) + Vector(12, 0)
            B = round(B, 4)
            node_keys[current_node_index] = B
            A_index = node_keys.index(A)
            A = B + Vector(-12, 0)
            node_keys[A_index] = A
        else:
            # not a floor node, can be manipulated in x and y
            node_keys[current_node_index] = inverse_transform(Vector(*pygame.mouse.get_pos()))

        lines = reconstruct_lines(node_keys, adjacency_matrix)
        forces = np.round(solve_truss(lines, A, B), decimals=4)
        member_forces = forces[:-3]
        Ax, Ay, By = forces[-3:]

    try:
        if os.stat(file_name)[8] != moddate:
            moddate = os.stat(file_name)[8]
            lines, (A, B) = extract_from_file(file_name)
            forces = np.round(solve_truss(lines, A, B), decimals=4)
            member_forces = forces[:-3]
            Ax, Ay, By = forces[-3:]

            nodes = get_nodes_from_lines(lines)  # value indices are line indices NOT node indices
            node_keys = list(nodes.keys())
            node_keys.sort(key=lambda e: e[0])
            adjacency_matrix = get_adjacency_matrix(lines,
                                                    node_keys)  # adjacency matrix will not change for a particular topology
    except FileNotFoundError:
        pass  # may have caught it between saves

    screen.fill((240, 240, 240))
    draw_truss_body(lines, member_forces, Vector(*pygame.mouse.get_pos()))
    write_forces(lines, member_forces)

    cost, _ = font.render(f"Cost: ${round(calculate_cost(lines, forces), 2)}", (0, 0, 0))
    valid, _ = font.render(f"Validity: {is_valid(lines, member_forces, A, B)}", (0, 0, 0))
    screen.blit(cost, (10, 10))
    screen.blit(valid, (10, 30))

    pygame.display.flip()

pygame.quit()


