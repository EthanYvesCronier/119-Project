import pygame
import pygame.freetype
import os
import math
import numpy as np
from vector import Vector
from DXFextractor import *


def draw_truss_body(lines, forces):
    """ draws the lines to the screen after transforming the coordinates to match screen

    :param lines: list of Member objects
    :return: None
    """
    color = (100, 100, 100)
    for member, force in zip(lines, forces):
        num_parallel = calculate_parallel(force)

        if num_parallel == 1 or num_parallel == 3:
            pygame.draw.line(screen, color, transform(member.start), transform(member.end), 1)
        if num_parallel == 2 or num_parallel == 3:
            offset = 0.03*(member.end - member.start).rotate(90).normalize()
            pygame.draw.line(screen, color, transform(member.start + offset), transform(member.end + offset), 1)
            pygame.draw.line(screen, color, transform(member.start - offset), transform(member.end - offset), 1)

        pygame.draw.circle(screen, (0, 0, 0), round(transform(member.start)), 5)
        pygame.draw.circle(screen, (0, 0, 0), round(transform(member.end)), 5)


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
    scale = 100
    x_offset = 100
    y_offset = 400

    return scale*vector.matrix_mult([[1, 0], [0, -1]]) + Vector(x_offset, y_offset)


def calculate_cost(lines, forces):
    gusset = 5
    member = 15
    cost = gusset * len(get_nodes_from_lines(lines))
    for line, force in zip(lines, forces):
        cost += member * (line.end - line.start).norm() * calculate_parallel(force)
    return cost


def is_valid(lines, forces):
    floor_nodes = get_floor_nodes(get_nodes_from_lines(lines))
    for i in range(len(floor_nodes) - 1):
        if (Vector(*floor_nodes[0]) - Vector(*floor_nodes[1])).norm() > 3.5:
            return "Floor beams too long"

    for line in lines:
        if (line.end - line.start).norm() < 1:
            return "Members too short"

    for force in forces:
        if force < min_force * calculate_parallel(force) or force > max_force * calculate_parallel(force):
            return "Force exceeded"

    return "Design Valid"


pygame.init()
pygame.freetype.init()
font = pygame.freetype.SysFont("", 11)
screen = pygame.display.set_mode((1500, 800))
pygame.display.set_caption("Why are you running?")
clock = pygame.time.Clock()

np.set_printoptions(linewidth=200)
file_name = 'how_15_8.DXF'
lines = extract_from_file(file_name)
moddate = os.stat(file_name)[8]
forces = np.round(solve_truss(lines), decimals=4)
member_forces = forces[:-3]
Ax, Ay, By = forces[-3:]
min_force = -9  # tension
max_force = 6  # compression



running = True
while running:
    clock.tick(30)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    try:
        if os.stat(file_name)[8] != moddate:
            moddate = os.stat(file_name)[8]
            lines = extract_from_file(file_name)
            forces = np.round(solve_truss(lines), decimals=4)
            member_forces = forces[:-3]
    except FileNotFoundError:
        pass  # may have caught it between saves


    screen.fill((240, 240, 240))
    draw_truss_body(lines, member_forces)
    write_forces(lines, member_forces)

    cost, _ = font.render(f"Cost: ${round(calculate_cost(lines, forces), 2)}", (0, 0, 0))
    valid, _ = font.render(f"Validity: {is_valid(lines, member_forces)}", (0, 0, 0))
    screen.blit(cost, (10, 10))
    screen.blit(valid, (10, 30))

    pygame.display.flip()

pygame.quit()


