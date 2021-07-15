import pygame
import pygame.freetype
import os
import numpy as np
from vector import Vector
from DXFextractor import *


def draw_truss_body(lines):
    """ draws the lines to the screen after transforming the coordinates to match screen

    :param lines: list of Member objects
    :return: None
    """

    for member in lines:
        pygame.draw.line(screen, (100, 100, 100), transform_coords(member.start), transform_coords(member.end), 1)
        pygame.draw.circle(screen, (0, 0, 0), round(transform_coords(member.start)), 3)
        pygame.draw.circle(screen, (0, 0, 0), round(transform_coords(member.end)), 3)

def write_forces(lines, forces):
    for i, line in enumerate(lines):
        render_str = f"{round(forces[i], 2)} kN {'(T)' if forces[i] < 0 else '(C)'}"
        textsurface, _ = font.render(render_str, (0, 0, 0))
        midpoint = line.start + 0.5*(line.end - line.start)
        screen.blit(textsurface, round(transform_coords((midpoint))-Vector(40, 10)))


def transform_coords(vector):
    scale = 100
    x_offset = 100
    y_offset = 300

    return scale*vector.matrix_mult([[1, 0], [0, -1]]) + Vector(x_offset, y_offset)

def calculate_cost(lines):
    gusset = 5
    member = 15
    cost = gusset * len(get_nodes_from_lines(lines))
    for line in lines:
        cost += member * (line.end - line.start).norm()
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
        if force < -9 or force > 6:
            return "Force exceeded"

    return "Design Valid"


pygame.init()
pygame.freetype.init()
font = pygame.freetype.SysFont("", 11)
screen = pygame.display.set_mode((1500, 600))
pygame.display.set_caption("Why are you running?")
clock = pygame.time.Clock()

np.set_printoptions(linewidth=200)
file_name = 'bridge1.DXF'
lines = extract_from_file(file_name)
moddate = os.stat(file_name)[8]
forces = np.round(solve_truss(lines), decimals=4)
member_forces = forces[:-3]
Ax, Ay, By = forces[-3:]



running = True
while running:
    clock.tick(30)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if os.stat(file_name)[8] != moddate:
        moddate = os.stat(file_name)[8]
        lines = extract_from_file(file_name)
        forces = np.round(solve_truss(lines), decimals=4)
        member_forces = forces[:-3]


    screen.fill((240, 240, 240))
    draw_truss_body(lines)
    write_forces(lines, member_forces)

    cost, _ = font.render(f"Cost: ${round(calculate_cost(lines), 2)}", (0, 0, 0))
    valid, _ = font.render(f"Validity: {is_valid(lines, member_forces)}", (0, 0, 0))
    screen.blit(cost, (10, 10))
    screen.blit(valid, (10, 30))

    pygame.display.flip()

pygame.quit()


