import random
import numpy as np
import matplotlib.pyplot as plt

def visit(x, y, hasVisited, maze, EMPTY, NORTH, SOUTH, EAST, WEST, WIDTH=21, HEIGHT=21):
    """"Carve out" empty spaces in the maze at x, y and then
    recursively move to neighboring unvisited spaces. This
    function backtracks when the mark has reached a dead end."""
    maze[(x, y)] = EMPTY # "Carve out" the space at x, y.
    # printMaze(maze, x, y) # Display the maze as we generate it.
    # print('\n\n')

    while True:
        # Check which neighboring spaces adjacent to
        # the mark have not been visited already:
        unvisitedNeighbors = []
        if y > 1 and (x, y - 2) not in hasVisited:
            unvisitedNeighbors.append(NORTH)

        if y < HEIGHT - 2 and (x, y + 2) not in hasVisited:
            unvisitedNeighbors.append(SOUTH)

        if x > 1 and (x - 2, y) not in hasVisited:
            unvisitedNeighbors.append(WEST)

        if x < WIDTH - 2 and (x + 2, y) not in hasVisited:
            unvisitedNeighbors.append(EAST)

        if len(unvisitedNeighbors) == 0:
            # BASE CASE
            # All neighboring spaces have been visited, so this is a
            # dead end. Backtrack to an earlier space:
            return
        else:
            # RECURSIVE CASE
            # Randomly pick an unvisited neighbor to visit:
            nextIntersection = random.choice(unvisitedNeighbors)

            # Move the mark to an unvisited neighboring space:

            if nextIntersection == NORTH:
                nextX = x
                nextY = y - 2
                maze[(x, y - 1)] = EMPTY # Connecting hallway.
            elif nextIntersection == SOUTH:
                nextX = x
                nextY = y + 2
                maze[(x, y + 1)] = EMPTY # Connecting hallway.
            elif nextIntersection == WEST:
                nextX = x - 2
                nextY = y
                maze[(x - 1, y)] = EMPTY # Connecting hallway.
            elif nextIntersection == EAST:
                nextX = x + 2
                nextY = y
                maze[(x + 1, y)] = EMPTY # Connecting hallway.

            hasVisited.append((nextX, nextY)) # Mark as visited.
            visit(nextX, nextY, hasVisited, maze, EMPTY, NORTH, SOUTH, EAST, WEST, WIDTH, HEIGHT) # Recursively visit this space.

def generateMaze(width=21, height=21, seed=40, start_point=(0, 0)):
    """Generates a maze with the given width and height. And also create a 2D map of the maze."""
    NORTH, SOUTH, EAST, WEST = 'n', 's', 'e', 'w'

    assert width % 2 == 1 and width >= 3
    assert height % 2 == 1 and height >= 3

    random.seed(seed)

    WALL = 0
    EMPTY = 1
    
    maze = {}

    for x in range(width):
        for y in range(height):
            maze[(x, y)] = WALL

    hasVisited = [start_point]
    visit(start_point[0], start_point[1], hasVisited, maze, EMPTY, NORTH, SOUTH, EAST, WEST, width, height)

    mapped_maze = np.zeros((width, height))

    for x in range(width):
        for y in range(height):
            mapped_maze[x, y] = maze[(x, y)]
    
    return np.array(mapped_maze, dtype=np.uint8).reshape((width, height))

# print(generateMaze())
