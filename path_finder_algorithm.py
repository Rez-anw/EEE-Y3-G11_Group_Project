from logging import currentframe
import pygame
import sys
import heapq

# Initialise pygame
pygame.init()

# Screen dimession
width, height = 800, 800

# Colours
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)

# Create the game window
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Labyrinth Maze Game")

# Game clock
clock = pygame.time.Clock()

# Maze layout: 1 = wall, 0 = Path
maze1 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

maze2 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]


# Cell and Target setting
cell_size = 40
target_x, target_y = 13 * cell_size, 17 * cell_size
maze = maze2


# Ball setting
ball_x, ball_y = cell_size + 10, cell_size + 10
ball_size = cell_size - 20
speed = 3
current_direction = None


# Draw the maze
def draw_maze():
    for row_index, row in enumerate(maze):
        for col_index, cell in enumerate(row):
            x = col_index * cell_size
            y = row_index * cell_size

            if cell == 1:
                pygame.draw.rect(screen, black, (x, y, cell_size, cell_size))
            else:
                pygame.draw.rect(screen, white, (x, y, cell_size, cell_size))



def draw_ball():
    pygame.draw.ellipse(screen, red, (ball_x, ball_y, ball_size, ball_size))

def draw_target():
    pygame.draw.rect(screen, green, (target_x, target_y, cell_size, cell_size))

# Movement control
def can_move(x, y):
    # Define the ball's bounding box based on its size
    ball_rect = pygame.Rect(x, y, ball_size, ball_size)

    # Check if any part of the ball overlaps a wall
    for row_index, row in enumerate(maze):
        for col_index, cell in enumerate(row):
            if cell == 1:  # Wall
                wall_rect = pygame.Rect(
                    col_index * cell_size,
                    row_index * cell_size,
                    cell_size,
                    cell_size,
                )
                if ball_rect.colliderect(wall_rect):
                    return False  # Ball hits a wall
    return True  # Ball is not colliding with any walls



# Check if ball is at the target
def is_ball_in_target():
    # Ball's bounding box
    ball_rect = pygame.Rect(ball_x, ball_y, ball_size, ball_size)

    # Target's bounding box
    target_rect = pygame.Rect(target_x, target_y, cell_size, cell_size)

    # check for collision
    return ball_rect.colliderect(target_rect)

# A* Algorithm to find the shortest path from start to target
def a_star(start, goal, maze):
    rows, cols = len(maze), len(maze[0])

    # Directions for movement: up, down, left, right
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    # Priority queue for A*
    open_set = []
    heapq.heappush(open_set, (0, start))

    # Cost from start to each cell
    g_score = {start: 0}

    # Parent tracking for reconstructing path
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        # If we've reached the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        # Explore neighbors
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            # Check if neighbor is within bounds and not a wall
            if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows and maze[neighbor[1]][neighbor[0]] == 0:
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Update scores
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + abs(goal[0] - neighbor[0]) + abs(goal[1] - neighbor[1])
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current

    return []  # Return an empty path if no path exists


# Convert ball and target positions to grid coordinates
start = (ball_x // cell_size, ball_y // cell_size)
goal = (target_x // cell_size, target_y // cell_size)

# Calculate the shortest path using A*
path = a_star(start, goal, maze)
current_step = 0




# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    #     elif event.type == pygame.KEYDOWN:
    #         # Update direction
    #         directions = {pygame.K_UP: "up", pygame.K_DOWN: "down", pygame.K_LEFT: "left", pygame.K_RIGHT: "right"}
    #         if event.key in directions:
    #             current_direction = directions[event.key]
    #
    #
    # # Handle continuous movement
    # if current_direction == "up" and can_move(ball_x, ball_y - speed):
    #         ball_y -= speed
    # if current_direction == "down" and can_move(ball_x, ball_y + speed):
    #         ball_y += speed
    # if current_direction == "left" and can_move(ball_x - speed, ball_y):
    #         ball_x -= speed
    # if current_direction == "right" and can_move(ball_x + speed, ball_y):
    #         ball_x += speed

    # Automatically move the ball along the path
    if current_step < len(path):
        next_cell = path[current_step]
        ball_x = next_cell[0] * cell_size + (cell_size - ball_size) // 2
        ball_y = next_cell[1] * cell_size + (cell_size - ball_size) // 2
        current_step += 1

    # Drawing
    screen.fill(white)
    draw_maze()
    draw_ball()
    draw_target()


    # Win condition
    if is_ball_in_target():
        print("You won!")
        current_direction = None
        # running = False

    pygame.display.flip()
    clock.tick(5)

pygame.quit()
sys.exit()

