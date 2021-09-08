from Maze import Maze

# opens + reads problem file
f1 = open('problem.txt', 'r')

problem = f1.readlines()

# gets size of maze
n = int(problem[0])

# turns the start and goal points into lists e.g. ['16', '20']
start = problem[1].split()
goal = problem[2].split()

# gets algorithm (0-4)
algo = int(problem[3])

# only use this for testing, make sure to edit problem.txt too
#f2 = open('example_3x3.txt', 'r')

# opens maze file
maze_file = 'mazes\maze_' + problem[4][:3] + '.txt'
maze_image = 'mazes\maze_' + problem[4][:3] + '.png'
f2 = open(maze_file, 'r')
maze_lines = f2.readlines()


# constructs maze as graph
maze = Maze(maze_lines, start, goal, n)



print()

if algo == 0:
    cost, exp_nodes = maze.bfs()
    print("--BREADTH FIRST SEARCH on '" + maze_file + "'--")
    print("Cost of Path: " + str(cost))
    print("# of Expanded Nodes: " + str(exp_nodes))
  
elif algo == 1:
    cost, exp_nodes = maze.dls()
    print("--DEPTH LIMITED SEARCH on '" + maze_file + "'--")
    print("Cost of Path: " + str(cost))
    print("# of Expanded Nodes: " + str(exp_nodes))

elif algo == 2:
    cost, exp_nodes = maze.A_star_manhattan()
    print("--A* USING MANHATTAN DISTANCE on '" + maze_file + "'--")
    print("Cost of Path: " + str(cost))
    print("# of Expanded Nodes: " + str(exp_nodes))

elif algo == 3:
    cost, exp_nodes = maze.A_star_h3()
    print("--A* USING H3 on '" + maze_file + "'--")
    print("Cost of Path: " + str(cost))
    print("# of Expanded Nodes: " + str(exp_nodes))

elif algo == 4: # h = average of manhattan and euclidean distances
    cost, exp_nodes = maze.A_star_own()
    print("--A* USING CHOICE HEURISTIC on '" + maze_file + "'--")
    print("Cost of Path: " + str(cost))
    print("# of Expanded Nodes: " + str(exp_nodes))



# displays path! ONLY IF U HAVE
#maze.display_path(maze_image, n)





# for i in range(0, 10):
#     f2 = open('mazes\maze_00' + str(i) + '.txt', 'r')
#     maze_lines = f2.readlines()
#     maze = Maze(maze_lines, start, goal, n)
#     print(maze.dls())
#     maze.reset()

# for i in range(10, 51):
#     f2 = open('mazes\maze_0' + str(i) + '.txt', 'r')
#     maze_lines = f2.readlines()
#     maze = Maze(maze_lines, start, goal, n)
#     print(maze.dls())
#     maze.reset()

# resets maze so other algorithms can be used on it
#maze.reset()

# depth-limited search, returns cost
#print(maze.dls())

# heuristic search - manhattan distance
#print(maze.A_star_manhattan())

