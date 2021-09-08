# Search-Algorithms

mazes.zip contains the mazes images and their .txt versions.

mazes are 100x100

main.py get the necessary information from problem.txt, then finds the path (if possible) and displays it using matplotlib:

problem.txt format by lines:

0. Size of the maze N - integer
1. Start state xy - two integers: 0 ≤ x, y < N
2. Goal state xy - two integers: 0 ≤ x, y < N
3. Algorithm - integer from 0 to 4
4. Maze - integer representing the maze to use

Maze.py contains the graph struct and the bfs, dls, and heuristic search algorithms.
