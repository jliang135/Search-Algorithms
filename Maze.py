
import math
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# from PIL import Image

# takes maze and makes graph

# node structure
class Node:

    def __init__ (self, cost, row, col):
        self.cost = cost
        self.row = row
        self.col = col
        self.start = False
        self.goal = False
        self.visited = False
        self.parent = None
        self.next = []
        self.costs = []
        self.path = False
        self.level = 0
        self.f = 0

    # getters and setters sorry there are so many LOL
    def visit(self):
        self.visited = True

    def isVisited(self):
        return self.visited

    def setParent(self, node):
        self.parent = node
    
    def getParent(self):
        return self.parent

    def getNeighbors(self):
        return self.next

    def addNeighbor(self, node):
        self.next.append(node)

    def addCost(self, cost):
        self.costs.append(cost)

    def getCosts(self):
        return self.costs

    def setCost(self, cost): # i is index
        self.cost = cost

    def getCost(self):
        return self.cost

    def setStart(self):
        self.start = True

    def isStart(self):
        return self.start

    def setGoal(self):
        self.goal = True

    def isGoal(self):
        return self.goal

    #def getLenNext(self):
    #    return len(self.next)

# graph data structure (adjacency list)
class Maze:

    def __init__ (self, maze_lines, start, goal, n):
        self.nodes = []

        # create empty maze
        maze = []
        for i in range(0, n):
            maze.append([None]*n)

        # create all vertices
        for line in maze_lines:
            point = line.split() # reads each line in maze_xxx.txt
            if int(point[2]) == 0:
                newNode = Node(0, int(point[0]), int(point[1]))
                maze[int(point[0])][int(point[1])] = newNode
                self.nodes.append(newNode)

        # set start and goal
        # check if empty
        if maze[int(start[0])][int(start[1])] != None:
            maze[int(start[0])][int(start[1])].setStart()
        else:
            startNode = Node(0, int(start[0]), int(start[1]))
            startNode.setStart()
            maze[int(start[0])][int(start[1])] = startNode

        if maze[int(goal[0])][int(goal[1])] != None:
            maze[int(goal[0])][int(goal[1])].setGoal()
        else:
            goalNode = Node(0, int(goal[0]), int(goal[1]))
            goalNode.setGoal()
            maze[int(goal[0])][int(goal[1])] = goalNode

        # set start here to add it into the frontier (for BFS)
        self.start = maze[int(start[0])][int(start[1])]
        self.goal = maze[int(goal[0])][int(goal[1])]

        # traverse the filled maze and set edges (using adjacency list)
        for i in range(0, n):
            for j in range(0, n):
                # if there is a vertex here
                thisNode = maze[i][j]
                if thisNode != None:
                    # check up, down, left, right, but don't go out of bounds
                    if i > 0: # can check up
                        if maze[i-1][j] != None: 
                            thisNode.addNeighbor(maze[i-1][j]) # if node here, add to thisNode.next 
                            thisNode.addCost(2)
                    if j > 0: # can check left
                        if maze[i][j-1] != None: 
                            thisNode.addNeighbor(maze[i][j-1])
                            thisNode.addCost(1)
                    if i < n-1: # can check down
                        if maze[i+1][j] != None: 
                            thisNode.addNeighbor(maze[i+1][j])
                            thisNode.addCost(2)
                    if j < n-1: # can check right
                        if maze[i][j+1] != None: 
                            thisNode.addNeighbor(maze[i][j+1])
                            thisNode.addCost(1)

        self.maze = maze

    # ONLY USED if we are running algorithms one after another
    def reset(self):
        n = len(self.maze)
        for i in range(0, n):
            for j in range(0, n):
                # if there is a vertex here
                thisNode = self.maze[i][j]
                if thisNode != None:
                    thisNode.start = False
                    thisNode.goal = False
                    thisNode.visited = False
                    thisNode.parent = None
                    thisNode.next = []
                    thisNode.costs = []
                    thisNode.path = False
                    thisNode.level = 0
                    thisNode.f = 0

    # breadth first search
    def bfs(self):
        frontier = []
        frontier.append(self.start)
        reached = False
        nodes_expanded = 0

        while frontier:
            next = []
            for node in frontier:
                # print("frontier node data: " + node.data)
                nodes_expanded += 1
                neighbors = node.getNeighbors()
                neighborsCosts = node.getCosts() # the way I access costs here is confusing so pls ask me questions!
                # using i here so we can get the cost
                for i in range(len(neighbors)):
                    neighbor = neighbors[i]
                    ##print("neighbor data: " + str(neighbor.cost))
                    if not neighbor.isVisited():
                        neighbor.setParent(node)
                        neighbor.setCost(neighborsCosts[i])
                        next.append(neighbor)
                        neighbor.visit()
                        # check if we have reached our goal
                        if neighbor.isGoal():
                            self.goal = neighbor
                            # exit out of search
                            i = len(neighbors)
                            next.clear()
                            frontier.clear()
                            reached = True
                            break
            frontier = next

        # find path + calculate cost
        if reached:
            curr = self.goal
            cost = 0
            while not curr.isStart():
                cost += curr.getCost()
                curr.path = True
                curr = curr.getParent()
        else:
            cost = float('inf')

        return cost, nodes_expanded

    # depth limited search
    def dls(self):
        frontier = [self.start]
        reached = False
        MAX_DEPTH = 1750
        nodes_expanded = 0

        while frontier:
            
            # always pop a node off the frontier at first
            # If we hit a dead end, no nodes will be added to the frontier, so this will
            # just backwards through the frontier
            curr_node = frontier.pop()
            

            # limits depth
            if curr_node.level <= MAX_DEPTH:

                # visit node if it hasn't been visited yet
                if not curr_node.isVisited():
                    curr_node.visit()

                # check for goal
                if curr_node.isGoal():
                    #print("FOUND GOAL")
                    reached = True
                    break

                neighbors = curr_node.getNeighbors()
                neighborsCosts = curr_node.getCosts() # the way I access costs here is confusing so pls ask me questions!
                nodes_expanded += 1
                # using i here so we can get the cost
                for i in range(len(neighbors)):
                    neighbor = neighbors[i]
                
                    if not neighbor.isVisited():
                        neighbor.setParent(curr_node)
                        neighbor.level = curr_node.level + 1
                        neighbor.setCost(neighborsCosts[i])
                        frontier.append(neighbor)
            #else:
                #print("max depth detected! going back")

        # find path + calculate cost
        if reached:
            curr = self.goal
            cost = 0
            while not curr.isStart():
                cost += curr.getCost()
                curr.path = True
                curr = curr.getParent()
        else:
            cost = float('inf')

        return cost, nodes_expanded



    # MANHATTAN DISTANCE
    def A_star_manhattan(self):
        n = len(self.maze)
        goalRow = self.goal.row
        goalCol = self.goal.col
        reached = False

        nodes_expanded = 0

        frontier = [self.start]
        curr_node = self.start

        while frontier:

            index = 0

            # get node w least f value in frontier
            curr_node = frontier[0]
            for i in range(0, len(frontier)):
                node = frontier[i]
                if node.f <= curr_node.f:
                    curr_node = node
                    index = i

            # remove from frontier and set as visited
            frontier.remove(curr_node)
            curr_node.visit()

            if curr_node.isGoal():
                reached = True
                break

            neighbors = curr_node.getNeighbors()
            neighborsCosts = curr_node.getCosts() # the way I access costs here is confusing so pls ask me questions!
            nodes_expanded += 1
            # using i here so we can get the cost
            for i in range(len(neighbors)):
                neighbor = neighbors[i]
                if not neighbor.isVisited():
                    neighbor.setParent(curr_node)
                    # CALCULATING 'G' HERE
                    neighbor.setCost(curr_node.getCost() + neighborsCosts[i])
                    # CALCULATING 'H' HERE
                    h = (goalRow - neighbor.row)*2 + goalCol - neighbor.col # MANHATTAN DISTANCE
                    # CALCULATING 'F' HERE
                    neighbor.f = neighbor.getCost() + h

                    for node in frontier:
                        if node == neighbor and neighbor.getCost() > node.getCost(): continue

                    if neighbor not in frontier:
                        frontier.append(neighbor)

            #for node in frontier:
            #    print(node.row, end=", ")
            #    print(node.col, end=" : ")
            #print()

        if reached:
            # find path
            curr = self.goal
            while not curr.isStart():
                curr.path = True
                curr = curr.getParent()

            return self.goal.cost, nodes_expanded
        else:
            return float('inf'), nodes_expanded


    def A_star_h3(self):
        n = len(self.maze)
        goalRow = self.goal.row
        goalCol = self.goal.col
        reached = False

        nodes_expanded = 0

        frontier = [self.start]
        curr_node = self.start

        while frontier:

            index = 0

            # get node w least f value in frontier
            curr_node = frontier[0]
            for i in range(0, len(frontier)):
                node = frontier[i]
                if node.f <= curr_node.f:
                    curr_node = node
                    index = i

            # remove from frontier and set as visited
            frontier.remove(curr_node)
            curr_node.visit()

            if curr_node.isGoal():
                reached = True
                break

            neighbors = curr_node.getNeighbors()
            neighborsCosts = curr_node.getCosts() # the way I access costs here is confusing so pls ask me questions!
            nodes_expanded += 1
            # using i here so we can get the cost
            for i in range(len(neighbors)):
                neighbor = neighbors[i]
                if not neighbor.isVisited():
                    neighbor.setParent(curr_node)
                    # CALCULATING 'G' HERE
                    neighbor.setCost(curr_node.getCost() + neighborsCosts[i])
                    # CALCULATING 'H' HERE
                    h0 =  math.sqrt(((goalRow - neighbor.row)*2)**2 + (goalCol - neighbor.col)**2)
                    h1 = max(goalRow - neighbor.row, goalCol - neighbor.col)
                    h = min(h0, h1) #H3
                    # CALCULATING 'F' HERE
                    neighbor.f = neighbor.getCost() + h

                    for node in frontier:
                        if node == neighbor and neighbor.getCost() > node.getCost(): continue

                    if neighbor not in frontier:
                        frontier.append(neighbor)

            #for node in frontier:
            #    print(node.row, end=", ")
            #    print(node.col, end=" : ")
            #print()

        if reached:
            # find path
            curr = self.goal
            while not curr.isStart():
                curr.path = True
                curr = curr.getParent()

            return self.goal.cost, nodes_expanded
        else:
            return float('inf'), nodes_expanded



    def A_star_own(self):
        n = len(self.maze)
        goalRow = self.goal.row
        goalCol = self.goal.col
        reached = False

        nodes_expanded = 0

        frontier = [self.start]
        curr_node = self.start

        while frontier:

            index = 0

            # get node w greatest f value in frontier
            curr_node = frontier[0]
            for i in range(0, len(frontier)):
                node = frontier[i]
                if node.f <= curr_node.f:
                    curr_node = node
                    index = i

            # remove from frontier and set as visited
            frontier.remove(curr_node)
            curr_node.visit()

            if curr_node.isGoal():
                reached = True
                break

            neighbors = curr_node.getNeighbors()
            neighborsCosts = curr_node.getCosts() # the way I access costs here is confusing so pls ask me questions!
            nodes_expanded += 1
            # using i here so we can get the cost
            for i in range(len(neighbors)):
                neighbor = neighbors[i]
                if not neighbor.isVisited():
                    neighbor.setParent(curr_node)
                    # CALCULATING 'G' HERE
                    neighbor.setCost(curr_node.getCost() + neighborsCosts[i])
                    # CALCULATING 'H' HERE
                    h0 =  math.sqrt(((goalRow - neighbor.row)*2)**2 + (goalCol - neighbor.col)**2)
                    h1 = (goalRow - neighbor.row)*2 + goalCol - neighbor.col 
                    h = (h0+h1)/2 # TAKING THE AVERAGE OF MANHATTAN AND EUCLIDEAN
                    # CALCULATING 'F' HERE
                    neighbor.f = neighbor.getCost() + h

                    for node in frontier:
                        if node == neighbor and neighbor.getCost() > node.getCost(): continue

                    if neighbor not in frontier:
                        frontier.append(neighbor)

            #for node in frontier:
            #    print(node.row, end=", ")
            #    print(node.col, end=" : ")
            #print()

        if reached:
            # find path
            curr = self.goal
            while not curr.isStart():
                curr.path = True
                curr = curr.getParent()

            return self.goal.cost, nodes_expanded
        else:
            return float('inf'), nodes_expanded


    # displays maze with path
    # def display_path(self, maze_file, n):

    #     im = Image.open(maze_file)

    #     # Create figure and axes
    #     fig, ax = plt.subplots()

    #     rects = []

    #     # Display the image
    #     ax.imshow(im)

    #     curr = self.goal
    #     while not curr.isStart():
    #         new_rect = patches.Rectangle((curr.col*600/n, curr.row*600/n), 4, 4, linewidth=1, edgecolor='r', facecolor='red')
    #         rects.append(new_rect)
    #         curr = curr.getParent()

    #     start = patches.Rectangle((self.start.col*600/n, self.start.row*600/n), 15, 15, linewidth=1, edgecolor='r', facecolor='green')
    #     goal = patches.Rectangle((self.goal.col*600/n, self.goal.row*600/n), 15, 15, linewidth=1, edgecolor='r', facecolor='blue')
                        
    #     rects.append(start)
    #     rects.append(goal)

    #     for rect in rects:
    #         ax.add_patch(rect)

    #     plt.show()











        
