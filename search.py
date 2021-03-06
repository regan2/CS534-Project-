# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state
        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state
        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take
        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()
    visited = {}
    dir = "None"

    stack = recursiveDfs(problem, startState, dir, visited)

    directions = []
    while not stack.isEmpty():
        item = stack.pop()
        directions.append(item)

    return directions

def recursiveDfs(problem, state, dir, visited):
    sta = util.Stack()
    sta.__init__()

    if problem.isGoalState(state):
        str = convertDirection(dir)
        sta.push(str)
        return sta

    if state in visited:
        return sta

    for newLocation in problem.getSuccessors(state):
        nextState,Dir,Cost = newLocation
        visited[state] = True

        stack = recursiveDfs(problem,nextState, Dir, visited)

        if not stack.isEmpty():
            str = convertDirection(dir)
            stack.push(str)
            return stack

    return sta

def convertDirection(dir):
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST
    n = Directions.NORTH
    none = Directions.STOP

    return {
        'South': s,
        'East': e,
        'West': w,
        'North': n,
        'None': none
    }[dir]

class bfs_node:
   def __init__(self, state, action = None, pred=None):
       self.state = state
       self.pred = pred
       self.action = action


   def Solution(self):
       solution = [self.action]
       if(self.pred!=None and self.pred.action != None):
           solution += self.pred.Solution()
       return solution


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    currentstate = problem.getStartState()
    currentnode = bfs_node(currentstate)
    if problem.isGoalState(problem.getStartState()):
        return []
    open = [currentnode]
    closed = []
    while len(open) != 0:
        n = open.pop(0)
        if n.state in closed:
            continue
        closed.append(n.state)
        successors = map(lambda x: bfs_node(x[0], x[1], n), problem.getSuccessors(n.state))
        #print successors
        for suc in successors:
            if suc.state not in closed:
                if problem.isGoalState(suc.state):
                    path = suc.Solution()
                    path.reverse()
                    #print path
                    return path

                open.append(suc)
    return

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

class node:
    def __init__(self, state, h, g, action = None, pred = None, f = 0):
        self.state = state
        self.pred = pred
        self.action = action
        self.g = g
        self.h = h
        self.f = f

    def Solution(self):
        solution = [self.action]
        if(self.pred != None and self.pred.action != None):
            solution += self.pred.Solution()
        return solution

#def manhattanHeuristic(position, problem, info={}):
#    "The Manhattan distance heuristic for a PositionSearchProblem"
#    xy1 = position
#    xy2 = problem.goal
#    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    actions = []
    frontier = util.PriorityQueueWithFunction(lambda x: x.h + x.g)
    curr_state = node((problem.getStartState(), None, None), heuristic(problem.getStartState(), problem), 0)
    frontier.push(curr_state)
    explored = []
    while True:
        if (frontier.isEmpty()):
            return []  # failure
        curr_node = frontier.pop()
        if (problem.isGoalState(curr_node.state[0])):
            ans_list = curr_node.Solution()
            ans_list.reverse()
            return ans_list
        for suc in problem.getSuccessors(curr_node.state[0]):
            if (suc[0] in explored):
                continue
            explored.append(suc[0])
            suc_node = node(suc, heuristic(suc[0], problem), suc[2] + curr_node.g, suc[1], curr_node)
            if(update_frontier(frontier, suc_node)):
                frontier.push(suc_node)



def update_frontier(frontier, node):
    frontier_list = frontier.heap
    state = node.state[0]
    f = node.h+node.g
    for heap_obj in frontier_list:
        old_node = heap_obj[2]
        old_state = old_node.state[0]
        old_f = old_node.h + old_node.g
        if old_state == state:
            if f < old_f:
                del frontier[old_node]
                return True
            else:
                return False
    return True

def greedySearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest heuristic first."""
    actions = []
    frontier = util.PriorityQueueWithFunction(lambda x: x.h)
    curr_state = node((problem.getStartState(), None, None), heuristic(problem.getStartState(), problem), 0)
    frontier.push(curr_state)
    explored = []
    while True:
        if (frontier.isEmpty()):
            return []  # failure
        curr_node = frontier.pop()
        if (problem.isGoalState(curr_node.state[0])):
            ans_list = curr_node.Solution()
            ans_list.reverse()
            return ans_list
        for suc in problem.getSuccessors(curr_node.state[0]):
            if (suc[0] in explored):
                continue
            explored.append(suc[0])
            suc_node = node(suc, heuristic(suc[0], problem), suc[2] + curr_node.g, suc[1], curr_node)
            if(update_frontier(frontier, suc_node)):
                frontier.push(suc_node)

def weightedAStarSearch(problem, heuristic = nullHeuristic):
    w = 1.5
    actions = []
    frontier = util.PriorityQueueWithFunction(lambda x: w*x.h + x.g)
    curr_state = node((problem.getStartState(), None, None), w*heuristic(problem.getStartState(), problem), 0)
    frontier.push(curr_state)
    explored = []
    while True:
        if (frontier.isEmpty()):
            return []  # failure
        curr_node = frontier.pop()
        if (problem.isGoalState(curr_node.state[0])):
            ans_list = curr_node.Solution()
            ans_list.reverse()
            return ans_list
        for suc in problem.getSuccessors(curr_node.state[0]):
            if (suc[0] in explored):
                continue
            explored.append(suc[0])
            suc_node = node(suc, w*heuristic(suc[0], problem), suc[2] + curr_node.g, suc[1], curr_node)
            if(update_frontier(frontier, suc_node)):
                frontier.push(suc_node)

class rtanode:
    def __init__(self, state, h, g, action = None, pred = None, f = 0):
        self.state = state
        self.pred = pred
        self.action = action
        self.g = g
        self.h = h
        self.f = f

    def Solution(self):
        solution = [self.action]
        if(self.pred != None and self.pred.action != None):
            solution += self.pred.Solution()
        return solution


def rtaStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    actions = []
    curr_node = rtanode((problem.getStartState(), None, None), heuristic(problem.getStartState(), problem), 0)
    explored = []
    explored_nodes = []
    while True:
        if (problem.isGoalState(curr_node.state[0])):
            ans_list = curr_node.Solution()
            ans_list.reverse()
            return ans_list
        suc_list = []
        for suc in problem.getSuccessors(curr_node.state[0]):
            if (suc[0] in explored):
                #print(suc[0])
                suc_node = get_explored_node(explored_nodes, suc[0])
                #print(suc_node.h)
                suc_list.append(suc_node)
            else:
                explored.append(suc[0])
                suc_node = rtanode(suc, heuristic(suc[0], problem), suc[2] + curr_node.g, suc[1], curr_node, f=suc[2] + heuristic(suc[0], problem))
                explored_nodes.append(suc_node)
                suc_list.append(suc_node)
        f_min = 100000
        f_sec = 100000
        min_node = None
        for node in suc_list:
            if node.f < f_min:
                min_node = node
                f_min = node.f
        for node in suc_list:
            if node.f > f_min and node.f < f_sec:
                f_sec = node.f
        if f_sec < 100000:
            print(f_sec)
            curr_node.h = f_sec
        curr_node = min_node
        print explored


def get_explored_node(explored_nodes, state):
    for node in explored_nodes:
        if node.state[0] == state:
            return node
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch