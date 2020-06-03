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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    fringe = util.Stack()
    Graph = GraphSearch(problem,fringe)
    return Graph.depthFirshSearchGraph()
    #return Graph.search()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    fringe = util.Queue()
    graph = GraphSearch(problem,fringe)
    #return graph.search()
    return graph.breathFirstSearchGraph()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    fringe = util.PriorityQueue()
    graph = GraphSearch(problem,fringe,None)
    return graph.uniformCostSearchGraph()
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    fringe = util.PriorityQueue()
    graph = GraphSearch(problem,fringe,heuristic)
    return graph.uniformCostSearchGraph()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch



class Node:
    def __init__(self,parent,state,action,step_cost,path_cost):
        self.parent = parent
        self.state = state
        self.action = action
        self.step_cost = step_cost
        self.path_cost = path_cost
    
    def expand(self,problem):# return list of node
        successors = problem.getSuccessors(self.state)
        list_expand = []
        for successor in successors:
            child_state,action,step_cost = successor
            child_node = Node(self,child_state,action,step_cost,self.path_cost+step_cost)
            list_expand.append(child_node)
        return list_expand
class GraphSearch:
    def __init__(self,problem,fringe,heuristic = nullHeuristic):
        self.problem = problem
        self.fringe = fringe
        self.heuristic = heuristic
        self.frontier = util.Counter()
        self.explored = util.Counter()
    
    
    def addToFringe(self,node):
        if isinstance(self.fringe,util.Stack):
            self.fringe.push(node)
        elif isinstance(self.fringe,util.Queue):
            self.fringe.push(node)
        else:
            if self.heuristic == nullHeuristic:
                self.fringe.push(node,node.path_cost)
            else:
                g = node.path_cost
                h=0
                if not self.heuristic is None:
                    h = self.heuristic(node.state,self.problem)
                f = h + g
                self.fringe.push(node,f)

    def extractPath(self,node):# trả về 1 list action các hành động từ start đến goal
        stack = util.Stack()
        while node is not None:
            stack.push(node.action)
            node = node.parent
        path = []
        while not stack.isEmpty():
            action = stack.pop()
            if action is not None:
                path.append(action)
        return path 



    def depthFirshSearchGraph(self):
        initial_state = self.problem.getStartState()
        initial_node = Node(None,initial_state,None,0,0)
        self.addToFringe(initial_node)
        self.frontier[hash(initial_state)] = initial_node
        while not self.fringe.isEmpty():
            current_node = self.fringe.pop()
            self.explored[hash(current_node.state)] = current_node
            self.frontier.pop(hash(current_node.state),None)
            if self.problem.isGoalState(current_node.state):
                return self.extractPath(current_node)
            for child_node in current_node.expand(self.problem):
                key = hash(child_node.state)
                if self.explored[key] == 0:
                    if self.frontier[key] == 0:
                        self.addToFringe(child_node)
                        self.frontier[key] = child_node
                    else:
                        self.addToFringe(child_node)


    def breathFirstSearchGraph(self):
        initial_state = self.problem.getStartState()
        initial_node = Node(None,initial_state,None,0,0)
        self.addToFringe(initial_node)
        self.frontier[hash(initial_state)] = initial_node
        while not self.fringe.isEmpty():
            current_node = self.fringe.pop()
            self.explored[hash(current_node.state)] = current_node
            self.frontier.pop(hash(current_node.state),None)
            if self.problem.isGoalState(current_node.state):
                return self.extractPath(current_node)
            for child_node in current_node.expand(self.problem):
                key = hash(child_node.state)
                if self.explored[key] == 0 and self.frontier[key] == 0:
                    self.addToFringe(child_node)
                    self.frontier[key] = child_node
    
    def uniformCostSearchGraph(self):
        initial_state = self.problem.getStartState()
        initial_node = Node(None,initial_state,None,0,0)
        self.addToFringe(initial_node)
        self.frontier[hash(initial_state)] = initial_node
        while not self.fringe.isEmpty():
            current_node = self.fringe.pop()
            self.explored[hash(current_node.state)] = current_node
            self.frontier.pop(hash(current_node.state),None)
            if self.problem.isGoalState(current_node.state):
                return self.extractPath(current_node)
            for child_node in current_node.expand(self.problem):
                key = hash(child_node.state)
                if self.explored[key] == 0:
                    if self.frontier[key] == 0:
                        self.addToFringe(child_node)
                        self.frontier[key] = child_node
                    else:
                        old_node = self.frontier[key]
                        if child_node.path_cost < old_node.path_cost:
                            self.fringe.update(child_node,child_node.path_cost)
                            self.frontier.pop(hash(old_node.state),None)
                            self.frontier[hash(child_node.state)] = child_node

