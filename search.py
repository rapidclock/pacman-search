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


def master_search_hyper(fringe, problem):
    # %init
    parent = {}
    visited = []
    path = []
    start_state = problem.getStartState()

    state = ((start_state, 'Start', 0), 'Root')
    fringe.push(state)

    while not fringe.isEmpty():
        # %pop
        u = fringe.pop()
        current_state = u[0][0]
        action = u[0][1]
        step_cost = u[0][2]
        parent_state = u[1]

        # uses the visited array, to ignore the already visited nodes(states)
        if current_state in visited:
            continue

        # The parent dictionary has the below configuration.
        # KEY   -> current_state
        # VALUE -> (parent_state, action_from_parent_to_current)
        parent[current_state] = (parent_state, action)

        # Visited Array holds the states expanded
        visited.append(current_state)

        # %Goal Test
        if problem.isGoalState(current_state):
            # uses the parent dictionary to re-assemble the path from the goal,
            # current_state to the source.
            path = reconstruct_path(parent, current_state)
            break

        for neighbour in problem.getSuccessors(current_state):
            # %parseNeighbour
            neigh_state = neighbour[0]
            neigh_action = neighbour[1]
            # cumulative_cost = cost_from_parent + step_cost_to_neighbour
            neigh_cost = neighbour[2] + step_cost
            # creates the neighbour_state and pushes onto the fringe.
            new_state = ((neigh_state, neigh_action, neigh_cost), current_state)
            fringe.push(new_state)
    return path

def reconstruct_path(parent, goal_state):
    list_of_moves = []
    # Till the start(root) state is not reached, keep appending the actions.
    # parent[state] => (parent_state, action_from_parent_to_child)
    while parent[goal_state][0] != 'Root':
        parent_state = parent[goal_state][0]
        action = parent[goal_state][1]
        list_of_moves.append(action)
        goal_state = parent_state
    # we get the actions from goal to root, now reverse for directions from
    # root to goal.
    list_of_moves.reverse()
    return list_of_moves


def depthFirstSearch(problem):
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
    from util import Stack
    stk = Stack()
    return master_search_hyper(stk, problem)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    queue = Queue()
    return master_search_hyper(queue, problem)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    # the step cost is the actual cumulative cost from parent to state,
    # as calculated by the master_search algorithm.
    cumulative_cost = lambda x : x[0][2]
    pqueue = PriorityQueueWithFunction(cumulative_cost)
    return master_search_hyper(pqueue, problem)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    # total_cost = cumulative cost from parent to current node +
    #              heuristic cost from current node to goal.
    # in other words, f(n) = g(n) + h(n)
    total_cost = lambda x : x[0][2] + heuristic(x[0][0], problem)
    pqueue = PriorityQueueWithFunction(total_cost)
    return master_search_hyper(pqueue, problem)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
