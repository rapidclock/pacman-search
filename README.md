# CS5100 - AI - Project 1 - Search
> by Rahul Thomas.
----


## Problem 1 - 4 : Search

#### General Graph Search Algorithm:
The problems 1 through 4 employ the same process of searching the graph, except using a different type of fringe.
- **DFS** - Stack
- **BFS** - Queue
- **UCS** - PriorityQueueWithFunction
- __A* Search__ - PriorityQueueWithFunction

The general graph search flowchart is shown below:
![picture alt](http://i65.tinypic.com/ie4bcn.jpg)

##### Explanation:
__Note :__ Prior to the process depicted in the flow chart, the starting state is pushed onto the fringe using the function `start_state = problem.getStartState()`.
The element actually pushed onto the fringe is modified as follows:
```python
state = ((start_state, 'Start', 0), 'Root')
```

We also make use of the following Data Structures in the graph Search.
1. `parent` - Dictionary. *[__Key__ -> Current State, __Value__ -> (Parent State, Action)]*
1. `visited` - List. *[List of visited States.]*
1. `path` - List. *[List of actions taken to reach the goal from start state.]*
1. `fringe` - The data structure passed into the search procedure. *[Determines the type of search]*

__Fringe Content :__ Although the type of fringe is passed into this function, we manage what we put on the fringe. We construct a special tuple that we push/pop onto/from the fringe. Lets consider the element on the fringe as u.
```python
u = ((current_state, action, cumulative_cost), parent_state)
```

Here,

Element Slice  | Relevance
-------------  | -------------
__u[0][0]__    | *current_state*
__u[0][1]__    | *action taken to get to current_state*
__u[0][2]__    | *cumulative_cost*
__u[1]__       | *parent_state*

__Process Flow :__

* First the Fringe is checked if its empty. If the fringe is empty, it indicates no path was found and the function returns an empty list.
* The first state is popped from the fringe.
* We check if this state has already been visited.
	* If the state has already been visited, `continue` is invoked and the empty check is performed on the fringe.
* Else, the parent of this current node is set in the parent dictionary along with the action taken on the parent to reach the current node. `parent[current_state] = (parent_state, action)`
* We now mark this state as Visited by adding it into the visited List.
* Next we perform the Goal Test by invoking the function `problem.isGoalState(current_state)` on the current state.
	* If the current state is the goal, then we reconstruct the path by calling the `reconstruct_path(parent, goal_state) : List[actions]` function that we defined. Now the program terminates after returning the list of actions.
* If the goal is not reached, loop through the neighbours of the state, and push all the non-visited neighbour_states on to the fringe.
* The step costs of the neighbours pushed onto the fringe is the cumulative cost of the `current cumulative costs of parent + step_cost from parent to neighbour`
* After this the control goes back to the Empty Check on the fringe.

__Reconstruction of Path :__

Signature of the function is :
```python
reconstruct_path(parent, goal_state) : List[actions]
```
* The inputs are the parent dictionary and the goal state.
* The output is a list of actions from the start state to the goal state.
* The function creates a list by tracing the previous states along the path from the goal state to the start state.
* Finally this list is reversed to get it in the order from Start to Goal State.

__Cost for UCS and A* :__
* For UCS, the cumulative cost is given by the `u[0][2]` slice of the element on the fringe. Given a state, we deduce this using the following code snippet.
```python
cumulative_cost = lambda x : x[0][2]
```
* For A*, we use the same formula as above but we add the heuristic cost,
```python
total_cost = lambda x : x[0][2] + heuristic(x[0][0], problem)
```
---

## Problem 8 : Suboptimal Search

##### findPathToClosestDot :
* Uses BFS to find the path to the closest food. Since BFS always finds the shortest number of steps to the food, we employ BFS as our search Strategy.
##### AnyFoodSearchProblem.isGoalState :
* Since we want the shortest path to any food, we check if the give state is a food or not, by cross referencing the problem's boolean food list.
```python
self.food[x][y]
```
---
## Problem 5 : Finding Corners

### Defining the problem :
The Main task is to give our definition of the problem. We deal with the `CornersProblem` Class. We define the working of three functions:
* `getStartState()`
* `isGoalState(state)`
* `getSuccessors(state)`

### State:
The state in the corners problem is defined as a tuple containing the pacman position and the tuple of visited corners.
```python
state = (self.startingPosition, ())
```
* `self.startingPosition`  --> Starting position of Pacman
* `()` --> Empty tuple of visited corners. [initially empty]

##### getStartState() :
This method just returns the state as mentioned previously with an empty tuple of visited corners.

##### isGoalState(state) :
This method checks if the length of the visited corner tuple. The goal is achieved when we the length equals the number of corners in the problem.
```python
len(visited_corners) == len(self.corners)
```

##### getSuccessors(state) :
General Flow Chart of the logic is below:
![picture alt](http://i67.tinypic.com/2gx06dd.jpg)

* For all Possible actions for a give state(Up, Down, Left, Right), we check those positions which are not walls.
* If the resulting position is a corner, we check if it is already visited.
* If it is an unvisited goal, we add it to the list of visited goals.
* We then use the successor position and this visited goal list to construct the successor state.
```python
successor_state = (successor_location, goals_visited)
```
* We append this list to the list of successors.
* We return this list of successors.

---
## Problem 6 : Corner Heuristic
### Strategy :

* We calculate the length from pacman to the nearest _unvisited_ corner, _`l1`_.
* From there we keep adding the distances to the nearest remaining corners, _`l2, l3, etc.`_

**Note :**
* We do not check distances to a corner that we have already visited.
* We use Manhattan Distances(i.e. No Diagonal Movement).

So, if we haven't visited any corner yet,
```
h(n) = l1 + l2 + l3 + l4
```

This is demonstrated in the below gif:

![picture alt](https://i.imgflip.com/1bwymw.gif)

---

## Problem 7 : Eat all Dots Heuristic
### Strategy :
* We find the furthest food from pacman.
* We find the furthest food from the furthest food.
* We find the distance between them, `d1`.
* We now find the closest food to pacman and its distance, `d2`
```
h(n) = d1 + d2
```
**Note :**
* This is if there are atleast 2 food in the map. If there is only one, we return the distance between pacman and that food.
* Both distances d1 and d2 are found out using the function `maze_distance()`

##### maze_distance(point_a, point_b, walls) :
* Maze distance uses a custom class `TestGameState` to simulate an artifical problem, `PositionSearchProblem`, where the pacman starting position is `point_a` and the goal state is `point_b`.
* This function then returns the length of the BFS solution path to that simulated problem.

##### TestGameState:
This is a custom class, that uses the walls of the problem to create a stripped down version of the GameState. Below is the class definition :
```python
from pacman import GameState
class TestGameState(GameState):
    def __init__(self, walls, start, food=None):
        self.walls = walls
        if food != None: self.food = food
        self.num_food = 1
        self.pacman_position = start
    def getWalls(self):
        return self.walls
    def getPacmanPosition(self):
        return self.pacman_position
    def getNumFood(self):
        return 1
    def hasFood(self, x, y):
        return False
```

---
