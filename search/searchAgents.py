from game import Directions
from game import Agent
from game import Actions
import util
import time
import search


class GoWestAgent(Agent):

    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class SearchAgent(Agent):

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        if fn not in dir(search):
            raise AttributeError(fn + ' is not a search function in search.py.')
        func = getattr(search, fn)
        if 'heuristic' not in func.__code__.co_varnames:
            print(('[SearchAgent] using function ' + fn))
            self.searchFunction = func
        else:
            if heuristic in list(globals().keys()):
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError(heuristic + ' is not a function in searchAgents.py or search.py.')
            print(('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic)))
            self.searchFunction = lambda x: func(x, heuristic=heur)

        if prob not in list(globals().keys()) or not prob.endswith('Problem'):
            raise AttributeError(prob + ' is not a search problem type in SearchAgents.py.')
        self.searchType = globals()[prob]
        print(('[SearchAgent] using problem type ' + prob))

    def registerInitialState(self, state):
        if self.searchFunction == None: raise Exception("No search function provided for SearchAgent")
        starttime = time.time()
        problem = self.searchType(state)  # Makes a new search problem
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print(('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime)))
        if '_expanded' in dir(problem): print(('Search nodes expanded: %d' % problem._expanded))

    def getAction(self, state):
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class PositionSearchProblem(search.SearchProblem):

    def __init__(self, gameState, costFn=lambda x: 1, goal=(1, 1), start=None, warn=True, visualize=True):
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')

        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display):
                    __main__._display.drawExpandedCells(self._visitedlist)

        return isGoal

    def getSuccessors(self, state):
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append((nextState, action, cost))

        self._expanded += 1  # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        if actions == None: return 999999
        x, y = self.getStartState()
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x, y))
        return cost


class StayEastSearchAgent(SearchAgent):
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)


class StayWestSearchAgent(SearchAgent):
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)


def manhattanHeuristic(position, problem, info={}):
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem, info={}):
    xy1 = position
    xy2 = problem.goal
    return ((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2) ** 0.5


def chebyshevHeuristic(position, problem, info={}):
    xy1 = position
    xy2 = problem.goal
    return max(abs(xy1[0] - xy2[0]), abs(xy1[1] - xy2[1]))


class CornersProblem(search.SearchProblem):
    def __init__(self, startingGameState):
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height - 2, self.walls.width - 2
        self.corners = ((1, 1), (1, top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print('Warning: no food in corner ' + str(corner))
        self._expanded = 0

    def getStartState(self):
        visited = []
        return (self.startingPosition, visited)

    def isGoalState(self, state):
        return len(state[1]) == 4

    def getSuccessors(self, state):
        currentPosition, foundCorners = state[0], state[1]
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = currentPosition
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            hitsWall = self.walls[nextx][nexty]
            if not hitsWall:
                if (nextx, nexty) in self.corners and (nextx, nexty) not in foundCorners:
                    visited = foundCorners + [(nextx, nexty)]
                    successors.append((((nextx, nexty), visited), action, 1))
                else:
                    successors.append((((nextx, nexty), foundCorners), action, 1))
        self._expanded += 1
        return successors

    def getCostOfActions(self, actions):
        if actions == None: return 999999
        x, y = self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)


def cornersHeuristic(state, problem):
    corners = problem.corners
    walls = problem.walls
    unvisited = []
    visited = state[1]
    node = state[0]
    heuristic = 0
    for corner in corners:
        if not corner in visited:
            unvisited.append(corner)
    while unvisited:
        distance, corner = min([(util.chebyshevDistance(node, corner), corner) \
                                for corner in unvisited])
        heuristic += distance
        node = corner
        unvisited.remove(corner)
    return heuristic


class AStarCornersAgent(SearchAgent):
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem


def findClosestPoint(location, goalArray):
    """
    Helper function for corners
    """

    closestPoint = 0
    closestPointCost = util.manhattanDistance(location, goalArray[0])

    for j in range(len(goalArray)):
        # calculate distance between current state to corner
        cornerLocation = goalArray[j]
        lengthToCorner = util.manhattanDistance(location, cornerLocation)

        if lengthToCorner < closestPointCost:
            closestPoint = j
            closestPointCost = lengthToCorner

    return (closestPoint, closestPointCost)


def findFarthestPoint(location, goalArray):
    """
    Helper function for corners
    """

    farthestPoint = 0
    farthestPointCost = util.manhattanDistance(location, goalArray[0])

    for j in range(len(goalArray)):
        # calculate distance between current state to corner
        cornerLocation = goalArray[j]
        lengthToCorner = util.manhattanDistance(location, cornerLocation)

        if lengthToCorner > farthestPointCost:
            farthestPoint = j
            farthestPointCost = lengthToCorner

    return (farthestPoint, farthestPointCost)


class FoodSearchProblem:
    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0  # DO NOT CHANGE
        self.heuristicInfo = {}  # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1  # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x, y = self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost


class AStarFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, ourHeuristic)
        self.searchType = FoodSearchProblem


def foodHeuristic(state, problem):
    """Encourage Pacman to eat all the pellets as fast as possible."""
    position, foodGrid = state

    heuristic = 0
    foodList = foodGrid.asList()

    if len(foodList) > 0:
        closestPoint = findClosestPoint(position, foodList)
        farthestPoint = findFarthestPoint(position, foodList)

        closestPointIndex = closestPoint[0]
        farthestPointIndex = farthestPoint[0]

        currentNode = problem.startingGameState
        closestFoodNode = foodList[closestPointIndex]
        farthestFoodNode = foodList[farthestPointIndex]

        currentToClosest = mazeDistance(position, closestFoodNode, currentNode)

        closestToFarthest = mazeDistance(closestFoodNode, farthestFoodNode, currentNode)

        heuristic = currentToClosest + closestToFarthest

    return heuristic


class ClosestDotSearchAgent(SearchAgent):
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while (currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState)  # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception('findPathToClosestDot returned an illegal move: %s!\n%s' % t)
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print('Path found with cost %d.' % len(self.actions))

    def findPathToClosestDot(self, gameState):
        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)
        return search.astar(problem)


class AnyFoodSearchProblem(PositionSearchProblem):

    def __init__(self, gameState):
        self.food = gameState.getFood()
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def isGoalState(self, state):
        x, y = state
        return (x, y) in self.food.asList()


def mazeDistance(point1, point2, gameState):
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))


def ourHeuristic(state, problem):
    position, foodGrid = state

    foodList = foodGrid.asList()

    if len(foodList) == 0:
        return 0
    closestFood = min(foodList, key=lambda food: util.chebyshevDistance(position, food))

    distanceToClosestFood = util.chebyshevDistance(position, closestFood)

    heuristic = distanceToClosestFood + len(foodList)

    return heuristic

