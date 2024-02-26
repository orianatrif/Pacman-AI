
import util

class SearchProblem:

    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()

    def getGoalState(self):
        return self.goal


def tinyMazeSearch(problem):
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    fringe = util.Stack()
    visited = set()
    actionList = []
    fringe.push((problem.getStartState(), actionList))
    while fringe:
        node, actions = fringe.pop()
        if node not in visited:
            visited.add(node)
            if problem.isGoalState(node):
                return actions
            for successor in problem.getSuccessors(node):
                state, action, cost = successor
                nextActions = actions + [action]
                fringe.push((state, nextActions))
    return []

def breadthFirstSearch(problem):
    start = problem.getStartState()
    currentPosition = (start, [])
    explored = []
    moves = []
    queue = util.Queue()
    queue.push(currentPosition)

    while not queue.isEmpty() and not problem.isGoalState(currentPosition):
        currentPosition, moves = queue.pop()

        if currentPosition not in explored:
            explored.append(currentPosition)

            if not problem.isGoalState(currentPosition):
                possibleMoves = problem.getSuccessors(currentPosition)

                for position, action, cost in possibleMoves:
                    move = moves + [action]
                    node = (position, move)
                    queue.push(node)
    return moves


def uniformCostSearch(problem):
    fringe = util.PriorityQueue()
    visited = set()
    actionList = []
    fringe.push((problem.getStartState(), actionList), 0)
    while fringe:
        node, actions = fringe.pop()
        if node not in visited:
            visited.add(node)
            if problem.isGoalState(node):
                return actions
            for successor in problem.getSuccessors(node):
                state, action, cost = successor
                nextActions = actions + [action]
                nextCost = problem.getCostOfActions(nextActions)
                fringe.push((state, nextActions), nextCost)
    return []

def nullHeuristic(state, problem=None):
    return 0

def aStarSearch(problem, heuristic=nullHeuristic, explored_structure=list, queue_structure=util.PriorityQueue):
    start = problem.getStartState()
    currentPosition = (start, [], 0)
    explored = explored_structure()
    moves = []
    queue = queue_structure()
    queue.push(currentPosition, heuristic(start, problem))

    while not queue.isEmpty():
        currentPosition, moves, currentCost = queue.pop()

        if problem.isGoalState(currentPosition):
            return moves

        if currentPosition not in explored:
            explored.add(currentPosition) if isinstance(explored, set) else explored.append(currentPosition)

            possibleMoves = problem.getSuccessors(currentPosition)

            for position, action, cost in possibleMoves:
                move = moves + [action]
                totalCost = cost + currentCost
                totalCostH = totalCost + heuristic(position, problem)
                node = (position, move, totalCost)
                queue.push(node, totalCostH)

    return []


def fringeSearch(problem, heuristic=nullHeuristic):
    fringe = util.PriorityQueue()
    visited = set()
    actionList = []
    fringe.push((problem.getStartState(), actionList), heuristic(problem.getStartState(), problem))

    while not fringe.isEmpty():
        node, actions = fringe.pop()

        if node in visited:
            continue

        visited.add(node)

        if problem.isGoalState(node):
            return actions

        for successor in problem.getSuccessors(node):
            state, action, cost = successor
            nextActions = actions + [action]
            nextCost = problem.getCostOfActions(nextActions) + heuristic(state, problem)

            if state not in visited:
                fringe.push((state, nextActions), nextCost)

    return None

def iterativeDeepeningAStar(problem, heuristic=nullHeuristic):
    start_state = problem.getStartState()
    limit = heuristic(start_state, problem)

    while True:
        result = depthLimitedAStar(problem, start_state, heuristic, limit)
        if result is not None:
            return result
        limit += 1

def depthLimitedAStar(problem, start_state, heuristic, limit):
    fringe = util.PriorityQueue()
    visited = set()

    fringe.push((start_state, [], 0), 0)

    while not fringe.isEmpty():
        node, actions, cost = fringe.pop()

        if node in visited:
            continue

        visited.add(node)

        if problem.isGoalState(node):
            return actions

        successors = problem.getSuccessors(node)
        for succ_state, succ_action, succ_cost in successors:
            if succ_state not in visited:
                next_actions = actions + [succ_action]
                next_cost = cost + succ_cost
                h = next_cost + heuristic(succ_state, problem)
                fringe.push((succ_state, next_actions, next_cost), h)

    return None


def bidirectionalSearch(problem):
    start = problem.getStartState()
    currentForward = (start, [])
    exploredForward = []
    movesForward = []
    queueForward = util.Queue()
    queueForward.push(currentForward)
    tempForward = []

    end = problem.goal
    currentBackward = (end, [])
    exploredBackward = []
    movesBackward = []
    queueBackward = util.Queue()
    queueBackward.push(currentBackward)
    tempBackward = []

    def oppositeMoves(moves):
        opposite = []

        for move in moves:
            if move == 'North':
                opposite.append('South')
            elif move == 'East':
                opposite.append('West')
            elif move == 'South':
                opposite.append('North')
            elif move == 'West':
                opposite.append('East')

        return opposite

    while not queueForward.isEmpty() and not queueBackward.isEmpty():
        if not queueForward.isEmpty():
            currentForward, movesForward = queueForward.pop()

            if currentForward not in exploredForward:
                exploredForward.append(currentForward)

                if currentForward not in tempBackward:
                    possibleMoves = problem.getSuccessors(currentForward)
                    for position, action, cost in possibleMoves:
                        move = movesForward + [action]
                        node = (position, move)
                        queueForward.push(node)
                        tempForward.append(position)

                else:
                    while not queueBackward.isEmpty():
                        currentBackward, movesBackward = queueBackward.pop()
                        if currentForward == currentBackward:
                            moves = movesForward + oppositeMoves(movesBackward[::-1])
                            return moves

        if not queueBackward.isEmpty():
            currentBackward, movesBackward = queueBackward.pop()

            if currentBackward not in exploredBackward:
                exploredBackward.append(currentBackward)

                if currentBackward not in tempForward:
                    possibleMoves = problem.getSuccessors(currentBackward)

                    for position, action, cost in possibleMoves:
                        move = movesBackward + [action]
                        node = (position, move)
                        queueBackward.push(node)
                        tempBackward.append(position)

                else:
                    while not queueForward.isEmpty():
                        currentForward, movesForward = queueForward.pop()
                        if currentBackward == currentForward:
                            moves = movesForward + oppositeMoves(movesBackward[::-1])
                            return moves

    return []


# Example usage:
# solution = bidirectionalSearch(problem)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

fgs = fringeSearch  # generic search algorithm that uses a priority queue
# It repeatedly selects and expands the state with the lowest cost (priority) until a goal state is reached.
ida = iterativeDeepeningAStar  # combination of iterative deepening depth-first search (IDDFS) and A*
# performs depth-limited searches with increasing depth limits. At each iteration, it uses A* to explore
# states up to the depth limit, gradually increasing the limit until the goal is found
bds = bidirectionalSearch  # explores the search space from both the start state and the goal state simultaneously,
# aiming to meet in the middle