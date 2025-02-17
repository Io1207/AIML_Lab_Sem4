import heapq

start=(3, 3, 'L')
goal=(0, 0, 'R')


def valid(state):
    mLeft, cLeft, boat = state
    mRight = 3 - mLeft
    cRight = 3 - cLeft

    return (mLeft >= 0 and cLeft >= 0 and mRight >= 0 and cRight >= 0 and (mLeft == 0 or mLeft >= cLeft) and (mRight == 0 or mRight >= cRight))

def getNeighbours(state):
    mLeft, cLeft, boat = state
    moves = [(2, 0), (0, 2), (1, 1), (1, 0), (0, 1)]
    neighbors = []
    for m,c in moves:
        if boat == 'L':
            next = (mLeft - m, cLeft - c, 'R')
        else:
            next = (mLeft + m, cLeft + c, 'L')
        if valid(next):
            neighbors.append(next)
    return neighbors

def heuristicMid(state):
    mLeft, cLeft, _ = state
    return (mLeft + cLeft) // 2


def aStar(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristicMid(start), 0, start, []))
    visited = set()

    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path + [current]

        if current in visited:
            continue
        visited.add(current)

        for neighbor in getNeighbours(current):
            if neighbor not in visited:
                gUpdate = g + 1
                fUpdate = gUpdate + heuristicMid(neighbor)
                heapq.heappush(open_set, (fUpdate, gUpdate, neighbor, path + [current]))

    return None

solution = aStar(start, goal)
if solution:
    for step in solution:
        print(step)
