import numpy as np
from collections import deque
import heapq
from typing import List, Tuple, Set, Dict
"""
Do not import any other package unless allowed by te TAs in charge of the lab.
Do not change the name of any of the functions below.
"""

def displaced(state):
    """
    displaced tiles heuristic
    """
    ans=0
    goal=[1,2,3,4,5,6,7,8,0]
    for i in range(9):
        if state[i]!=goal[i]:
            ans=ans+1
    return ans

def stringMove(path):
    """
    converting state sequence into list of characters to show transitions
    """
    ans=[]
    for i in range(len(path)-1):
        initial=list(path[i])
        final=list(path[i+1])
        inIdx=initial.index(0)
        finIdx=final.index(0)
        yIn,xIn=divmod(inIdx,3)
        yFin,xFin=divmod(finIdx,3)
        if((xFin - xIn) == 1):
            ans.append("R")
        elif((xFin - xIn) == -1):
            ans.append("L")
        elif((yFin - yIn) == -1):
            ans.append("U")
        else:
            ans.append("D")
    return ans

def manhattan(currState):
    """
    manhattan distance heuristic
    """
    distance = 0
    goal=[1,2,3,4,5,6,7,8,0]
    for i, j in enumerate(currState):
        if j == 0:
            continue
        goalx = goal.index(j)
        x1=i//3
        y1=i%3
        x2=goalx//3
        y2=goalx%3
        distance=distance+abs(x1 - x2) + abs(y1 - y2)
    return distance

def getNeighbours(state):
    """
    to find the immediate successor states
    """
    neighbors = []
    state=list(state)
    zeroIdx = state.index(0)
    x=zeroIdx//3
    y=zeroIdx%3
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dx, dy in directions:
        newx, newy = x + dx, y + dy
        if 0 <= newx < 3 and 0 <= newy < 3:
            newIdx = newx * 3 + newy
            newState = list(state)
            newState[zeroIdx], newState[newIdx] = newState[newIdx], newState[zeroIdx]
            neighbors.append(newState)
    return neighbors

def bfs(initial: np.ndarray, goal: np.ndarray) -> Tuple[List[str], int]:
    """
    Implement Breadth-First Search algorithm to solve 8-puzzle problem.
    
    Args:
        initial (np.ndarray): Initial state of the puzzle as a 3x3 numpy array.
                            Example: np.array([[1, 2, 3], [4, 0, 5], [6, 7, 8]])
                            where 0 represents the blank space
        goal (np.ndarray): Goal state of the puzzle as a 3x3 numpy array.
                          Example: np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])
    
    Returns:
        Tuple[List[str], int]: A tuple containing:
            - List of moves to reach the goal state. Each move is represented as
              'U' (up), 'D' (down), 'L' (left), or 'R' (right), indicating how
              the blank space should move
            - Number of nodes expanded during the search

    Example return value:
        (['R', 'D', 'R'], 12) # Means blank moved right, down, right; 12 nodes were expanded
              
    """
    # TODO: Implement this function

    initial=initial.flatten()
    goal=goal.flatten()

    openList = []
    heapq.heappush(openList, (0, 0, initial, []))
    visited = set() #closed list

    while openList:
        f, g, current, path = heapq.heappop(openList)

        if np.array_equal(current, goal):
            path=path+[current]
            dirString=stringMove(path)
            solTuple=(dirString,len(visited))
            return solTuple

        if tuple(current) in visited:
            continue #already expanded node
        visited.add(tuple(current))

        for neighbor in getNeighbours(current):
            if tuple(neighbor) not in visited:
                gUpdate=g+1
                heapq.heappush(openList,(gUpdate, gUpdate,neighbor,path+[current]))
    return ([],len(visited))
    pass

def dfs(initial: np.ndarray, goal: np.ndarray) -> Tuple[List[str], int]:
    """
    Implement Depth-First Search algorithm to solve 8-puzzle problem.
    
    Args:
        initial (np.ndarray): Initial state of the puzzle as a 3x3 numpy array
        goal (np.ndarray): Goal state of the puzzle as a 3x3 numpy array
    
    Returns:
        Tuple[List[str], int]: A tuple containing:
            - List of moves to reach the goal state
            - Number of nodes expanded during the search
    """
    # TODO: Implement this function
    # print("entered djikstra")
    initial=initial.flatten()
    goal=goal.flatten()

    openList = []
    heapq.heappush(openList, (0, 1, initial, []))
    visited = set() #closed list

    while openList:
        # print("check1")
        f, g, current, path = heapq.heappop(openList)
        if np.array_equal(current, goal)==True:
            # print("xyz")
            path=path+[current]
            dirString=stringMove(path)
            solTuple=(dirString,len(visited))
            return solTuple

        if tuple(current) in visited:
            # print("abc")
            continue #already expanded node
        # print("check2")
        visited.add(tuple(current))

        for neighbor in getNeighbours(current):
            # print("not here")
            if tuple(neighbor) not in visited:
                # print("check3")
                gUpdate=float(1/(int(1/g)+1))
                heapq.heappush(openList,(gUpdate, gUpdate,neighbor,path+[current]))
    return ([],len(visited))
    pass

def dijkstra(initial: np.ndarray, goal: np.ndarray) -> Tuple[List[str], int, int]:
    """
    Implement Dijkstra's algorithm to solve 8-puzzle problem.
    
    Args:
        initial (np.ndarray): Initial state of the puzzle as a 3x3 numpy array
        goal (np.ndarray): Goal state of the puzzle as a 3x3 numpy array
    
    Returns:
        Tuple[List[str], int, int]: A tuple containing:
            - List of moves to reach the goal state
            - Number of nodes expanded during the search
            - Total cost of the path for transforming initial into goal configuration
            
    """
    # TODO: Implement this function
    initial=initial.flatten()
    goal=goal.flatten()

    openList = []
    heapq.heappush(openList, (0, 0, initial, []))
    visited = set()  #closed list

    while openList:
        f, g, current, path = heapq.heappop(openList)

        #goal reached
        if np.array_equal(current, goal):
            path=path+[current]
            dirString=stringMove(path)
            solTuple=(dirString,len(visited),len(dirString))
            return solTuple

        if tuple(current) in visited:
            continue #already expanded node
        visited.add(tuple(current))

        for neighbor in getNeighbours(current):
            if tuple(neighbor) not in visited:
                gUpdate=g+1
                heapq.heappush(openList,(gUpdate, gUpdate,neighbor,path+[current]))
    return ([],len(visited),0)
    pass

def astar_md(initial: np.ndarray, goal: np.ndarray) -> Tuple[List[str], int, int]:
    """
    Implement A* Search with Manhattan Distance heuristic to solve 8-puzzle problem.
    
    Args:
        initial (np.ndarray): Initial state of the puzzle as a 3x3 numpy array
        goal (np.ndarray): Goal state of the puzzle as a 3x3 numpy array
    
    Returns:
        Tuple[List[str], int, int]: A tuple containing:
            - List of moves to reach the goal state
            - Number of nodes expanded during the search
            - Total cost of the path for transforming initial into goal configuration
    """
    # TODO: Implement this function
    
    initial=initial.flatten()
    goal=goal.flatten()

    openList = []
    heapq.heappush(openList, (0 + manhattan(initial), 0, initial, []))
    visited = set() #closed list

    while openList:
        f, g, current, path = heapq.heappop(openList)

        # If the goal is reached, return the path
        if np.array_equal(current, goal):
            path=path+[current]
            dirString=stringMove(path)
            solTuple=(dirString,len(visited),len(dirString))
            return solTuple

        if tuple(current) in visited:
            continue
        visited.add(tuple(current))

        # Explore neighbors
        for neighbor in getNeighbours(current):
            if tuple(neighbor) not in visited:
                gUpdate=g+1
                fUpdate=gUpdate+manhattan(neighbor)
                heapq.heappush(openList,(fUpdate, gUpdate,neighbor,path + [current]))
    return ([],len(visited),0)
    pass

def astar_dt(initial: np.ndarray, goal: np.ndarray) -> Tuple[List[str], int, int]:
    """
    Implement A* Search with Displaced Tiles heuristic to solve 8-puzzle problem.
    
    Args:
        initial (np.ndarray): Initial state of the puzzle as a 3x3 numpy array
        goal (np.ndarray): Goal state of the puzzle as a 3x3 numpy array
    
    Returns:
        Tuple[List[str], int, int]: A tuple containing:
            - List of moves to reach the goal state
            - Number of nodes expanded during the search
            - Total cost of the path for transforming initial into goal configuration
    """
    # TODO: Implement this function
    
    initial=initial.flatten()
    goal=goal.flatten()

    openList = []
    heapq.heappush(openList, (0 + displaced(initial), 0, initial, []))
    visited = set() #closed list

    while openList:
        f, g, current, path = heapq.heappop(openList)

        #goal reached
        if np.array_equal(current, goal):
            # print("entered here")
            path=path+[current]
            dirString=stringMove(path)
            solTuple=(dirString,len(visited),len(dirString))
            return solTuple

        if tuple(current) in visited:
            continue
        visited.add(tuple(current))

        for neighbor in getNeighbours(current):
            if tuple(neighbor) not in visited:
                gUpdate=g+1
                fUpdate=gUpdate+displaced(neighbor)
                heapq.heappush(openList,(fUpdate, gUpdate,neighbor,path + [current]))
    return ([],len(visited),0)
    pass

# Example test case to help verify your implementation
if __name__ == "__main__":
    # Example puzzle configuration
    initial_state = np.array([
        [6, 4, 3],
        [7, 1,0],
        [2, 8,5 ]
    ])
    
    goal_state = np.array([
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 0]
    ])
    
    # Test each algorithm
    print("Testing BFS...")
    bfs_moves, bfs_expanded = bfs(initial_state, goal_state)
    bfs(initial_state,goal_state)
    print(f"BFS Solution: {bfs_moves}")
    print(f"Nodes expanded: {bfs_expanded}")
    
    print("\nTesting DFS...")
    dfs_moves, dfs_expanded = dfs(initial_state, goal_state)
    print(f"DFS Solution: {dfs_moves}")
    print(f"Nodes expanded: {dfs_expanded}")
    
    print("\nTesting Dijkstra...")
    dijkstra_moves, dijkstra_expanded, dijkstra_cost = dijkstra(initial_state, goal_state)
    print(f"Dijkstra Solution: {dijkstra_moves}")
    print(f"Nodes expanded: {dijkstra_expanded}")
    print(f"Total cost: {dijkstra_cost}")
    
    print("\nTesting A* with Displaced Tiles...")
    dt_moves, dt_expanded, dt_fscore = astar_dt(initial_state, goal_state)
    print(f"A* (DT) Solution: {dt_moves}")
    print(f"Nodes expanded: {dt_expanded}")
    print(f"Total cost: {dt_fscore}")
    
    print("\nTesting A* with Manhattan Distance...")
    md_moves, md_expanded, md_fscore = astar_md(initial_state, goal_state)
    print(f"A* (MD) Solution: {md_moves}")
    print(f"Nodes expanded: {md_expanded}")
    print(f"Total cost: {md_fscore}")
