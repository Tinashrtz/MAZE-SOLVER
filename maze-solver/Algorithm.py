from _helpers import Node, Stack, Queue, PriorityQueue
import heapq
from collections import deque

class DFS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.stack = Stack()
        self.stack.push(Node(pos=start_pos, parent=None))

    def get_successors(self, x, y):
        return [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def update(self, grid):
        curr_state = self.stack.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1, 3]: # 1: empty cell has not explored yet, 3: goal cell
                self.stack.push(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break
            
            grid[x, y] = 4 # visited - we use this number for marking the viseted nodes

        return solution_path, done, grid


class BFS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.queue = deque()
        self.queue.append(Node(pos=start_pos, parent=None))

    def get_successors(self, x, y):
        return [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def update(self, grid):
        if not self.queue:
            return [], False, grid
        
        curr_state = self.queue.popleft()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1, 3]: # 1: empty cell has not explored yet, 3: goal cell
                self.queue.append(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

            grid[x, y] = 4 # visited- we use this number for marking the viseted nodes

        return solution_path, done, grid


class IDS_Algorithm:  #Iterative Deepening Search
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim

    def get_successors(self, x, y):
        return [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def dfs(self, start, depth, grid):
        stack = [(start, [start])] #Each item in stack is a tuple of current position and path 
        visited = set()   #avoid revisiting 
        
        while stack:
            curr_pos, path = stack.pop()
            x, y = curr_pos

            if len(path) > depth:
                continue  # If depth limit exceeded, continue to the next iteration

            if curr_pos == self.goal_pos:
                return path  # If goal found, return the path

            visited.add(curr_pos)

            for step in self.get_successors(x, y):
                if step not in visited and self.is_valid_cell(step) and grid[step[0]][step[1]] in [1, 3]:
                    stack.append((step, path + [step]))

        return None  # If goal not found within depth limit, return None

    def update(self, grid):
        depth = 0
        while True:
            result = self.dfs(self.start_pos, depth, [row[:] for row in grid])  # Use a copy of the grid for each depth
            if result is not None:
                return result, True, grid
            depth += 1





class A_Star_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim

    def heuristic(self, pos): #Manhatan distance is used for heuristic function
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def get_successors(self, x, y):
        return [(x+1, y), (x-1, y), (x, y+1), (x, y-1),(x-1, y-1),(x+1, y-1),(x-1, y+1),(x+1, y+1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def update(self, grid):
        open_set = []
        closed_set = set()
        heapq.heappush(open_set, (self.heuristic(self.start_pos), 0, self.start_pos, [self.start_pos]))

        while open_set:
            _, cost, curr_pos, path = heapq.heappop(open_set)

            if curr_pos == self.goal_pos:
                return path, True, grid

            if curr_pos in closed_set:
                continue

            for step in self.get_successors(*curr_pos):
                if self.is_valid_cell(step) and grid[step[0]][step[1]] in [1, 3]: # 1: empty cell has not explored yet, 3: goal cell
                    new_cost = cost + 1
                    new_path = path + [step]
                    heapq.heappush(open_set, (new_cost + self.heuristic(step), new_cost, step, new_path))

            closed_set.add(curr_pos)

        return None, False, grid


class A_Star_Geometric_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim

    def heuristic(self, pos): #Euclidean distance is used for heuristic function
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def get_successors(self, x, y):
        return [(x+1, y), (x-1, y), (x, y+1), (x, y-1),(x-1, y-1),(x+1, y-1),(x-1, y+1),(x+1, y+1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def update(self, grid):
        open_set = []
        closed_set = set()
        heapq.heappush(open_set, (self.heuristic(self.start_pos), 0, self.start_pos, [self.start_pos]))

        while open_set:
            _, cost, curr_pos, path = heapq.heappop(open_set)

            if curr_pos == self.goal_pos:
                return path, True, grid

            if curr_pos in closed_set:
                continue

            for step in self.get_successors(*curr_pos):
                if self.is_valid_cell(step) and grid[step[0]][step[1]] in [1, 3]: # 1: empty cell has not explored yet, 3: goal cell
                    new_cost = cost + 1
                    new_path = path + [step]
                    heapq.heappush(open_set, (new_cost + self.heuristic(step), new_cost, step, new_path))

            closed_set.add(curr_pos)
            #we should add some if for optimization =))

        return None, False, grid

