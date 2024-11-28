import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index


    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
    
        draw(self.map, path[::-1], self.map_index)


    def goal_test(self, current_state):
        return current_state == self.goal_state


    def get_cost(self, current_state, next_state):
        return 1


    def get_successors(self, state):
        successors = []
        row_change = [0, 0, -1, 1]
        col_change = [-1, 1, 0, 0]
        r, c = state

        for i in range(4):
            row = r + row_change[i]
            col = c + col_change[i]
            if self.map[row, col] != 0.0:
                successors.append((r + row_change[i], c + col_change[i]))

        return successors


    # heuristics function
    def heuristics(self, state):
        r, c = state
        row, col = self.goal_state
        return abs(r - row) + abs(c - col)


    # priority of node 
    def priority(self, node):
        return node.cost_from_start + self.heuristics(node.state)

    
    # solve it
    def solve(self):
        state = self.start_state

        if self.goal_test(state):
            return state

        node = Node(state, 0, None)
        self.visited.append(state)
        index = 0
        priority_queue = [(self.priority(node), index, node)]

        while priority_queue:
            best_node = heappop(priority_queue)[2]

            successors = self.get_successors(best_node.state)

            for next_state in successors:
                if next_state in self.visited:
                    continue

                self.visited.append(next_state)

                next_node = Node(next_state, best_node.cost_from_start + self.get_cost(best_node.state, next_state),
                                 best_node)

                if self.goal_test(next_state):
                    self.draw(next_node)
                    return

                index = index + 1
                heappush(priority_queue, (self.priority(next_node), index, next_node))

            
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1
    
    data = np.load('map_'+str(index)+'.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
    