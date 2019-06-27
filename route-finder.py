import math
import heapq


class Intersection:
    def __init__(self, parent, pos, children):
        self.parent = parent
        self.pos = pos
        self.f = 0
        self.g = 0
        self.children = children

    def __eq__(self, node):
        return self.pos == node.pos

    def __lt__(self, node):
        return self.f < node.f


def shortest_path(M, start, goal):
    print("shortest path called")
    lookup = {start: 0}
    explored = []
    start_node = Intersection(None, start, M.roads[start])
    frontier = [start_node]
    goal_node = Intersection(None, goal, M.roads[goal])
    while len(frontier) > 0:
        # get node with min value

        curr_node = heapq.heappop(frontier)
        explored.append(curr_node)
        # if goal is found
        if curr_node == goal_node:
            path = []
            curr = curr_node
            while curr is not None:
                path.append(curr.pos)
                curr = curr.parent
            return list(reversed(path))
        child_nodes = []
        for child in curr_node.children:
            child_node = Intersection(curr_node, child, M.roads[child])
            child_nodes.append(child_node)
        for child in child_nodes:
            if child in explored:
                continue
            child.g = lookup[curr_node.pos] + calculate_distance(M.intersections[curr_node.pos],
                                                                 M.intersections[child.pos])
            lookup[child.pos] = child.g
            child.f = calculate_f(child.g, M.intersections[child.pos], M.intersections[goal_node.pos])

            skip = False
            for front in frontier:
                if front == child and child.g > front.g:
                    skip = True
                    break
            if skip:
                continue
            heapq.heappush(frontier, child)


def calculate_f(g, start_node, current_node):
    return g + calculate_distance(start_node, current_node)


def calculate_distance(coordinateA, coordinateB):
    return math.hypot((coordinateA[0] - coordinateB[0]), (coordinateA[1] - coordinateB[1])) * 100


pass
