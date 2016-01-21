#!/usr/bin/python

class Node(object):
    'Represents a single node (or point of movement) within a map.'
    nodes = 0

    def __init__(self, location):
        self._id = Node.nodes
        self._parent = -1
        self._location = location
        self._h = 0
        self._g = 0

        # Add to overall node count
        Node.nodes += 1

    def get_id(self):
        return self._id

    def set_parent(self, parent):
        self._parent = parent

    def get_parent(self):
        return self._parent

    def set_h_cost(self, cost):
        self._h = cost

    def set_g_cost(self, cost):
        self._g = cost

    def get_h_cost(self):
        return self._h

    def get_g_cost(self):
        return self._g

    # Calculates G cost to move from a node to an adjacent node
    def calculate_g_cost(self, goal):
        return 10

    # Calculates the H cost using the diagonal distance heuristic
    def calculate_h_cost(self, goal):
        dx = abs(self._location[0] - goal.get_location()[0])
        dy = abs(self._location[1] - goal.get_location()[1])
        return (dx + dy) - 1 * min(dx, dy)

    def get_f_cost(self):
        return self._g + self._h

    def get_location(self):
        return self._location


class Map(object):
    'Represents a map of individual nodes.'
    def __init__(self, cols, rows):
        self._cols = cols
        self._rows = rows

        # Create reference map for neighbor calculation
        self._nodes = []
        self._reference_map = []
        for x in range(cols):
            for y in range(rows):
                self._nodes.append(Node((x, y)))
                self._reference_map.append((x, y))

    # Prints the map coordinates in an easy to read string
    def print_map(self):
        print self._reference_map

    # Return list of all node objects in this map
    def get_nodes(self):
        return self._nodes

    # Returns a map of (x, y) coordinates for reference
    def get_reference_map(self):
        return self._reference_map

    # Adds obstacles to the map by removing the associated nodes from the reference map
    def add_obstacles(self, locations):
        for location in locations:
            self._reference_map.remove(location)

    # Returns the node object at the given location
    def get_node_by_location(self, location):
        for node in self._nodes:
            if node.get_location() == location:
                return node

    # Returns the node object with the given id
    def get_node_by_id(self, id):
        for node in self._nodes:
            if node.get_id() == id:
                return node

    # Finds and returns all nodes adjacent to the given node
    def get_adjacent_nodes(self, location):
        dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        result = []
        for dir in dirs:
            neighbor_loc = (location[0] + dir[0], location[1] + dir[1])
            if neighbor_loc in self._reference_map:
                result.append(self.get_node_by_location(neighbor_loc))
        return result


class Util:
    # Returns the node with the lowest cost
    @staticmethod
    def get_lowest(list, goal):
        target_node = list[0]
        low_cost = target_node.get_f_cost()

        for i in range(len(list)):
            new_node = list[i]
            new_cost = new_node.get_f_cost()

            if new_cost < low_cost:
                low_cost = new_cost
                target_node = new_node

        return target_node

