#!/usr/bin/python
from Pathfinding import Map
from Pathfinding import Node
from Pathfinding import Util

# Create the map
map = Map(50, 50)
map.add_obstacles([(3, 1), (3, 2), (3, 3), (3, 4)])

# Set start and end nodes
start = map.get_node_by_location((1, 4))
goal = map.get_node_by_location((40, 32))

# Display their id's for debug
print "Start node id: ", start.get_id()
print "Goal node id: ", goal.get_id()

# Calculate all initial costs
for node in map.get_nodes():
    node.set_g_cost(node.calculate_g_cost(goal))
    node.set_h_cost(node.calculate_h_cost(goal))

# Create open and closed lists
open_list = [start]
closed_list = []

# Loop and perform calculations to determine path
while open_list:
    # Current node is the node with the shortest distance to the goal node
    current_node = Util.get_lowest(open_list, goal)

    # Remove the current node from the open list and place on closed list
    open_list.remove(current_node)
    closed_list.append(current_node)

    # Get all adjacent nodes to the current node
    for node in map.get_adjacent_nodes(current_node.get_location()):
        # Create a cost variable consisting of the current node's
        # current G cost + the cost to get to the loop node
        cost = current_node.get_g_cost() + current_node.calculate_g_cost(node.get_location())

        # If the adjacent node is in the open list and the calculated
        # cost is less the cost of the adjacent node, remove it from
        # the open list (new path is better)
        if node in open_list and cost < node.get_g_cost():
            # Remove node from open, new path is better
            open_list.remove(node)
        # More or less the same as above, though this should rarely
        # be encountered
        if node in closed_list and cost < node.get_g_cost():
            # Remove node from open, new path is better
            closed_list.remove(node)
        # If the adjacent node is not in the open list or the closed
        # list then set its G cost to the previously calculated cost
        # and append it to the open list, then set its parent to the
        # id of the current node
        if node not in open_list and node not in closed_list:
            node.set_g_cost(cost)
            open_list.append(node)
            node.set_parent(current_node.get_id())

# Calculations complete, print every node plus the parents
# (-1 represents no parent)
print "All nodes and their parents:"
for node in map.get_nodes():
    print node.get_id(), " -> ", node.get_parent()

# Grab the final path and display working backwards from the
# goal node to the start node
print "Final path from goal node to start node:"
final_path = []
current_node = goal
while current_node.get_location() != start.get_location():
    final_path.append(current_node.get_location())
    current_node = map.get_node_by_id(current_node.get_parent())
for location in final_path:
    print location, " -> ",
print start.get_location()
