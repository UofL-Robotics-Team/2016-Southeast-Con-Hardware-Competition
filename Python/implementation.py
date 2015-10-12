#!/usr/bin/python
from Pathfinding import Map
from Pathfinding import Node
from Pathfinding import Util

# Create the map
map = Map(5, 5)
map.add_obstacles([(3, 1), (3, 2), (3, 3), (3, 4)])

start = map.get_node_by_location((1, 4))
goal = map.get_node_by_location((4, 4))

print "Start node id: ", start.get_id()
print "Goal node id: ", goal.get_id()

# Calculate all initial costs
for node in map.get_nodes():
    node.set_g_cost(node.calculate_g_cost(goal))
    node.set_h_cost(node.calculate_h_cost(goal))

open_list = [start]
closed_list = []

while open_list:
    current_node = Util.get_lowest(open_list, goal)
    open_list.remove(current_node)
    closed_list.append(current_node)

    for node in map.get_adjacent_nodes(current_node.get_location()):
        cost = current_node.get_g_cost() + current_node.calculate_g_cost(node.get_location())

        if node in open_list and cost < node.get_g_cost():
            # Remove node from open, new path is better
            open_list.remove(node)
        if node in closed_list and cost < node.get_g_cost():
            # Remove node from open, new path is better
            closed_list.remove(node)
        if node not in open_list and node not in closed_list:
            node.set_g_cost(cost)
            open_list.append(node)
            node.set_parent(current_node.get_id())

print "All nodes and their parents:"
for node in map.get_nodes():
    print node.get_id(), " -> ", node.get_parent()

print "Final path from goal node to start node:"
final_path = []
current_node = goal
while current_node.get_location() != start.get_location():
    final_path.append(current_node.get_location())
    current_node = map.get_node_by_id(current_node.get_parent())
for location in final_path:
    print location, " -> ",
print start.get_location()
