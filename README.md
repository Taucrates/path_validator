# path_validator
This node allows to replanificate paths of the global_planner for multi robot tasks or dynamic environments, using gmapping as a map source.

The node is subscribed to "/map" and "/move_base_simple/goal" to get the map's grid and the global objective.
The node publish the global objective obtained from "/move_base_simple/goal" if the path cross over an obstacle located in the updated map. In that case the global_planner replan a new path, taking into account the new obstacles.
