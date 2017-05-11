# Navigation instructions
Include navigation_demo.launch to change base_local_planner and move_base parameters for the navigation to work well.

Move.py node can be used in two ways:

* Using hard coded locations managed in a dictionary inside the node, each location is defined by a key which will be the locations name, for example: room_2 for the initial robot position, room_4 for the coke can position etc. The node is listening to the topic '/navigation/move_cmd' of type std_msgs/String, for the name of the desired location, when receiving a location, a goal command will be sent to the move_base with the coordinates corresponding to the name received. The node will send a result to the topic '/navigation/move_res' with the message 'Success' if arrived to the destination, otherwise 'Failed'.

* Managing the locations outside of the node. The node will be listening to a topic '/navigation/pos_cmd' of type geometry_msgs/Pose. When receiving a message, the node will navigate to the position and will send a result as mentioned earlier.
