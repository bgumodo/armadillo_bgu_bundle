Steps to follow:

1. There is a library called libobj_pos.a that contains modified yolo code, and this is used for object identification.
Access to this library happens via function predict_scuares (misspelled)

2. Usage:
	(a): Update the location in the cmake file in place of: "/opt/object_positions/libobj_pos.a" 
	(b): Update the location (OBJ_POS_PATH) of this library in the src/find_object.cpp file too. 
	(c): In CmakeLists.txt, to run find_object, the only 6 relevant lines are required, starts from "Adding Yolo detector dependancies (starts)"
