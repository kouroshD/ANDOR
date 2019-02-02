Starting the AND/OR graph process, it loads the AND/OR graph description from a \textit{text} file, and generates the AND/OR graph $G$.

To generate the AND/OR graph, we describe it with the following structure offline:

```ruby
# 1st line:
[AND/OR graph name] [No. nodes (N)] [root node name]
#list all the nodes in the AND/OR graph:
[node_1 name] [node_1 cost]
[node_2 name] [node_2 cost]
...
...
...
[node_N name] [node_N cost]
#After the list of nodes, list all the hyper-arcs (H  hyper-arcs):
[hyper-arc_1 name] [No. child nodes(NC1)] [parent node name] [hyper-arc_1 cost] [hyper-arc_1 lower level graph file name]
[1st child node name]
...
[NC1'th child node name]
[hyper-arc_2 name] [No. child nodes(NC2)] [parent node name] [hyper-arc_2 cost] [hyper-arc_2 lower level graph file name]
[1st child node name]
...
[NC2'th child node name]
....
....
....
[hyper-arc_H name] [No. child nodes(NCH)] [parent node name] [hyper-arc_H cost] [hyper-arc_H lower level graph file name]
[1st child node name]
...
[NCH'th child node name]
```

If a hyper-arc does not hold a lower level AND/OR graph, we simply put a dash (`-`) character.

The description of the hierarchical AND/OR of a **table assembly** scenario with two legs, presented in [Figure 4 of paper](), is as following:

```
TableAssembly 7 Table_FinalPose
Plate_initialPose 0
Plate_assemblyPose 0
Leg1_initialPose 0
Leg2_initialPose 0
Leg1_Plate_connected 0
Leg2_Plate_connected 0
Table_FinalPose 0
h0 1 Plate_assemblyPose 1 -
Plate_initialPose
h1 2 Leg1_Plate_connected 1 basic_connection
Leg1_initialPose
Plate_assemblyPose
h2 2 Leg2_Plate_connected 1 basic_connection
Leg2_initialPose
Leg1_Plate_connected
h3 1 Table_FinalPose 1 -
Leg2_Plate_connected
````
In this description `basic_connection` is the lower level AND/OR file name which exists in the same path (folder) of the higher level AND/OR graph (`basic_connection.txt`).
This AND/OR graph describes how the human or the robot can cooperatively connect a leg to a plate (tabletop) using FOL AND/OR:

```
ConnectLegPlate 4 Leg_Plate_Connected
Leg_Plate_Connected 0
Leg_initialPose 0
Leg_middlePose 0
Plate 0
h1 2 Leg_middlePose 2 -
Leg_initialPose
Plate
h2 2 Leg_Plate_Connected 1 -
Leg_initialPose
Plate
h3 1 Leg_Plate_Connected 1 -
Leg_middlePose
h4_human 1 Leg_Plate_Connected 2 -
Leg_middlePose
h5_human 2 Leg_Plate_Connected 5 -
Leg_initialPose
Plate
```
The interpretation of nodes and hyper-arcs titles given in the provided example of the **table assembly** AND/OR graph is following.

| Title | Interpretation | 
| --- | --- |
| `TableAssembly` | the name of the higher level AND/OR graph $G^0$ (how to assemble a table) |
| `ConnectLegPlate` | the name of the lower level AND/OR graph $G^1$ (how to connect a leg to a plate or tabletop) | 
| `Table$\_$FinalPose` | the root node name of $G^0$ (the table is assembled and it is in its final pose) | 
| `Plate$\_$initialPose` | a node name (the tabletop or plate is in its initial position) | 
| `Plate$\_$assemblyPose` | a node name (the plate is in assembly position) | 
| `Leg$(i)\_$initialPose` | a node name (the leg $i$ is in initial position) | 
| `Leg$(i)\_$Plate$\_$connected` | a node name (the leg $i$ is connected to the plate) | 
| `h$i$` | a hyper-arc name (a state transition from the child nodes to the parent node) | 
| `basic$\_$connection` | the file name of the lower level AND/OR Graph $G^1$ | 
| `Leg$\_$middlePose` | a node name of Graph $G^1$ (a leg is placed in front of the human) | 
| `Plate` | a node name of Graph $G^1$ (a plate or tabletop) | 
| `Leg$\_$Plate$\_$Connected` | a root node name of Graph $G^1$ (a leg is connected to the tabletop) | 
| `h$(i)\_$human` | a hyper-arc name of graph $G^1$ (a state transition from the child nodes to the parent node and done by human) | 

