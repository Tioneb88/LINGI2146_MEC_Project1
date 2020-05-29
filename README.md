# LINGI2146_MEC_Project1

First project of the course LINGI2146 - Mobile and Embedded Computing at UCLouvain (Belgium). Implementation of a sensors network connected to a server in Contiki (with simulation in Cooja).

## Project description
Fictive scenario of a building management system. We need to manage the air quality of the rooms.

3 different nodes :
- sensor node : directly attached to an air quality sensor (here data generated randomly) and to a motorized valve for air ventilation (respresented here by a LED)
- computation node : node able to compute data of sensor node and send message to open or not the valve (can only supervize 5 sensor nodes simultaneously)
- border node : node at the root of the tree built by the node, connected to the server

The sensor nodes read data values once per minute. They send data to their parent node. The data arrives to the server or to a computation node with free space (limited to 2 nodes). The
data is stored and interpreted. If the slope of the line obtained by a least-squares fit to the last thirty sensor values is above a certain threshold, a message is sent to open the valve 
for 10 minutes.
Only the computation nodes or the server can compute a leat-squares and store the data. The lost of a child by an other node (especially a computation node) is supported. Indeed, a node 
can lose the connection and change its parent by reconnecting. Each node has only one parent, chosen via the signal strength. Each node has a rank greater than the rank of its parent so 
that there is exactly one path from the root node (border node) to any other node.

The nodes communicate over a wireless IEEE 802.15.4 multi-hop network, using the Rime modules for single-hop (reliable) unicast and best effort local area broadcast. All nodes are simulated
in Cooja with Z1 mote type.

The server is a Python application running on Linux. It receives and replies to messages from the nodes. The server is connected to the border node via a network connection to Cooja on 
port 60001.


## Repositiory description
- __/border node__ : contains all files relative to the border node
	- __Makefile__ : file needed to compile border.c
	- __border.c__ : file containing the C code of the border node
- __/computation node__ : contains all files relative to the computation nodes 
	- __Makefile__ : file needed to compile computation_node.c
	- __computation_node.c__ : file containing the C code of a computation node
- __/sensor node__ : contains all files relative to the sensor nodes
	- __Makefile__ : file needed to compile sensor.c
	- __sensor.c__ : file containing the C code of a sensor node
- __/server__ : contains all files relative to the server
	- __server.py__ : file containing the Python code of the server

## Requirements
- Contiki 3.x 
- Cooja
- Python 3.x

## How to test
1. Create a new simulation in Cooja (to lauch Cooja : "contiki/tools/cooja ant run")
2. Add a new Z1 mote with border.c code, compile and create one
3. On this node, perform a right click -> Mote tools -> Serial Socket (SERVER), put "60001" as "Listen port" and click "start"
4. Add a new Z1 mote with computation_node.c code, compile and create some (2 used for test purposes)
5. Add a new Z1 mote with sensor.c code, compile and create many (13 used for test purposes)
6. Place the nodes where you want (in the range of the radio transmission of another node)
7. Activate the view options that you want (recommended : Mote IDs, LEDs, Radio traffic, Radio environment)
8. Inside a new command prompt, in the __/server__ directory, enter "python server.py"
9. Start the simulation in Cooja

You can now communicate with the network by writing in the command prompt and look at the behaviour (LED, radio signals and outputs) of the nodes in the Cooja simulation.


