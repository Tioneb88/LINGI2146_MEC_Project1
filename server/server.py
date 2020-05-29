import socket


HOST = '127.0.0.1'
PORT = 60001
TRESHOLD = 20

# create and connect socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# dict of lists to store the values of the nodes
nodes = dict()
for i in range(100):
	nodes[i] = list()

# compute slope and check if the valve must be open
def compute_slope(node):
	slope = (sum(nodes[node])) / len(nodes[node])
	if (slope >= TRESHOLD):
		return "OPENING_VALVE"
	return "OCLOSING_VALVE"

# process the received messages and acts according to the message
def process(message):
	message = message.split()
	if (message[0] == "SENSOR_INFO"):
		temp = int(message[3])
		addr0 = int(message[1])
		addr1 = int(message[2])
		nodes[addr0].append(temp)
		nodes[addr0] = nodes[addr0][-30:]

		print("Sensor data: " + str(temp) + " from node : " + str(addr0) + "." + str(addr1))
		print("Last values for this sensor node : " + nodes[addr0])

		result = compute_slope(addr0)
		if result == "OPENING_VALVE" :
			answer = "OPENING " + str(addr0) + " " + str(addr1) + " " + str(result) + "\n"
		else :
			answer = "OCLOSING " + str(addr0) + " " + str(addr1) + " " + str(result) + "\n"
		return answer
	else:
		return "NONE"

# reads the received messages, decode them and answers if necessary
while True:
	message = ""
	char = sock.recv(1).decode()
	while char != '\n':
		message = message + char
		char = sock.recv(1).decode()
	answer = process(message)
	if (answer != "NONE"):
		sock.sendall(str(answer))
