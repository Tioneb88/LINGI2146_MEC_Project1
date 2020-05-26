import socket

########################################################################################################################################################################
########################################## variables ###################################################################################################################
########################################################################################################################################################################

TRESHOLD = 30
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 60004  # The port used by the server
sensors = {k: [] for k in range(50)} # I suppose the server will not have to handle more than 50 sensors

########################################################################################################################################################################
######################################### helpers ######################################################################################################################
########################################################################################################################################################################
def process(pack):
	dpack = pack.split() # dpack=[Type, adr.u8[0], adr.u8[1], measure] example: ["SENSOR_VALUE", 3, 0, 5] = sensor value from node 3.0 with value 5
	msgtype = dpack[0]
	if (msgtype == "SENSOR_VALUE"):
		dst0 = int(dpack[1])
		dst1 = int(dpack[2])
		val = int(dpack[3])
		print("Sensor data: "+ str(val) +" from node " + str(dst0) + "." + str(dst1)) 
		sensors[dst0].append(val) #add the new value to the concerned sensor list of values
		sensors[dst0] = sensors[dst0][-30:] #only take the last 30 values. Olders are discarded
		print("Last values for this sensor :")
		print(sensors[dst0])
		action = compute(dst0) #compute the mean
		answer = "ACTION " + str(dst0) + " " + str(dst1) + " " + str(action) + " \n" #create a message to send to the boarder-node, using same message construction
		return answer 
	else:
		return "none"


def compute(dest): #mean computation and decide the action to do
	values = sensors[dest]
	length = len(sensors[dest])
	mean = (sum(values)) / length 
	if (mean >= TRESHOLD):
		return "CLOSE"
	return "OPEN" #mean < TRESHOLD

########################################################################################################################################################################
######################################### Main body ####################################################################################################################
########################################################################################################################################################################


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #socket creation
s.connect((HOST, PORT)) #socket connection


while True: #loop on every new message
	msg = ""
	currChar = s.recv(1).decode()
	while currChar != '\n': #decode untill end of line
		msg = msg + currChar
		currChar = s.recv(1).decode()
	answer = process(msg) #decode the entire message and compute mean
	if (answer != "none"):
		s.sendall(str(answer)) #send answer to boarder-node
