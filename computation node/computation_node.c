#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>
#include <limits.h>
#include "sys/ctimer.h"
#include "sys/timer.h"
#include "cc2420.h"
#include "cc2420_const.h"
#include <math.h>
#include <stdlib.h>

//DEFINE
#define seconds_before_routing_info 120 //TODO change to something like 120 after testing
#define seconds_before_flushing_routing_info 45 //TODO change to something like 300 after testing
#define seconds_before_calculating_measurement 60 //TODO change to something like 60 after testing
#define MAX_RETRANSMISSIONS 4
/**************************************************************DEFINE *******************************************************************
******************************************************************* ********************************************************************/
#define NBR_SENSOR_COMPUTED 5
#define THRESHOLD 50
#define FALSE 0
#define TRUE !(FALSE)

/****************************************************************** ********************************************************************
******************************************************************* ********************************************************************/

//STRUCT DEF
/**
*	senderAdr: address from the sensor sending datasize
*	destAddr: usually the border node (or sensor if open/close command)
*	status: contain the temperature value
*	rank: contain the rank of the sensor (not used for now)
*	option: used to define the content of the message
*	datasize: size of the data
*	Data: contain the data of the message eg: children list
**/
struct Runicast
{
	linkaddr_t senderAdr;
	linkaddr_t destAddr;
	short status;
	short rank;
	short option;
	int datasize;
	void *Data;
};
typedef struct Runicast runicast_struct;

struct Broadcast
{
	short rank;
	linkaddr_t srcAdr;
};
typedef struct Broadcast broadcast_struct;

struct Parent //structure in order to have the signal strength and the address (need to refresh rss from time to time)
{
	int rssi;
	linkaddr_t parent_adr;
};
typedef struct Parent parent_struct;

struct Children
{
	linkaddr_t address;
	linkaddr_t next_hop;
	clock_time_t last_update;
};
typedef struct Children Children;


//static variable
static short _rank;
static parent_struct *_parent;
static linkaddr_t parent_addr;
static struct ctimer broadcast_ctimer;
static struct ctimer compute_ctimer;
static Children *children_list[100]; //to be replaced by a contiki list

static struct broadcast_conn broadcast;
static struct runicast_conn uc;


/*---------------------------------------------------------------------------*/
PROCESS(computation_node_process, "computing");
AUTOSTART_PROCESSES(&computation_node_process);
/*---------------------------------------------------------------------------*/

/****************************************************************** ********************************************************************
******************************************************************* ********************************************************************/
static int last_measurements [NBR_SENSOR_COMPUTED][3]; //[index][u8[0]][u8[1]][last measurement]
static int number_of_measurements = 0; //this is usefull at the begining, before 30 nodes have sent measurements, is cap at NBR_SENSOR_COMPUTED
static int sum;
/****************************************************************** ********************************************************************
******************************************************************* ********************************************************************/

//merge the node's children list with the list received by children
static void merged_list(Children *received_List){
	int i;
	for(i = 0; i<100; i++){
		if(children_list[i] == NULL){
			*children_list[i] = received_List[i];
		}
	}
}

//find the next hop in the routing table (the list)
static void find_next_hop(linkaddr_t *next_hop, linkaddr_t address){
	 next_hop = &((children_list[address.u8[0]])->next_hop);
}

//update the timing of the last received message to flush unactive children
static void update_timing_children(linkaddr_t address){
	(children_list[address.u8[0]])->last_update = clock_time();
}

//send message to children saying that the border node is no more accessible by the parent /!\ send only to direct children
static void flush_children(){
	printf("flushing\n");
	runicast_struct message;
	message.senderAdr.u8[0] = linkaddr_node_addr.u8[0];
	message.senderAdr.u8[1] = linkaddr_node_addr.u8[1];
	message.rank = _rank;
	message.option = 5;
	message.datasize = 0;
	message.Data = NULL;
	int i;	
	for(i = 0; i<100; i++){
		//comparison between next hop address and children address to send only to direct children
		if(linkaddr_cmp(&((children_list[i])->address), &((children_list[i])->next_hop))){
			message.destAddr = (children_list[i])->address;
			packetbuf_copyfrom(&message, sizeof(runicast_struct));
			printf("delete children\n");
			runicast_send(&uc, &((children_list[i])->address), MAX_RETRANSMISSIONS);
			children_list[i] = NULL;
		}
	}
}

/**
* //TODO remove the broadcast struct and just use a pointer to the rank
* broadcast_recv is called when a routing information from a neighbouring node is recieved
* it will decide if the node sending the broadcast would be a better suited parent that the current one 
* Parameters: 
* struct broadcast_conn *c : a pointer to the structure containing the rank and the adress of the sending node
* const linkaddr_t *from : the adress of the sender 
**/
//TODO !!!!RSSI REMOVED FOR TEST PURPOSES
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	static signed char rss_val;
	static signed char rss_offset;
	if(linkaddr_node_addr.u8[0] == 1){
		//TODO remove once we have a real root node
		return;
	}
	broadcast_struct* arrival = packetbuf_dataptr();
		
	//rss_val = cc2420_last_rssi;
	//rss_offset=-45;
	//rss_val = rss_val + rss_offset;
	printf("Routing information recieved from: src %d.%d with rank %d rss value: %d\n", arrival->srcAdr.u8[0],arrival->srcAdr.u8[1] , arrival->rank, rss_val );
	//TODO change the +1 by the quality of communication or smth
	if(arrival->rank < _rank -1){// && rss_val > _parent->rssi){ //cant add 1 on the left because it would overflow, so substract one on right side (since always > 0)
		_rank = arrival->rank +1;
		parent_addr.u8[0] = (arrival->srcAdr).u8[0];
		parent_addr.u8[1] = (arrival->srcAdr).u8[1];
		//_parent->rssi = rss_val;
		printf("\t parent changed, new ranking : new rank: %d, parent address %d.%d \n",_rank, (parent_addr).u8[0],(arrival->srcAdr).u8[1]);
	}   

}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


static void recv_ruc(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	static signed char rss_val;
	static signed char rss_offset;
	runicast_struct* arrival = packetbuf_dataptr();
	
	printf("unicast message recieved from %d.%d | originating from %d.%d | value %d \n",from->u8[0], from->u8[1],arrival->senderAdr.u8[0],arrival->senderAdr.u8[1],arrival->status);
	//FOR debugging, root node don't transmit received messages
	if(linkaddr_node_addr.u8[0] == 1){
		return; //TODO remove, for testing only
	}
	//If the sender is not the parent and the dest is not the current node, it's a children then the message up
	if(!linkaddr_cmp(&parent_addr, from) && !linkaddr_cmp(&(arrival->destAddr),&linkaddr_node_addr)) {
		//if children is not yet in the list we add it
		if(children_list[(arrival->senderAdr).u8[0]] == NULL){
			printf("children add\n");
			Children received;
			received.address.u8[0]= (arrival->senderAdr).u8[0];
			received.address.u8[1]= (arrival->senderAdr).u8[1];
			received.next_hop.u8[0]= from->u8[0];
			received.next_hop.u8[1]= from->u8[1];
			//printf("address %d.%d next hop %d.%d\n",((&received)->address).u8[0], ((&received)->address).u8[1], ((&received)->next_hop).u8[0], ((&received)->next_hop).u8[1]);
			children_list[(arrival->senderAdr).u8[0]] = &received;
		}
		update_timing_children(*from);
		/*while (runicast_is_transmitting(&uc)){
			clock_wait(100);
			printf("wait to parent\n");
		}*/
		
		
		/****************************************************************** ********************************************************************
		******************************************************************* ********************************************************************/
		
		if(add_to_compute_table(arrival)){
			printf("%d.%d arrival = %d \n",arrival->senderAdr.u8[0], arrival->senderAdr.u8[1],arrival->status);
			int i;
			for(i=0;i<number_of_measurements;i++){
				printf("|%d: %d",last_measurements[i][0],last_measurements[i][2]);
			}
			printf("\n");
		}else{
			printf("forwarding to server, too much to compute");
			packetbuf_copyfrom( arrival ,sizeof(runicast_struct));
			runicast_send(&uc, &parent_addr, MAX_RETRANSMISSIONS);
		}
		
		/****************************************************************** ********************************************************************
		******************************************************************* ********************************************************************/
		
	}
	//Message going down, if the dest is not the current node, the message must be transmit to the next hop children.
	else if(!linkaddr_cmp(&(arrival->destAddr), &linkaddr_node_addr)){
		printf("retransmission\n");
		//rss_val = cc2420_last_rssi; TODO understand why this mess with the timers
		//rss_offset=-45;
		//_parent->rssi = rss_val + rss_offset;
		linkaddr_t *next = NULL;
		find_next_hop(next, arrival->destAddr);
		arrival->rank = _rank;
		
		if(next != NULL){
			/*while (runicast_is_transmitting(&uc)){
				clock_wait(100);
				printf("wait retransmit\n");
			}*/
			packetbuf_copyfrom(arrival, sizeof(runicast_struct));
			runicast_send(&uc, next, MAX_RETRANSMISSIONS);
		}
	}
	//the current node is the destination 
	//TODO add option like open/close valve,...
	else{
		if(arrival->option == 3){ //list merging
			Children *received_List = (Children *) arrival->Data;
			merged_list(received_List);
			
		}
		if(arrival->option == 5){//flush after parent has been disconnect
			printf("parent lost, flushing children\n");
			_rank = SHRT_MAX;
			_parent = NULL;
			_parent->rssi= -SHRT_MAX;
			flush_children();
		}
	}
}


static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
  printf("runicast message sent to %d.%d, retransmissions %d\n",
	 to->u8[0], to->u8[1], retransmissions);
}
static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
  printf("runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
	 //if parent is not reachable, we forget it
	 if(linkaddr_cmp(to, &parent_addr)){
		printf("parent lost\n");
		_rank = SHRT_MAX;
		_parent->rssi= -SHRT_MAX;
		flush_children();
	 }
	 //in case children is no more reachable
	 else{
		children_list[to->u8[0]] = NULL;
	 }
}

static const struct runicast_callbacks runicast_callbacks = {recv_ruc, sent_runicast, timedout_runicast};

void broadcast_timeout()
{	
	ctimer_reset(&broadcast_ctimer);
	if(_rank == SHRT_MAX){
		printf("Not broadcasting info as no parent yet registered\n");
		return;
	} //if node has just booted or flushed we don't want it to propagate useless or possibly wrongs info 
	printf("broadcasting routing info rank %d \n", _rank);
	broadcast_struct message;
	message.rank = _rank; 
	message.srcAdr.u8[0] = linkaddr_node_addr.u8[0]; 
	message.srcAdr.u8[1] = linkaddr_node_addr.u8[1];
	packetbuf_copyfrom(&message ,sizeof(message)); 
	broadcast_send(&broadcast);
}


/****************************************************************** ********************************************************************
******************************************************************* ********************************************************************/

/**
* add_to_compute_table add the node to it's proccessing table if enough room or update it if already there 
**/
int add_to_compute_table(runicast_struct* arrival){
	int status = arrival->status;
	int zero = arrival->senderAdr.u8[0];
	int one = arrival->senderAdr.u8[1];
	int i;
	for(i=0;i<number_of_measurements;i++){
		if(last_measurements[i][0]==zero && last_measurements[i][1]==one){
			last_measurements[i][2]=status;
			return TRUE;
		}
	}
    if(number_of_measurements<NBR_SENSOR_COMPUTED){
		last_measurements[number_of_measurements][0]=zero;
		last_measurements[number_of_measurements][1]=one;
		last_measurements[number_of_measurements][2]=status;
        number_of_measurements++;
		return TRUE;
    }else{
		return FALSE;
	}
}


double mean_measurements(){
    sum=0;
	int i;
	for(i=0;i<number_of_measurements;i++){
        sum += last_measurements[i][2];
	}
	printf("sum %d\n",sum);
	printf("number of measurement %d\n",number_of_measurements);
	printf("mean: %d\n", sum/number_of_measurements);
    return sum/number_of_measurements;
}


/**
* Compute the mean of the value used for y axis
* since those values are 1,2,3... the mean can easily be calculated by computing the triangular number
**/
double mean_y(){
    int nombre_triangulaire = (number_of_measurements*(number_of_measurements+1))/2;
	printf("mean y: %d\n", nombre_triangulaire/number_of_measurements);
    return nombre_triangulaire/number_of_measurements;
}

/**
* sum of (x - x_mean) * (y - y_mean)
**/
double sum_product(double average, double average_y){
    sum = 0;
	int i;
	for(i=0;i<number_of_measurements;i++){
		sum += (last_measurements[i][2]-average) * (i-average_y);
	}
	printf("sum_product: %d\n", sum);
    return sum;
}

/**
* sum of (x - x_mean)²
**/
double sum_of_squares(double average){
	sum = 0;
	int i;
	for(i=0;i<number_of_measurements;i++){
		sum += pow((last_measurements[i][2]-average),2);
	}
    if(sum==0)return 1;
	printf("sum_of squares: %d\n", sum);
    return sum;
}

/**
* the slope = sum of product / sum of squares
* the data are stored in reverse order in a table (last entry is at index 0),
*   this causes me to compute the slope in the reverse sense
*   I thus has to reverse the result of my computation by adding a minus
**/
int compute_least_square_slope(){
   int avg = mean_measurements();
   int avg_y = mean_y();
   //- is required because I compute in reverse order to not mess with pointers
   return (-sum_product(avg,avg_y) / sum_of_squares(avg))*1000;
}

void compute_measurement_timeout()
{
	ctimer_reset(&compute_ctimer);
	if(number_of_measurements<3){
		return;
	}
	int slope = compute_least_square_slope();
	printf("slope %d \n", slope );
	
	runicast_struct message;
	message.senderAdr.u8[0] = linkaddr_node_addr.u8[0];
	message.senderAdr.u8[1] = linkaddr_node_addr.u8[1];
	
	if(slope > THRESHOLD){
        printf("sending messages to open the valve:\n");
		message.option = 1;
    }else{
         printf("sending messages to close the valve: \n");
		message.option = 2;
    }
	
	int i;
	for (i=0;i<number_of_measurements;i++){
		message.destAddr.u8[0] = last_measurements[i][1]; 
		message.destAddr.u8[1] = last_measurements[i][2];
		
		linkaddr_t *next = NULL;
		find_next_hop(next, message.destAddr);
		message.rank = _rank; //TODO not sure? do i need to send it
		
		if(next != NULL){
			packetbuf_copyfrom(&message, sizeof(runicast_struct));
			runicast_send(&uc, next, MAX_RETRANSMISSIONS);
			printf("option message to %d.%d\n",last_measurements[i][0],last_measurements[i][1]);
		}else{
			printf("no known path to send the option message to %d.%d\n",last_measurements[i][0],last_measurements[i][1] );
		}
	}
	
	
		
}


/****************************************************************** ********************************************************************
******************************************************************* ********************************************************************/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(computation_node_process, ev, data)
{
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_EXITHANDLER(runicast_close(&uc);)
	PROCESS_BEGIN();
	random_init(linkaddr_node_addr.u8[0]); //to use a different seed per node to generate random measurement, I use their IP address
	broadcast_open(&broadcast, 129, &broadcast_call);
	runicast_open(&uc, 144, &runicast_callbacks);
	_rank = SHRT_MAX;
	//_parent->rssi= -SHRT_MAX;
	if(linkaddr_node_addr.u8[0]==1){_rank = 0; printf("fake root\n");} //TODO remove, just for testing without border router
	broadcast_timeout();//broadcast as soon as the node starts 
	ctimer_set(&broadcast_ctimer, seconds_before_routing_info * CLOCK_SECOND, broadcast_timeout, NULL);
	if(linkaddr_node_addr.u8[0]!=1){
		ctimer_set(&compute_ctimer, seconds_before_calculating_measurement * CLOCK_SECOND, compute_measurement_timeout, NULL);
	}
	PROCESS_YIELD();

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
