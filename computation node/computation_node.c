/*
	LINGI2146 Mobile and Embedded Computing : Project1
	Author : Benoît Michel
	Date : May 2020
*/
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
#include "lib/list.h"
#include "lib/memb.h"
#include <stdbool.h>



//DEFINE
#define seconds_before_routing_info 120 //TODO change to something like 120 after testing
#define seconds_before_sending_measurement 60 //TODO change to something like 60 after testing
#define seconds_before_calculating_measurement 60
#define MAX_VALUES_PER_SENSOR 30
#define MAX_RETRANSMISSIONS 10
#define NUM_HISTORY_ENTRIES 10
#define MAX_CHILDREN 128
#define NBR_SENSOR_COMPUTED 2
#define THRESHOLD 10
#define FALSE 0
#define TRUE !(FALSE)

//STRUCT DEF
/**
*	senderAdr: address from the sensor sending datasize
*	destAddr: usually the border node (or sensor if open/close command)
*	status: contain the temperature value
*	rank: contain the rank of the sensor (not used for now)
*	option: used to define the content of the message
**/
struct Runicast
{
	linkaddr_t senderAdr;
	linkaddr_t destAddr;
	short status;
	short valve_status;
	short rank;
	uint8_t option;
	linkaddr_t child_lost;
};
typedef struct Runicast runicast_struct;

enum{
	SENSOR_INFO,
	OPENING_VALVE,
	FLUSH_CHILDREN,
	CHILDREN_LOST
};

/*
* Detects duplicate callbacks at receiving nodes.
* Duplicates appear when ack messages are lost. */
struct history_entry {
	struct history_entry *next;
	linkaddr_t addr;
	uint8_t seq;
};
LIST(history_table);
MEMB(history_mem, struct history_entry, NUM_HISTORY_ENTRIES);

struct Broadcast
{
	short rank;
	linkaddr_t srcAdr;
	uint8_t option;
};
typedef struct Broadcast broadcast_struct;
enum {
	BROADCAST_INFO,
	BROADCAST_REQUEST
};

struct Children
{
	struct Children *next;
	linkaddr_t address;
	linkaddr_t next_hop;
	clock_time_t last_update;
};

struct compute_children{
	struct compute_children *next;
	linkaddr_t address;
	linkaddr_t next_hop;
	uint8_t nbrValue;
	int sensorValue[MAX_VALUES_PER_SENSOR];
	int slope;
};

//static variable
static short _rank;
static linkaddr_t parent_addr;
static int rssi_parent;

static int sum;


MEMB(children_memb, struct Children, MAX_CHILDREN);
LIST(children_list);

MEMB(computation_children_memb, struct compute_children, NBR_SENSOR_COMPUTED);
LIST(computation_list);

static struct broadcast_conn broadcast;
static struct runicast_conn ruc;


/*---------------------------------------------------------------------------*/
PROCESS(runicast_process, "computation runicast");
PROCESS(broadcast_routing, "broadcast routing");
AUTOSTART_PROCESSES(&runicast_process, &broadcast_routing);
/*---------------------------------------------------------------------------*/



/**
* add_to_compute_table add the node to it's proccessing table if enough room or update it if already there
**/
int add_to_compute_table(runicast_struct* arrival, linkaddr_t *from){
	struct compute_children *n;

	for(n = list_head(computation_list); n != NULL; n = list_item_next(n)){
		//printf("node list %d.%d, arrival %d.%d\n", n->address.u8[0], n->address.u8[1], arrival->senderAdr.u8[0], arrival->senderAdr.u8[1]);
		if(linkaddr_cmp(&arrival->senderAdr, &n->address)){
			//printf("node already in table %d.%d\n", n->address.u8[0], n->address.u8[1]);
			(n->sensorValue)[(n->nbrValue)%MAX_VALUES_PER_SENSOR] = arrival->status;
			(n->nbrValue)++;
			return TRUE;
		}

	}
	//printf("node not in list address %d.%d\n", arrival->senderAdr.u8[0],arrival->senderAdr.u8[1]);
	if(n == NULL && list_length(computation_list)<NBR_SENSOR_COMPUTED){
		n = memb_alloc(&computation_children_memb);
		linkaddr_copy(&n->address, &arrival->senderAdr);
		n->nbrValue = 1;
		(n->sensorValue)[0] = arrival->status;
		n->slope = 0;
		linkaddr_copy(&n->next_hop,from);
		list_add(computation_list, n);
		printf("add to the table %d.%d\n", n->address.u8[0],n->address.u8[1]);
		return TRUE;
	}else{
		return FALSE;
	}
}


double mean_measurements(struct compute_children *n){
	sum=0;
	int i;
	for(i=0;i<n->nbrValue;i++){
		sum += (n->sensorValue)[i];
	}

	//printf("sum %d\n",sum);
	//printf("number of measurement %d\n",n->nbrValue);
	//printf("mean: %d\n", sum/n->nbrValue);
	return sum/n->nbrValue;

}


/**
* Compute the mean of the value used for y axis
* since those values are 1,2,3... the mean can easily be calculated by computing the triangular number
**/

double mean_y(struct compute_children *n){
	int nombre_triangulaire = ((n->nbrValue)*(n->nbrValue+1))/2;
	printf("mean y: %d\n", nombre_triangulaire/n->nbrValue);
	return nombre_triangulaire/n->nbrValue;
}

/**
* sum of (x - x_mean) * (y - y_mean)
**/
double sum_product(double average, double average_y, struct compute_children *n){
	sum = 0;
	int i;
	for(i=0;i<n->nbrValue;i++){
		sum += ((n->sensorValue)[i]-average) * (i-average_y);
	}
	//printf("sum_product: %d\n", sum);
	return sum;
}

/**
* sum of (x - x_mean)²
**/
double sum_of_squares(double average, struct compute_children *n){
	sum = 0;
	int i;
	for(i=0;i<n->nbrValue;i++){
		sum += pow(((n->sensorValue)[i]-average),2);
	}
	if(sum==0)return 1;
	//printf("sum_of squares: %d\n", sum);
	return sum;
}

/**
* the slope = sum of product / sum of squares
* the data are stored in reverse order in a table (last entry is at index 0),
*   this causes me to compute the slope in the reverse sense
*   I thus has to reverse the result of my computation by adding a minus
**/
void compute_least_square_slope(){
	struct compute_children *n;
	for(n = list_head(computation_list); n != NULL; n = list_item_next(n)){
		//if(n->nbrValue == 30){
		if(n->nbrValue >= 3){
			int avg = mean_measurements(n);
			int avg_y = mean_y(n);
			//- is required because I compute in reverse order to not mess with pointers
			int slope = (-sum_product(avg, avg_y, n) / sum_of_squares(avg, n))*1000;
			n->slope = slope;
			printf("slope: %d \n",slope);
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
	broadcast_struct* arrival = packetbuf_dataptr();
	if(arrival->option == BROADCAST_INFO ) { // && linkaddr_node_addr.u8[0] != 1){	 //modified
		rss_val = cc2420_last_rssi;
		rss_offset=-45;
		rss_val = rss_val + rss_offset;
		//printf("Routing information recieved from: src %d.%d with rank %d rss value: %d\n", arrival->srcAdr.u8[0],arrival->srcAdr.u8[1] , arrival->rank, rss_val );
		//TODO change the +1 by the quality of communication or smth
		if(arrival->rank < _rank -1 && rss_val > rssi_parent){ //cant add 1 on the left because it would overflow, so substract one on right side (since always > 0)
			_rank = arrival->rank +1;
			parent_addr.u8[0] = (arrival->srcAdr).u8[0];
			parent_addr.u8[1] = (arrival->srcAdr).u8[1];
			rssi_parent = rss_val;
			printf("\t parent changed, new ranking : new rank: %d, parent address %d.%d \n",_rank, (parent_addr).u8[0],(arrival->srcAdr).u8[1]);
		}
	}
	else if(arrival->option == BROADCAST_REQUEST && _rank != SHRT_MAX){
		//printf("Sending routing info via broadcast rank: %d\n", _rank);
		broadcast_struct message;
		message.rank = _rank;
		message.srcAdr.u8[0] = linkaddr_node_addr.u8[0];
		message.srcAdr.u8[1] = linkaddr_node_addr.u8[1];
		packetbuf_copyfrom(&message ,sizeof(message));
		broadcast_send(&broadcast);
	}
	else{
		return;
	}
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


static void recv_ruc(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	/* OPTIONAL: Sender history */
	struct history_entry *e = NULL;
	for(e = list_head(history_table); e != NULL; e = e->next) {
		if(linkaddr_cmp(&e->addr, from)) {
			break;
		}
	}
	if(e == NULL) {
		/* Create new history entry */
		e = memb_alloc(&history_mem);
		if(e == NULL) {
			e = list_chop(history_table); /* Remove oldest at full history */
		}
		linkaddr_copy(&e->addr, from);
		e->seq = seqno;
		list_push(history_table, e);
	} else {
		/* Detect duplicate callback */
		if(e->seq == seqno) {
			//printf("runicast message received from %d.%d, seqno %d (DUPLICATE)\n", from->u8[0], from->u8[1], seqno);
			return;
		}
		/* Update existing history entry */
		e->seq = seqno;
	}
	static signed char rss_val;
	static signed char rss_offset;
	runicast_struct* arrival = packetbuf_dataptr();

	printf("runicast message recieved from %d.%d | originating from %d.%d | value %d \n",from->u8[0], from->u8[1],arrival->senderAdr.u8[0],arrival->senderAdr.u8[1],arrival->status);
	/*
	//FOR debugging, root node don't transmit received messages
	if(linkaddr_node_addr.u8[0] == 1){
		return; //TODO remove, for testing only
	}
*/
	if(arrival->option == SENSOR_INFO){

		if(add_to_compute_table(arrival, from)){
			struct compute_children *n;
			for(n = list_head(computation_list); n != NULL; n = list_item_next(n)){
				if(linkaddr_cmp(&n->address, &arrival->senderAdr)){
					printf("%d.%d arrival = %d , value number: %d, slope: %d\n",arrival->senderAdr.u8[0], arrival->senderAdr.u8[1],arrival->status, n->nbrValue, n->slope);
					if(arrival->valve_status==1){
						//valve is already open so do nothing
						printf("the valve of %d.%d is open so do nothing\n", arrival->senderAdr.u8[0], arrival->senderAdr.u8[1]);
					}else{
						//valve is close so let's open it if threshold is reached
						if(abs(n->slope) > THRESHOLD){
							runicast_struct message;
							linkaddr_copy(&(&message)->senderAdr, &linkaddr_node_addr);
							linkaddr_copy(&(&message)->destAddr, &n->address);
							message.option = OPENING_VALVE;
							printf("sending messages to open the valve:\n");
							packetbuf_copyfrom(&message, sizeof(runicast_struct));
							runicast_send(&ruc, &n->next_hop, MAX_RETRANSMISSIONS);
						}
					}
					printf("No need to open the valve, no message to send \n");
					break;
				}
			}

		}
		else{
			printf("forwarding to server via my parent %d.%d, too much to compute\n", parent_addr.u8[0], parent_addr.u8[1] );
			linkaddr_copy(&arrival->destAddr, &parent_addr);
			packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
			runicast_send(&ruc, &parent_addr, MAX_RETRANSMISSIONS);
		}
		struct Children *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &arrival->senderAdr)) {
				break;
			}
		}
		if(n == NULL){//children not found
			n = memb_alloc(&children_memb);

			if(n == NULL){
				return;
			}
			linkaddr_copy(&n->address, &arrival->senderAdr);
			linkaddr_copy(&n->next_hop, from);
			list_add(children_list, n);
		}
	}
	else if(arrival->option == OPENING_VALVE){
		rss_val = cc2420_last_rssi;
		rss_offset=-45;
		rss_val = rss_val + rss_offset;
		if(linkaddr_cmp(&arrival->destAddr, &linkaddr_node_addr)){
			printf("Opening valve\n");
		}
		else{
			struct Children *n;
			bool found = false;
			for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
				if(linkaddr_cmp(&n->address, &arrival->destAddr)) {
					found = true;
					break;
				}
			}


			if(found==true){
				packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
				printf("transmitting a message to %d.%d via nexthop %d.%d \n", arrival->destAddr.u8[0], arrival->destAddr.u8[1], n->next_hop.u8[0], n->next_hop.u8[1] );
				runicast_send(&ruc, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			else{ //don't know where to send it, -> send to parent
				runicast_send(&ruc, &parent_addr, MAX_RETRANSMISSIONS);
			}
		}
	}
	/*else if(arrival->option == CLOSING_VALVE){
		if(linkaddr_cmp(&arrival->destAddr, &linkaddr_node_addr)){
			printf("Closing valve\n");
		}
		else{
			struct Children *n;
			bool found = false;
			for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
				if(linkaddr_cmp(&n->address, &arrival->destAddr)) {
					found=true;
					break;
				}
			}

			if(found==true){
				packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
				printf("transmitting a message to %d.%d via nexthop %d.%d \n", arrival->destAddr.u8[0], arrival->destAddr.u8[1], n->next_hop.u8[0], n->next_hop.u8[1] );
				runicast_send(&ruc, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			else{ //don't know where to send it, -> send to parent
				runicast_send(&ruc, &parent_addr, MAX_RETRANSMISSIONS);
			}
		}
	}*/
	else if(arrival->option == FLUSH_CHILDREN){
		printf("flushing info received\n");
		printf("parent lost\n");
		_rank = SHRT_MAX;
		rssi_parent= -SHRT_MAX;
		struct Children *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &n->next_hop)){
				arrival->destAddr = n->address;
				packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
				runicast_send(&ruc, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, n);
		}
	}
	else if(arrival->option == CHILDREN_LOST){
		struct Children *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &arrival->child_lost)){
				list_remove(children_list, n);
				break;
			}

		}
		packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
		runicast_send(&ruc, &parent_addr, MAX_RETRANSMISSIONS);
	}
}


static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	printf("runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	printf("runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
	//if parent is not reachable, we forget it
	if(linkaddr_cmp(to, &parent_addr)){
		printf("parent lost\n");
		_rank = SHRT_MAX;
		rssi_parent= -SHRT_MAX;
		struct Children *n;
		runicast_struct flush_message;
		linkaddr_copy(&(&flush_message)->senderAdr, &linkaddr_node_addr);
		(&flush_message)->option = FLUSH_CHILDREN;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &n->next_hop)){
				(&flush_message)->destAddr = n->address;
				packetbuf_copyfrom(&flush_message ,sizeof(runicast_struct));
				runicast_send(&ruc, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, n);
		}
	}
	//in case children is no more reachable
	else{
		struct Children *n;
		bool found = false;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, to)){
				list_remove(children_list, n);
				found = true;
				break;
			}
		}
		if (found==true) {
			runicast_struct lost_msg;
			linkaddr_copy(&(&lost_msg)->senderAdr, &linkaddr_node_addr);
			(&lost_msg)->option = CHILDREN_LOST;
			packetbuf_copyfrom(&lost_msg ,sizeof(runicast_struct));
			runicast_send(&ruc, &n->next_hop, MAX_RETRANSMISSIONS);
		}
	}
}

static const struct runicast_callbacks runicast_callbacks = {recv_ruc, sent_runicast, timedout_runicast};


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(runicast_process, ev, data){
	PROCESS_EXITHANDLER(runicast_close(&ruc);)
	PROCESS_BEGIN();

	runicast_open(&ruc, 144, &runicast_callbacks);
	while(1){
		static struct etimer et;
		etimer_set(&et, CLOCK_SECOND * seconds_before_routing_info);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		//printf("computing \n");
		compute_least_square_slope();
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_routing, ev, data){
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	broadcast_open(&broadcast, 129, &broadcast_call);

	_rank = SHRT_MAX;
	rssi_parent= -SHRT_MAX;

	//if(linkaddr_node_addr.u8[0]==1){_rank = 0; printf("fake root\n");} //TODO remove, just for testing without border router
	while(1) {
		static struct etimer et;
		/* Send routing informations via broadcast every 60 secondes */
		etimer_set(&et, CLOCK_SECOND * seconds_before_routing_info);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		if(_rank != SHRT_MAX){
			//printf("Sending routing info via broadcast rank: %d\n", _rank);
			broadcast_struct message;
			message.rank = _rank;
			message.option = BROADCAST_INFO; //modified
			message.srcAdr.u8[0] = linkaddr_node_addr.u8[0];
			message.srcAdr.u8[1] = linkaddr_node_addr.u8[1];
			packetbuf_copyfrom(&message ,sizeof(message));
			broadcast_send(&broadcast);
		}
		else{
			printf("Not sending routing info, not connected to the network\n");
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
