/*
	LINGI2146 Mobile and Embedded Computing : Project1
	Author : Benoît Michel
	Date : May 2020
*/
#include "contiki.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "net/rime/rime.h"
#include "sys/timer.h"
#include "sys/ctimer.h"
#include "cc2420.h"
#include "cc2420_const.h"

#include <stdio.h>
#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define TRUE 1
#define FALSE 0
#define MAX_HISTORY 10
#define MAX_CHILDREN 100
#define ROUTING_INTERVAL 120
#define MAX_RETRANSMISSIONS 10
#define COMPUTING_INTERVAL 60
#define MAX_VALUES_BY_SENSOR 30

#define NBR_SENSOR_COMPUTED 2
#define THRESHOLD 10


// Structures definition
typedef struct Broadcast broadcast_struct;
struct Broadcast {
	short rank;                             // rank of the node
	linkaddr_t sendAddr;                    // address of the node sending the message
	uint8_t option;                         // type of message
};

typedef struct Runicast runicast_struct;
struct Runicast {
	short rank;                             // rank of the node
	short temp;                             // temperature value read by the sensor
	short valve_status;                     // current state of the valve : closed(0) or open(1)
	linkaddr_t sendAddr;                    // address of the node sending the message
	linkaddr_t destAddr;                    // address of the node targeted to receive the message
	linkaddr_t child_lost;                  // if child is lost
	uint8_t option;                         // type of message
};

typedef struct Children children_struct;
struct Children {
	linkaddr_t address;                     // address of the child
	linkaddr_t next_hop;                    // nexthop
	clock_time_t last_update;               // last update of the children
	children_struct *next;                  // next child
};

typedef struct History history_struct;
struct History {
	uint8_t seq;                           // sequence number
	linkaddr_t addr;                       // address of the node
	history_struct *next;                  // next entry in the history
};

typedef struct Compute compute_struct;
struct Compute {
	int slope;                             // current slope
	uint8_t nbrValue;                      // number of sensor values
	linkaddr_t address;                    // address of the node
	linkaddr_t next_hop;                   // next_hop
	int sensorValue[MAX_VALUES_BY_SENSOR]; // the different sensor values
	compute_struct *next;                  // next computation structure
};


// Enumerations definition
enum {
	SENSOR_INFO,
	OPENING_VALVE,
	FLUSH_CHILDREN,
	CHILDREN_LOST
};

enum {
	BROADCAST_INFO,
	BROADCAST_REQUEST
};


// Memory blocks allocation
LIST(history_table);
MEMB(history_mem, history_struct, MAX_HISTORY);

LIST(children_list);
MEMB(children_memb, children_struct, MAX_CHILDREN);

LIST(computation_list);
MEMB(computation_children_memb, compute_struct, NBR_SENSOR_COMPUTED);


// Static variables definition
static int sum;
static int parent_rssi;
static short static_rank;
static linkaddr_t parent_addr;

// Static structures definition
static struct broadcast_conn broadcast;
static struct runicast_conn runicast;


/*---------------------------------------------------------------------------*/
PROCESS(computation_process_runicast, "[Computation node] +++ runicast execution");
PROCESS(computation_broadcast_routing, "[Computation node] +++ broadcast routing");
AUTOSTART_PROCESSES(&computation_process_runicast, &computation_broadcast_routing);
/*---------------------------------------------------------------------------*/



/*
	Addition of sensor nodes to the computation table
*/
int compute(runicast_struct* arrival, const linkaddr_t *from) {
	compute_struct *node;

	for(node = list_head(computation_list); node != NULL; node = list_item_next(node)) {
		if(linkaddr_cmp(&arrival->sendAddr, &node->address)) {
			(node->sensorValue)[(node->nbrValue) % MAX_VALUES_BY_SENSOR] = arrival->temp;
			(node->nbrValue)++;
			printf("[Computation node] Node already in table : %d.%d\n", node->address.u8[0], node->address.u8[1]);
			return 1;
		}
	}

	if(node == NULL && list_length(computation_list) < NBR_SENSOR_COMPUTED){
		node = memb_alloc(&computation_children_memb);
		linkaddr_copy(&node->address, &arrival->sendAddr);
		node->slope = 0;
		node->nbrValue = 1;
		(node->sensorValue)[0] = arrival->temp;
		linkaddr_copy(&node->next_hop, from);
		list_add(computation_list, node);
		printf("[Computation node] New node added to the table : %d.%d\n", node->address.u8[0], node->address.u8[1]);
		return 1;
	}
	else return 0;
}


/*
double mean_measurements(compute_struct *n){
	sum=0;
	int i;
	for(i=0;i<n->nbrValue;i++){
		sum += (n->sensorValue)[i];
	}
	return sum/n->nbrValue;
}



double mean_y(compute_struct *n){
	int nombre_triangulaire = ((n->nbrValue)*(n->nbrValue+1))/2;
	printf("mean y: %d\n", nombre_triangulaire/n->nbrValue);
	return nombre_triangulaire/n->nbrValue;
}


double sum_product(double average, double average_y, compute_struct *n){
	sum = 0;
	int i;
	for(i=0;i<n->nbrValue;i++){
		sum += ((n->sensorValue)[i]-average) * (i-average_y);
	}
	//printf("sum_product: %d\n", sum);
	return sum;
}


double sum_of_squares(double average, compute_struct *n){
	sum = 0;
	int i;
	for(i=0;i<n->nbrValue;i++){
		sum += pow(((n->sensorValue)[i]-average),2);
	}
	if(sum==0)return 1;
	//printf("sum_of squares: %d\n", sum);
	return sum;
}
*/

/*
	Computes the slope as the division of the sum of the product on the sum of the squares.
	For simplicity, the data is stored in reverse order (index 0 = last entry) => slope computed in reverse order
*/
void compute_least_square_slope() {
	compute_struct *node;
	for(node = list_head(computation_list); node != NULL; node = list_item_next(node)) {
		if(node->nbrValue >= 3) { // == 30
			sum = 0;
			int i;

			int avg_y = (((node->nbrValue)*(node->nbrValue+1))/2) / node->nbrValue;
			for(i=0; i<node->nbrValue; i++) {
				sum += (node->sensorValue)[i];
			}
			int avg = sum/node->nbrValue;

			int product_sum = 0;
			int squares_sum = 0;
			int j;
			for(j=0; j<node->nbrValue; j++) {
				product_sum += ((node->sensorValue)[i]-average) * (i-average_y);
				squares_sum += pow(((node->sensorValue)[i]-average), 2);
			}
			if(squares_sum==0) squares_sum++;

			node->slope = (-product_sum / squares_sum(avg, node)) * 1000;
		}
	}
}


/**
* broadcast_recv is called when a routing information from a neighbouring node is recieved
* it will decide if the node sending the broadcast would be a better suited parent that the current one
* Parameters:
* struct broadcast_conn *c : a pointer to the structure containing the rank and the adress of the sending node
* const linkaddr_t *from : the adress of the sender
**/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	static signed char rss_val;
	static signed char rss_offset;
	broadcast_struct* arrival = packetbuf_dataptr();
	if(arrival->option == BROADCAST_INFO ) { // && linkaddr_node_addr.u8[0] != 1){	 //modified
		rss_val = cc2420_last_rssi;
		rss_offset=-45;
		rss_val = rss_val + rss_offset;
		//printf("Routing information recieved from: src %d.%d with rank %d rss value: %d\n", arrival->sendAddr.u8[0],arrival->sendAddr.u8[1] , arrival->rank, rss_val );
		//TODO change the +1 by the quality of communication or smth
		if(arrival->rank < static_rank -1 && rss_val > parent_rssi){ //cant add 1 on the left because it would overflow, so substract one on right side (since always > 0)
			static_rank = arrival->rank +1;
			parent_addr.u8[0] = (arrival->sendAddr).u8[0];
			parent_addr.u8[1] = (arrival->sendAddr).u8[1];
			parent_rssi = rss_val;
			printf("\t parent changed, new ranking : new rank: %d, parent address %d.%d \n",static_rank, (parent_addr).u8[0],(arrival->sendAddr).u8[1]);
		}
	}
	else if(arrival->option == BROADCAST_REQUEST && static_rank != SHRT_MAX){
		//printf("Sending routing info via broadcast rank: %d\n", static_rank);
		broadcast_struct message;
		message.rank = static_rank;
		message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
		message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];
		packetbuf_copyfrom(&message ,sizeof(message));
		broadcast_send(&broadcast);
	}
	else{
		return;
	}
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	history_struct *e = NULL;
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

	printf("runicast message recieved from %d.%d | originating from %d.%d | value %d \n",from->u8[0], from->u8[1],arrival->sendAddr.u8[0],arrival->sendAddr.u8[1],arrival->temp);

	if(arrival->option == SENSOR_INFO) {

		if(compute(arrival, from)) {
			compute_struct *n;
			for(n = list_head(computation_list); n != NULL; n = list_item_next(n)){
				if(linkaddr_cmp(&n->address, &arrival->sendAddr)){
					printf("%d.%d arrival = %d , value number: %d, slope: %d\n",arrival->sendAddr.u8[0], arrival->sendAddr.u8[1],arrival->temp, n->nbrValue, n->slope);
					if(arrival->valve_status==1){
						//valve is already open so do nothing
						printf("the valve of %d.%d is open so do nothing\n", arrival->sendAddr.u8[0], arrival->sendAddr.u8[1]);
					}else{
						//valve is close so let's open it if threshold is reached
						if(abs(n->slope) > THRESHOLD){
							runicast_struct message;
							linkaddr_copy(&(&message)->sendAddr, &linkaddr_node_addr);
							linkaddr_copy(&(&message)->destAddr, &n->address);
							message.option = OPENING_VALVE;
							printf("sending messages to open the valve:\n");
							packetbuf_copyfrom(&message, sizeof(runicast_struct));
							runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
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
			runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
		}
		children_struct *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &arrival->sendAddr)) {
				break;
			}
		}
		if(n == NULL) {
			n = memb_alloc(&children_memb);

			if(n == NULL){
				return;
			}
			linkaddr_copy(&n->address, &arrival->sendAddr);
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
			children_struct *n;
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
				runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			else{ //don't know where to send it, -> send to parent
				runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
			}
		}
	}

	else if(arrival->option == FLUSH_CHILDREN){
		printf("flushing info received\n");
		printf("parent lost\n");
		static_rank = SHRT_MAX;
		parent_rssi = -SHRT_MAX;
		children_struct *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &n->next_hop)){
				arrival->destAddr = n->address;
				packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
				runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, n);
		}
	}

	else if(arrival->option == CHILDREN_LOST){
		children_struct *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &arrival->child_lost)){
				list_remove(children_list, n);
				break;
			}
		}
		packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
		runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
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
		static_rank = SHRT_MAX;
		parent_rssi = -SHRT_MAX;
		children_struct *n;
		runicast_struct flush_message;
		linkaddr_copy(&(&flush_message)->sendAddr, &linkaddr_node_addr);
		(&flush_message)->option = FLUSH_CHILDREN;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &n->next_hop)){
				(&flush_message)->destAddr = n->address;
				packetbuf_copyfrom(&flush_message ,sizeof(runicast_struct));
				runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, n);
		}
	}

	else{
		children_struct *n;
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
			linkaddr_copy(&(&lost_msg)->sendAddr, &linkaddr_node_addr);
			(&lost_msg)->option = CHILDREN_LOST;
			packetbuf_copyfrom(&lost_msg ,sizeof(runicast_struct));
			runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
		}
	}
}
static const struct runicast_callbacks runicast_call = {recv_runicast, sent_runicast, timedout_runicast};


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(computation_process_runicast, ev, data){
	PROCESS_EXITHANDLER(runicast_close(&runicast);)

	PROCESS_BEGIN();
	printf("[Computation node] Starting runicast");
	runicast_open(&runicast, 144, &runicast_call);

	while(1) {
		static struct etimer et;
		etimer_set(&et, CLOCK_SECOND * ROUTING_INTERVAL);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		compute_least_square_slope();
	}
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(computation_broadcast_routing, ev, data) {
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();
	printf("[Computation node] Starting broadcast");
	broadcast_open(&broadcast, 129, &broadcast_call);

	static_rank = SHRT_MAX;
	parent_rssi = -SHRT_MAX;

	while(1) {
		broadcast_struct message;
		static struct etimer et;
		etimer_set(&et, CLOCK_SECOND * ROUTING_INTERVAL);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		if(static_rank != SHRT_MAX) {
			message.option = BROADCAST_INFO;
			message.rank = static_rank;
			message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
			message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];

			packetbuf_copyfrom(&message ,sizeof(message));
			printf("[Computation node] Broadcast sent with rank : %d\n", static_rank);
			broadcast_send(&broadcast);
		}
		else {
			printf("[Computation node] No routing info sent, not connected to the network !\n");
		}
	}
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
