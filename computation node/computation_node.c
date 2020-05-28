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
	SAVE_CHILDREN,
	LOST_CHILDREN
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
int compute(runicast_struct* arrival, const linkaddr_t *from)
{
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
	Computes the slope of the measurements as the division of the sum of the product on the sum of the squares.
	For simplicity, the data is stored in reverse order (index 0 = last entry) => slope computed in reverse order
*/
void compute_slope()
{
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
				product_sum += ((node->sensorValue)[i]-avg) * (i-avg_y);
				squares_sum += pow(((node->sensorValue)[i]-avg), 2);
			}
			if(squares_sum==0) squares_sum++;

			node->slope = (-product_sum / squares_sum) * 1000;
		}
	}
}


/*
	Functions for runicast
*/
static void runicast_recv(struct runicast_conn *c, const linkaddr_t *from, uint8_t seq)
{
	runicast_struct* arrival = packetbuf_dataptr();
	history_struct *e = NULL;
	static signed char rssi_val;
	static signed char rssi_offset = -45;

	// History managing
	for(e = list_head(history_table); e != NULL; e = e->next) {
		if(linkaddr_cmp(&e->addr, from)) break;
	}

	if(e != NULL) {
		if(e->seq == seq) {
			printf("[Computation node] Duplicate runicast message received from : node %d.%d, sequence number : %d\n", from->u8[0], from->u8[1], seq);
			return;
		}
		e->seq = seq;
	}
	else {
		e = memb_alloc(&history_mem);
		if(e == NULL) {
			e = list_chop(history_table);
		}
		linkaddr_copy(&e->addr, from);
		e->seq = seq;
		list_push(history_table, e);
	}
	printf("[Computation node] Runicast message received from : node %d.%d, value : %d, source : %d.%d\n", from->u8[0], from->u8[1], arrival->temp, arrival->sendAddr.u8[0], arrival->sendAddr.u8[1]);

	// Behaviour by type of message
	if(arrival->option == SENSOR_INFO) {
		if(compute(arrival, from)) {
			compute_struct *node;

			for(node = list_head(computation_list); node != NULL; node = list_item_next(node)) {
				if(linkaddr_cmp(&node->address, &arrival->sendAddr) && arrival->valve_status!=1 && abs(node->slope) > THRESHOLD) {
					runicast_struct message;
					message.option = OPENING_VALVE;
					linkaddr_copy(&(&message)->sendAddr, &linkaddr_node_addr);
					linkaddr_copy(&(&message)->destAddr, &node->address);
					packetbuf_copyfrom(&message, sizeof(runicast_struct));
					printf("[Computation node] Message to open the valve sent to : %d.%d\n", from->u8[0], from->u8[1]);
					runicast_send(&runicast, &node->next_hop, MAX_RETRANSMISSIONS);
				}
			}
		}

		else {
			linkaddr_copy(&arrival->destAddr, &parent_addr);
			packetbuf_copyfrom(arrival, sizeof(runicast_struct));
			printf("[Computation node] Overloaded, sent to server by parent : %d.%d\n", parent_addr.u8[0], parent_addr.u8[1] );
			runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
		}

		children_struct *node;
		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, &arrival->sendAddr)) {
				break;
			}
		}
		if(node == NULL) {
			n = memb_alloc(&children_memb);
			if(node == NULL) return;
			linkaddr_copy(&node->address, &arrival->sendAddr);
			linkaddr_copy(&node->next_hop, from);
			list_add(children_list, node);
		}
	}


	else if(arrival->option == OPENING_VALVE) {
		rssi_val = cc2420_last_rssi + rssi_offset;

		if(!linkaddr_cmp(&arrival->destAddr, &linkaddr_node_addr)) {
			children_struct *node;
			bool found = false;
			for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
				if(linkaddr_cmp(&node->address, &arrival->destAddr)) {
					found = true;
					break;
				}
			}
			packetbuf_copyfrom(arrival, sizeof(runicast_struct));
			if(found) runicast_send(&runicast, &node->next_hop, MAX_RETRANSMISSIONS);
			else runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
		}

		else printf("[Computation node] +++ Opening valve\n");
	}


	else if(arrival->option == SAVE_CHILDREN) {
		parent_rssi = -SHRT_MAX;
		static_rank = SHRT_MAX;
		children_struct *node;

		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, &node->next_hop)){
				arrival->destAddr = node->address;
				packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
				runicast_send(&runicast, &node->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, node);
		}
	}


	else if(arrival->option == LOST_CHILDREN) {
		children_struct *node;
		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, &arrival->child_lost)) {
				list_remove(children_list, node);
				break;
			}
		}
		packetbuf_copyfrom(arrival, sizeof(runicast_struct));
		runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
	}
}


static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
	printf("[Computation node] Runicast message sent to %d.%d, retransmission %d\n", to->u8[0], to->u8[1], retransmissions);
}


static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
	if(!linkaddr_cmp(to, &parent_addr)) {
		children_struct *node;
		bool found = false;
		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, to)) {
				list_remove(children_list, node);
				found = true;
				break;
			}
		}
		if (found) {
			runicast_struct lost_msg;
			(&lost_msg)->option = LOST_CHILDREN;
			linkaddr_copy(&(&lost_msg)->sendAddr, &linkaddr_node_addr);
			packetbuf_copyfrom(&lost_msg, sizeof(runicast_struct));
			printf("[Computation node] Runicast message timed out when sending to %d.%d, retransmission %d\n", to->u8[0], to->u8[1], retransmissions);
			runicast_send(&runicast, &node->next_hop, MAX_RETRANSMISSIONS);
		}
	}

	else {
		children_struct *node;
		runicast_struct save_message;
		parent_rssi = -SHRT_MAX;
		static_rank = SHRT_MAX;

		(&save_message)->option = SAVE_CHILDREN;
		linkaddr_copy(&(&save_message)->sendAddr, &linkaddr_node_addr);

		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, &node->next_hop)) {
				linkaddr_copy(&(&save_message)->destAddr, &node->address);
				packetbuf_copyfrom(&save_message, sizeof(runicast_struct));
				runicast_send(&runicast, &node->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, node);
		}
	}
}
static const struct runicast_callbacks runicast_call = {runicast_recv, sent_runicast, timedout_runicast};


/*
	Function for broadcast
*/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	broadcast_struct* arrival = packetbuf_dataptr();
	static signed char rssi_signal;
	static signed char rssi_offset;

	// Behaviour by type of message
	if(arrival->option == BROADCAST_INFO ) {
		rssi_offset = -45;
		rssi_signal = cc2420_last_rssi + rssi_offset;
		printf("[Computation node] Routing information received from : src %d.%d with rank %d and rssi signal : %d\n", arrival->sendAddr.u8[0], arrival->sendAddr.u8[1] , arrival->rank, rssi_signal );

		if(rssi_signal > parent_rssi && arrival->rank < static_rank-1) {
			static_rank = arrival->rank + 1;
			parent_rssi = rssi_signal;
			parent_addr.u8[0] = (arrival->sendAddr).u8[0];
			parent_addr.u8[1] = (arrival->sendAddr).u8[1];
			printf("[Computation node] New parent : %d.%d, new rank : %d\n", (parent_addr).u8[0], (arrival->sendAddr).u8[1], static_rank);
		}
	}

	else if(arrival->option == BROADCAST_REQUEST && static_rank != SHRT_MAX) {
		broadcast_struct message;
		message.rank = static_rank;
		message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
		message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];
		packetbuf_copyfrom(&message, sizeof(message));
		printf("[Computation node] Routing info sent via broadcast, rank : %d\n", static_rank);
		broadcast_send(&broadcast);
	}

	else return;
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


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

		compute_slope();
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
