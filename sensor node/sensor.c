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

#define MAX_HISTORY 10
#define MAX_CHILDREN 100
#define ROUTING_INTERVAL 120
#define MAX_RETRANSMISSIONS 10
#define MEASUREMENT_INTERVAL 60


// Structures definition
typedef struct Broadcast broadcast_struct;
struct Broadcast
{
	short rank;                             // rank of the node
	linkaddr_t sendAddr;                    // address of the node sending the message
	uint8_t option;                         // type of message
};

typedef struct Runicast runicast_struct;
struct Runicast
{
	short rank;                             // rank of the node
	short temp;                             // temperature value read by the sensor
	short valve_status;                     // current state of the valve : closed(0) or open(1)
	linkaddr_t sendAddr;                    // address of the node sending the message
	linkaddr_t destAddr;                    // address of the node targeted to receive the message
	linkaddr_t child_lost;                  //
	uint8_t option;                         // type of message
};

typedef struct Children children_struct;
struct Children
{
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


// Enumerations definition
enum{
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


// Static variables definition
static int parent_rssi;
static short static_rank;
static linkaddr_t parent_addr;
static short valve_is_open = 0;

// Static structures definition
static struct broadcast_conn broadcast;
static struct runicast_conn runicast;


/*---------------------------------------------------------------------------*/
PROCESS(sensor_process_runicast, "[Sensor node] +++ runicast execution");
PROCESS(sensor_broadcast_routing, "[Sensor node] +++ broadcast routing");
AUTOSTART_PROCESSES(&sensor_process_runicast, &sensor_broadcast_routing);
/*---------------------------------------------------------------------------*/

/*
	Generation of random measurements
	Values in [1:50] if the valve is open
	       in [1:100] else
*/
short collect_measurement(){
	leds_off(LEDS_ALL);
	printf("[Sensor node] Loading measurements\n");
	short measurement;
	if(valve_is_open) { measurement = (random_rand() % 50) +1; }
	else { measurement = (random_rand() % 100) +1; }

	if(measurement >= 40){
		leds_toggle(LEDS_GREEN);
	}
	return measurement;
}

/*
	Functions for runicast
*/
static void runicast_recv(struct runicast_conn *c, const linkaddr_t *from, uint8_t seq)
{
	runicast_struct* arrival = packetbuf_dataptr();
	history_struct *e = NULL;
	static signed char rssi_offset = -45;

	// History managing
	for (e = list_head(history_table); e != NULL; e = e->next) {
		if(linkaddr_cmp(&e->addr, from)) break;
	}

	if(e != NULL) {
		if(e->seq == seq) {
			printf("[Sensor node] Duplicate runicast message received from : node %d.%d, sequence number : %d\n", from->u8[0], from->u8[1], seq);
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
	printf("[Sensor node] Runicast message received from : node %d.%d, value : %d, source : %d.%d\n",from->u8[0], from->u8[1], arrival->temp, arrival->sendAddr.u8[0], arrival->sendAddr.u8[1]);

	// Behaviour by type of message
	if(arrival->option == SENSOR_INFO) {
		linkaddr_copy(&arrival->destAddr, &parent_addr);
		packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
		printf("[Sensor node] Sensor info received from : node %d.%d, source : %d.%d, Sending to parent: %d.%d \n", from->u8[0], from->u8[1], arrival->sendAddr.u8[0], arrival->sendAddr.u8[1], parent_addr.u8[0], parent_addr.u8[1]);
		runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);

		children_struct *child;
		for(child = list_head(children_list); child != NULL; child = list_item_next(child)) {
			if(linkaddr_cmp(&child->address, &arrival->sendAddr)) {
				child->last_update = clock_time();
				break;
			}
		}
		if(child == NULL){
			child = memb_alloc(&children_memb);
			if(child == NULL) return;
			linkaddr_copy(&child->address, &arrival->sendAddr);
			linkaddr_copy(&child->next_hop, from);
			child->last_update = clock_time();
			list_add(children_list, child);
		}
	}


	else if(arrival->option == OPENING_VALVE) {
		parent_rssi = cc2420_last_rssi + rssi_offset;

		if(!linkaddr_cmp(&arrival->destAddr, &linkaddr_node_addr)) {
			children_struct *child;
			bool found = false;
			for(child = list_head(children_list); child != NULL; child = list_item_next(child)) {
				if(linkaddr_cmp(&child->address, &arrival->destAddr)) {
					found = true;
					break;
				}
			}
			packetbuf_copyfrom(arrival, sizeof(runicast_struct));
			if(found) {
				runicast_send(&runicast, &child->next_hop, MAX_RETRANSMISSIONS);
			}
			else {
				runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
			}
		}

		else printf("[Sensor node] +++ Opening valve\n");
	}


	else if(arrival->option == SAVE_CHILDREN) {
		parent_rssi = -SHRT_MAX;
		static_rank = SHRT_MAX;
		children_struct *child;

		for(child = list_head(children_list); child != NULL; child = list_item_next(child)) {
			if(linkaddr_cmp(&child->address, &child->next_hop)) {
				arrival->destAddr = child->address;
				packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
				runicast_send(&runicast, &child->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, child);
		}
	}


	else if(arrival->option == LOST_CHILDREN) {
		children_struct *child;
		for(child = list_head(children_list); child != NULL; child = list_item_next(child)) {
			if(linkaddr_cmp(&child->address, &arrival->child_lost)) {
				list_remove(children_list, child);
				break;
			}
		}
		packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
		runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
	}
}


static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
	printf("[Sensor node] Runicast message sent to %d.%d, retransmission %d\n", to->u8[0], to->u8[1], retransmissions);
}


static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
	if(!linkaddr_cmp(to, &parent_addr)) {
		children_struct *child;
		bool found = false;
		for(child = list_head(children_list); child != NULL; child = list_item_next(child)) {
			if(linkaddr_cmp(&child->address, to)) {
				list_remove(children_list, child);
				found = true;
				break;
			}
		}
		if (found==true) {
			runicast_struct lost_msg;
			(&lost_msg)->option = LOST_CHILDREN;
			linkaddr_copy(&(&lost_msg)->sendAddr, &linkaddr_node_addr);
			packetbuf_copyfrom(&lost_msg ,sizeof(runicast_struct));
			printf("[Sensor node] Runicast message timed out when sending to %d.%d, retransmission %d\n", to->u8[0], to->u8[1], retransmissions);
			runicast_send(&runicast, &child->next_hop, MAX_RETRANSMISSIONS);
		}
	}

	else {
		children_struct *child;
		runicast_struct save_message;
		parent_rssi = -SHRT_MAX;
		static_rank = SHRT_MAX;

		(&save_message)->option = SAVE_CHILDREN;
		linkaddr_copy(&(&save_message)->sendAddr, &linkaddr_node_addr);

		for(child = list_head(children_list); child != NULL; child = list_item_next(child)) {
			if(linkaddr_cmp(&child->address, &child->next_hop)){
				linkaddr_copy(&(&save_message)->destAddr, &child->address);
				packetbuf_copyfrom(&save_message ,sizeof(runicast_struct));
				runicast_send(&runicast, &child->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, child);
		}
	}
}
static const struct runicast_callbacks runicast_call = {runicast_recv, sent_runicast, timedout_runicast};


/*
	Functions for broadcast
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
		printf("[Sensor node] Routing information received from : src %d.%d with rank %d and rssi signal : %d\n", arrival->sendAddr.u8[0], arrival->sendAddr.u8[1] , arrival->rank, rssi_signal );

		if(rssi_signal > parent_rssi && arrival->rank < static_rank-1){
			static_rank = arrival->rank + 1;
			parent_rssi = rssi_signal;
			parent_addr.u8[0] = (arrival->sendAddr).u8[0];
			parent_addr.u8[1] = (arrival->sendAddr).u8[1];
			printf("[Sensor node] New parent : %d.%d, new rank : %d\n", (parent_addr).u8[0], (arrival->sendAddr).u8[1], static_rank);
		}
	}

	else if(arrival->option == BROADCAST_REQUEST && static_rank != SHRT_MAX) {
		broadcast_struct message;
		message.rank = static_rank;
		message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
		message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];
		packetbuf_copyfrom(&message ,sizeof(message));
		printf("[Sensor node] Routing info sent via broadcast, rank : %d\n", static_rank);
		broadcast_send(&broadcast);
	}

	else return;
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process_runicast, ev, data) {
	PROCESS_EXITHANDLER(runicast_close(&runicast);)

	PROCESS_BEGIN();
	printf("[Sensor node] Starting runicast");
	random_init(linkaddr_node_addr.u8[0]);
	runicast_open(&runicast, 144, &runicast_call);

	while(1) {
		runicast_struct msg;
		static struct etimer et;
		etimer_set(&et, CLOCK_SECOND * MEASUREMENT_INTERVAL);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		if(static_rank != SHRT_MAX) {
			msg.option = SENSOR_INFO;
			msg.temp = collect_measurement();
			msg.rank = static_rank;
			msg.valve_status = valve_is_open;
			linkaddr_copy(&(&msg)->sendAddr, &linkaddr_node_addr);
			linkaddr_copy(&(&msg)->destAddr, &parent_addr);

			packetbuf_copyfrom(&msg, sizeof(msg));
			printf("[Sensor node] Runicast value sent to parent %d.%d\n", parent_addr.u8[0], parent_addr.u8[1]);
			runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
		}
	}
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_broadcast_routing, ev, data) {
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();
	printf("[Sensor node] Starting broadcast");
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

			packetbuf_copyfrom(&message, sizeof(message));
			printf("[Sensor node] Broadcast sent with rank : %d\n", static_rank);
			broadcast_send(&broadcast);
		}
		else {
			printf("[Sensor node] No routing info sent, not connected to the network !\n");
		}
	}
	PROCESS_END();
}

	/*---------------------------------------------------------------------------*/
