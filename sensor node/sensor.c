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
#define MAX_RETRANSMISSIONS 10
#define seconds_before_routing_info 120
#define seconds_before_sending_measurement 60


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
	children_struct *next;           // next child
};

typedef struct History history_struct;
struct History {
	uint8_t seq;                           // sequence number
	linkaddr_t addr;                       // address of the node
	history_struct *next;           // next entry in the history
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
//static struct ctimer ctimer_valve_reset; // not used yet


// Static constant structure definition
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


/*---------------------------------------------------------------------------*/
PROCESS(sensor_process_runicast, "[Sensor node] +++ runicast execution");
PROCESS(broadcast_routing, "[Sensor node] +++ broadcast routing");
AUTOSTART_PROCESSES(&sensor_process_runicast, &broadcast_routing);
/*---------------------------------------------------------------------------*/


// not used yet
/*
static void closing_valve_timeout(){
	valve_is_open=0;
	printf("[Sensor node] --- Closing valve after 10 minutes open\n");
}
*/


/*
* //TODO remove the broadcast struct and just use a pointer to the rank
* broadcast_recv is called when a routing information from a neighbouring node is recieved
* it will decide if the node sending the broadcast would be a better suited parent that the current one
* Parameters:
* struct broadcast_conn *c : a pointer to the structure containing the rank and the adress of the sending node
* const linkaddr_t *from : the adress of the sender
* INSPIRED FROM example-broadcast.c
*/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	broadcast_struct* arrival = packetbuf_dataptr();
	static signed char rssi_signal;
	static signed char rssi_offset;

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
		printf("[Sensor node] Routing info sent via broadcast rank : %d\n", static_rank);
		broadcast_send(&broadcast);
	}

	else return;
}

static void runicast_recv(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	static signed char rss_offset = -45;
	runicast_struct* arrival = packetbuf_dataptr();
	history_struct *e = NULL;
	for (e = list_head(history_table); e != NULL; e = e->next) {
		if(linkaddr_cmp(&e->addr, from)) { break; }
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

	printf("runicast message recieved from %d.%d | originating from %d.%d | value %d \n",from->u8[0], from->u8[1],arrival->sendAddr.u8[0],arrival->sendAddr.u8[1],arrival->temp);

	if(arrival->option == SENSOR_INFO){
		printf("sensor info received from %d.%d | originating from %d.%d, transmitting to parent: %d.%d \n", from->u8[0], from->u8[1],arrival->sendAddr.u8[0],arrival->sendAddr.u8[1], parent_addr.u8[0], parent_addr.u8[1]);
		linkaddr_copy(&arrival->destAddr, &parent_addr);
		packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
		runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);

		children_struct *n;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &arrival->sendAddr)) {
				n->last_update = clock_time();
				break;
			}
		}
		if(n == NULL){//children not found
			n = memb_alloc(&children_memb);

			if(n == NULL){
				return;
			}
			linkaddr_copy(&n->address, &arrival->sendAddr);
			linkaddr_copy(&n->next_hop, from);
			n->last_update = clock_time();
			list_add(children_list, n);
		}
	}
	else if(arrival->option == OPENING_VALVE){
		parent_rssi = cc2420_last_rssi + rss_offset;
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
			packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
			if(found==true){
				runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			else{ //don't know where to send it, -> send to parent
				runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
			}
		}
	}
	/*else if(arrival->option == CLOSING_VALVE){
		parent_rssi = cc2420_last_rssi + rss_offset;
		if(linkaddr_cmp(&arrival->destAddr, &linkaddr_node_addr)){
			printf("Closing valve\n");
			valve_is_open = 1;
			ctimer_set(&ctimer_valve_reset, 600 * CLOCK_SECOND, closing_valve_timeout, NULL);
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

			packetbuf_copyfrom(arrival ,sizeof(runicast_struct));
			if(found==true){
				runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			else{ //don't know where to send it, -> send to parent
				runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
			}
		}
	}*/
	else if(arrival->option == SAVE_CHILDREN){
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
	else if(arrival->option == LOST_CHILDREN){
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
		(&flush_message)->option = SAVE_CHILDREN;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &n->next_hop)){
				linkaddr_copy(&(&flush_message)->destAddr, &n->address);
				packetbuf_copyfrom(&flush_message ,sizeof(runicast_struct));
				runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
			}
			list_remove(children_list, n);
		}
	}
	//in case children is no more reachable
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
			(&lost_msg)->option = LOST_CHILDREN;
			packetbuf_copyfrom(&lost_msg ,sizeof(runicast_struct));
			runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
		}
	}
}

static const struct runicast_callbacks runicast_callbacks = {runicast_recv, sent_runicast, timedout_runicast};

short collect_measurement(){
	leds_off(LEDS_ALL);
	short measurement;
	if(valve_is_open) { measurement = (random_rand() % 50) +1; } // values in the interval 1-50
	else { measurement = (random_rand() % 100) +1; } // values in the interval 1-100

	if(measurement >= 40){
		leds_toggle(LEDS_GREEN);
	}
	return measurement;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensor_process_runicast, ev, data){
	PROCESS_EXITHANDLER(runicast_close(&runicast);)
	PROCESS_BEGIN();
	random_init(linkaddr_node_addr.u8[0]); //to use a different seed per node to generate random measurement, I use their IP address
	runicast_open(&runicast, 144, &runicast_callbacks);

	while(1) {
		static struct etimer et;
		runicast_struct msg;
		etimer_set(&et, CLOCK_SECOND * seconds_before_sending_measurement);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		if(static_rank != SHRT_MAX){
			printf("sending runicast value to parent %d.%d\n", parent_addr.u8[0], parent_addr.u8[1]);
				linkaddr_copy(&(&msg)->sendAddr, &linkaddr_node_addr);
				linkaddr_copy(&(&msg)->destAddr, &parent_addr);
				msg.temp = collect_measurement();
				msg.rank = static_rank;
				msg.valve_status = valve_is_open;
				msg.option = SENSOR_INFO;

			packetbuf_copyfrom(&msg, sizeof(msg));
			runicast_send(&runicast, &parent_addr, MAX_RETRANSMISSIONS);
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_routing, ev, data){
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	broadcast_open(&broadcast, 129, &broadcast_call);

	static_rank = SHRT_MAX;
	parent_rssi = -SHRT_MAX;

	//if(linkaddr_node_addr.u8[0]==1){static_rank = 0; printf("fake root\n");} //TODO remove, just for testing without border router

	while(1) {
		static struct etimer et;
		/* Send routing informations via broadcast every 60 secondes */
		etimer_set(&et, CLOCK_SECOND * seconds_before_routing_info);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		if(static_rank != SHRT_MAX){
			//printf("Sending routing info via broadcast rank: %d\n", static_rank);
			broadcast_struct message;
			message.rank = static_rank;
			message.option = BROADCAST_INFO; //modified
			message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
			message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];
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
