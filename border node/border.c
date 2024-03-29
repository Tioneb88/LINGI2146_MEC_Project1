/*
	LINGI2146 Mobile and Embedded Computing : Project1
	Author : Benoît Michel
	Date : May 2020
*/
#include "contiki.h"
#include "contiki-net.h"
#include "dev/serial-line.h"
#include "net/rime/rime.h"
#include "sys/timer.h"
#include "sys/ctimer.h"

#include <stdio.h>
#include <limits.h>
#include <stdbool.h>
#include <string.h>

#define MAX_HISTORY 10
#define MAX_CHILDREN 100
#define ROUTING_INTERVAL 120
#define MAX_RETRANSMISSIONS 10
#define PORT = 60001
#define HOST = "127.0.0.1"


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
	linkaddr_t child_lost;                  // if node is lost
	uint8_t option;                         // type of message
};

typedef struct Children children_struct;
struct Children {
	linkaddr_t address;                     // address of the node
	linkaddr_t next_hop;                    // nexthop
	clock_time_t last_update;               // last update of the children
	children_struct *next;                  // next node
};

typedef struct History history_struct;
struct History {
	uint8_t seq;                           // sequence number
	linkaddr_t addr;                       // address of the node
	history_struct *next;                  // next entry in the history
};


// Enumerations definition
enum {
	SENSOR_INFO,
	OPENING_VALVE,
	SAVE_CHILDREN,
	LOST_CHILDREN,
	CLOSING_VALVE
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
static short static_rank;

// Static structures definition
static struct ctimer broadcast_ctimer;
static struct broadcast_conn broadcast;
static struct runicast_conn runicast;


/*---------------------------------------------------------------------------*/
PROCESS(border_process_cast, "[Border node] Runicast and broadcast");
PROCESS(border_process_messages, "[Border node] Process messages");
AUTOSTART_PROCESSES(&border_process_cast, &border_process_messages);
/*---------------------------------------------------------------------------*/


/*
	Process the received messages
*/
void process(char str[])
{
	bool open = false;
	if(str[0] == 'O') {
		if(str[11] == 'P') {
			open = true;
		}

		runicast_struct message;
		linkaddr_t dest_addr;

		message.rank = 1;
		message.temp = open;
		if(open) message.option = OPENING_VALVE;
		else message.option = CLOSING_VALVE;

		dest_addr.u8[0] = str[7] -'0';
		dest_addr.u8[1] = str[9] - '0';
		linkaddr_copy(&(&message)->sendAddr, &linkaddr_node_addr);
		linkaddr_copy(&(&message)->destAddr, &dest_addr);
		packetbuf_copyfrom(&message, sizeof(message));

		children_struct *node;
		bool found = false;
		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, &(&message)->destAddr)) {
				found = true;
				break;
			}
		}
		if(!found) node = list_head(children_list);
		printf("[Border node] Runicast message forwarded from server, destination : %d, nexthop : %d\n", dest_addr.u8[0], node->next_hop.u8[0]);
		runicast_send(&runicast, &node->next_hop, MAX_RETRANSMISSIONS);
	}
}


/*
	Functions for runicast
*/
static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seq)
{
	runicast_struct* arrival = packetbuf_dataptr();
	history_struct *h = NULL;
	//static signed char rssi_offset = -45;

	// History managing
	for(h = list_head(history_table); h != NULL; h = h->next) {
		if(linkaddr_cmp(&h->addr, from)) break;
	}

	if(h != NULL) {
		if(h->seq == seq) {
			printf("[Border node] Duplicate runicast message received from : node %d.%d, sequence number : %d\n", from->u8[0], from->u8[1], seq);
			return;
		}
		h->seq = seq;
	}
	else {
		h = memb_alloc(&history_mem);
		if(h == NULL) h = list_chop(history_table);
		h->seq = seq;
		linkaddr_copy(&h->addr, from);
		list_push(history_table, h);
	}
	printf("[Border node] Runicast message received from : node %d.%d, value : %d, source : %d.%d\n", from->u8[0], from->u8[1], arrival->temp, arrival->sendAddr.u8[0], arrival->sendAddr.u8[1]);

	// Behaviour by type of message
	if(arrival->option == SENSOR_INFO) {
		children_struct *node;
		for(node = list_head(children_list); node != NULL; node = list_item_next(node)) {
			if(linkaddr_cmp(&node->address, &arrival->sendAddr)) {
				node->last_update = clock_time();
				break;
			}
		}
		if(node == NULL) {
			node = memb_alloc(&children_memb);
			if(node == NULL) return;
			linkaddr_copy(&node->address, &arrival->sendAddr);
			linkaddr_copy(&node->next_hop, from);
			node->last_update = clock_time();
			list_add(children_list, node);
		}
	}
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}
static const struct runicast_callbacks runicast_call = {recv_runicast, sent_runicast};


/*
	Functions for broadcast
*/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	broadcast_struct* arrival = packetbuf_dataptr();
	printf("[Border node] Routing information recieved from : node %d with rank : %d\n", arrival->sendAddr.u8[0], arrival->rank);
}


void broadcast_timeout()
{
	ctimer_reset(&broadcast_ctimer);
	broadcast_struct message;
	message.option = BROADCAST_INFO;
	message.rank = static_rank;
	message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
  message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];
	packetbuf_copyfrom( &message ,sizeof(message));
	printf("[Border node] Routing information broadcasted with rank : %d\n", static_rank);
	broadcast_send(&broadcast);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(border_process_cast, ev, data)
{
	PROCESS_EXITHANDLER(runicast_close(&runicast);broadcast_close(&broadcast);)
	//PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();
	printf("[Border node] Starting runicast and broadcast");
	runicast_open(&runicast, 144, &runicast_call);
	broadcast_open(&broadcast, 129, &broadcast_call);

	static_rank = 1;
	broadcast_timeout();
	ctimer_set(&broadcast_ctimer, CLOCK_SECOND * ROUTING_INTERVAL, broadcast_timeout, NULL);
	PROCESS_YIELD();

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/

PROCESS_THREAD(border_process_messages, ev, data)
{
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();

	for(;;) {
		PROCESS_YIELD();
		if(ev == serial_line_event_message) {
		//printf("recieved line: %s \n", (char *) data);
		process( (char *) data);
		}
	}
   PROCESS_END();
}

/*---------------------------------------------------------------------------*/
