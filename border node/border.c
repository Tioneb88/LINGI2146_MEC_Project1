/*
	LINGI2146 Mobile and Embedded Computing : Project1
	Author : Benoît Michel
	Date : May 2020
*/
#include "contiki.h"
#include "contiki-net.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "net/rime/rime.h"
#include "sys/ctimer.h"
#include "sys/timer.h"

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
	CLOSING_VALVE,
	FLUSH_CHILDREN,
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
static struct ctimer broadcast_ctimer;

static struct broadcast_conn broadcast;
static struct runicast_conn runicast;


/*---------------------------------------------------------------------------*/
PROCESS(upstream_boarder_node_process, "upstream");
PROCESS(downstream_boarder_node_process, "downstream");
AUTOSTART_PROCESSES(&upstream_boarder_node_process, &downstream_boarder_node_process);
/*---------------------------------------------------------------------------*/


void process_answer(char str[])
{
	/*char delim[] = " ";
	char *ptr = strtok(str, delim);
	*/
	char type[7];
	short action;
	if ( str[0] == 'A') { //it's an ACTION
		if ( str[11] == 'O'){
			action = 0;
		}
		else if ( str[11] == 'C') {
			action = 1;
		}
		//printf("sensor = %c.%c action = %d \n", str[7], str[9], action );

		runicast_struct msg;
		linkaddr_t dest_addr;

		int addr_part_1 = str[7] -'0';
		int addr_part_2 = str[9] - '0';
		dest_addr.u8[0] = addr_part_1;
		dest_addr.u8[1] = addr_part_2;
		linkaddr_copy(&(&msg)->sendAddr, &linkaddr_node_addr); //sender = myself
		linkaddr_copy(&(&msg)->destAddr, &dest_addr); //destination = dest_addr
		msg.temp = action;
		msg.rank = 1;
		if(action==1){
			msg.option = CLOSING_VALVE;
		}
		else if(action==0){
		msg.option = OPENING_VALVE;
		}
		packetbuf_copyfrom(&msg, sizeof(msg));

		children_struct *n;
		bool found = false;
		for(n = list_head(children_list); n != NULL; n = list_item_next(n)) {
			if(linkaddr_cmp(&n->address, &(&msg)->destAddr)) {
				found = true;
				break;
			}
		}

		if(found==true){
			printf("sending a runicast message (server answer), destination %d, nexthop %d \n", dest_addr.u8[0], n->next_hop.u8[0]);
			runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
		}
		else{
			n = list_head(children_list); // I should have at least one
			printf("sending a runicast message (server answer), destination %d, nexthop %d \n", dest_addr.u8[0], n->next_hop.u8[0]);
			runicast_send(&runicast, &n->next_hop, MAX_RETRANSMISSIONS);
		}
	}
}


/*
	Functions for runicast
*/
static void runicast_recv(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	runicast_struct* arrival = packetbuf_dataptr();
	history_struct *e = NULL;
	static signed char rss_offset = -45;

	  /* OPTIONAL: Sender history */
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

	if(arrival->option == SENSOR_INFO){ //adding the child
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

	//runicast_struct* arrival = packetbuf_dataptr();
	printf("unicast message recieved from %d.%d | originating from %d.%d | value %d \n",from->u8[0], from->u8[1],arrival->sendAddr.u8[0],arrival->sendAddr.u8[1],arrival->temp);
	char message[] = "SENSOR_VALUE " ;
	char str[21];
	sprintf(str, "%d", arrival->sendAddr.u8[0]);
	strcat(message, str);
	strcat(message, " ");
	char str2[21];
	sprintf(str2, "%d", arrival->sendAddr.u8[1]);
	strcat(message, str2);
	strcat(message, " ");
	char str3[21];
	sprintf(str3, "%d", arrival->temp);
	strcat(message, str3);
	strcat(message, " \n");
	//printf("message I will send to server : %s", message);
	printf(message); //printing will actually send to server via serial socket (activated within the simulation tools)

}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}
static const struct runicast_callbacks runicast_callbacks = {runicast_recv, sent_runicast};



static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	broadcast_struct* arrival = packetbuf_dataptr();
	//printf("Routing information recieved from: src %d  with rank %d \n", arrival->sendAddr.u8[0], arrival->rank );
	//printf("I can just ignore it since my rank will always be 1 \n");
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};




void broadcast_timeout() //let over nodes know that they can join me
{
	ctimer_reset(&broadcast_ctimer);
	//printf("broadcasting routing info rank %d \n", static_rank);
	broadcast_struct message;
	message.rank = static_rank;
	message.option = BROADCAST_INFO; //modified
	message.sendAddr.u8[0] = linkaddr_node_addr.u8[0];
  message.sendAddr.u8[1] = linkaddr_node_addr.u8[1];
	packetbuf_copyfrom( &message ,sizeof(message));
	broadcast_send(&broadcast);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////// upstream process //////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PROCESS_THREAD(upstream_boarder_node_process, ev, data)
{
	PROCESS_EXITHANDLER(runicast_close(&runicast);)
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();
	runicast_open(&runicast, 144, &runicast_callbacks);
	broadcast_open(&broadcast, 129, &broadcast_call);

	static_rank = 1;
	broadcast_timeout();
	ctimer_set(&broadcast_ctimer, CLOCK_SECOND * ROUTING_INTERVAL, broadcast_timeout, NULL);
	PROCESS_YIELD();
	PROCESS_END();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////// downstream process //////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This part is temporar not finished yet. It should decode the message received and send the value to the correct node

PROCESS_THREAD(downstream_boarder_node_process, ev, data)
{

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	for(;;) {
		PROCESS_YIELD();
		if(ev == serial_line_event_message) {
		//printf("recieved line: %s \n", (char *) data);
		process_answer( (char *) data);
		}
	}
   PROCESS_END();
}
