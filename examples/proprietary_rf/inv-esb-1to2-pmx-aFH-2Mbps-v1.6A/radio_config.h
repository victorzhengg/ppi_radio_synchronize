#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#define DATA_SENDING_P0   3
#define DATA_SENDING_P1   4
#define DATA_SENDING_P2  28
#define DATA_SENDING_P3	 29
#define DATA_SENDING_P4	 30

#define RADIO_FRM_SIZE		 12
#define NUM_OF_PERIPH			 2

#define	PAIRING_PIPE			0
#define	DATA_PIPE					1

#define PAIRING_ADDRESS			{0xE7, 0xE7, 0xE7, 0xE7}
#define SYSTEM_ADDRESS			{0xC2, 0xC2, 0xC2, 0xC2}


#define PERIPH_ON_SYNC_CNT		100

#define LOG_CNT								1000

#define RADIO_CHAN_TAB 					{2, 42, 72, 12, 22, 32, 52, 62, 78}		
#define TOTAL_CHAN_TAB_SIZE			9
#define	RADIO_CHAN_TAB_SIZE		 	3
#define	RADIO_UNUSED_CHAN_TAB_SIZE	 (TOTAL_CHAN_TAB_SIZE - RADIO_CHAN_TAB_SIZE)
#define CENTRAL_SWITCH_CHAN_CNT		5//3

#define	RETRAN_CNT								1

#define ADJ                    	  15//20  
#define POLL_TICKS            		30//30.5 us  PER TICK 
	
#define RX_SEARCH_PERIOD    				 	POLL_TICKS * (NUM_OF_PERIPH+2)
#define RX_OPERATE_PERIOD_JUST_SYNC		255
#define RX_OPERATE_PERIOD_W_WAIT			POLL_TICKS * (NUM_OF_PERIPH) - ADJ -15
#define	RX_WAIT_FOR_ACK_WR_PERIOD			ADJ
#define RX_OPERATE_PERIOD							POLL_TICKS * (NUM_OF_PERIPH) 

	
#define RX_LOSS_THRESHOLD			RADIO_CHAN_TAB_SIZE * NUM_OF_PERIPH*2

//aFH
#define CENTRAL_LOSS_THRESHOLD  6//4

#endif
