/* ###################################################################
**     Filename    : main.c
**     Project     : FRDMK64F-Radio-Transmitter
**     Processor   : MK64FN1M0VLL12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-12-31, 15:35, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "FRTOS1.h"
#include "UTIL1.h"
#include "SM1.h"
#include "SMasterLdd1.h"
#include "CE.h"
#include "BitIoLdd1.h"
#include "CSN.h"
#include "BitIoLdd2.h"
#include "IRQ1.h"
#include "ExtIntLdd1.h"
#include "WAIT1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"

typedef uint8_t RPHY_FlagsType;

typedef struct {
  RPHY_FlagsType flags;/*!< flags, see RPHY_PACKET_FLAGS_XXXX above */
  uint8_t phySize;     /*!< size of PHY data buffer */
  uint8_t *phyData;    /*!< pointer to the PHY data buffer */
  uint8_t *rxtx;       /*!< pointer into phyData, start of TX/RX data */
} RPHY_PacketDesc;

#define nRFSPI_Enable()                   SM1_Enable()
#define nRFSPI_Disable()                  SM1_Disable()
#define nRFSPI_SetFastMode()              SM1_SetFastMode()
#define nRFSPI_SetSlowMode()              SM1_SetSlowMode()
#define nRFSPI_SetShiftClockPolarity(val) (void)SM1_SetShiftClockPolarity(val)
#define nRFSPI_SetIdleClockPolarity(val)  (void)SM1_SetIdleClockPolarity(val)
#define nRFSPI_GetCharsInTxBuf()          SM1_GetCharsInTxBuf()
#define nRFSPI_GetCharsInRxBuf()          SM1_GetCharsInRxBuf()
#define nRFSPI_SendChar(ch)               SM1_SendChar(ch)
#define nRFSPI_RecvChar(p)                SM1_RecvChar(p)
#define nRFSPI_SetBaudRateMode(m)         SM1_SetBaudRateMode(m)


#define RX_POWERUP()         nRFWriteRegister(0x00, ((1<<3)|(1<<2))|(1<<1)|(1<<0))

// #define nRFWAIT_US(x)  WAIT1_Waitus(x)          /* wait for the given number of micro-seconds */
#define nRFWAIT_MS(x)  WAIT1_Waitms(x)          /* wait for the given number of milli-seconds */

#define RPHY_HEADER_SIZE    (2) /* <flags><size> */
//#define RPHY_PAYLOAD_SIZE   (RNET_CONFIG_TRANSCEIVER_PAYLOAD_SIZE) /* total number of payload bytes */
//#define RPHY_BUFFER_SIZE    (RPHY_HEADER_SIZE+RPHY_PAYLOAD_SIZE) /* <size><phy payload> */

#define RPHY_BUF_IDX_FLAGS                (0) /* <flags> index */
#define RPHY_BUF_IDX_SIZE                 (1) /* <size> index */
#define RPHY_BUF_IDX_PAYLOAD              (2) /* <phy payload> index */
/* access macros */
#define RPHY_BUF_FLAGS(phy)               ((phy)[RPHY_BUF_IDX_FLAGS])
#define RPHY_BUF_SIZE(phy)                ((phy)[RPHY_BUF_IDX_SIZE])
#define RPHY_BUF_PAYLOAD_START(phy)       ((phy)+RPHY_HEADER_SIZE)

static RPHY_PacketDesc radioRx;
static uint8_t radioRxBuf[2+32];
static xQueueHandle RMSG_MsgRxQueue, RMSG_MsgTxQueue;

static uint8_t SPIWriteRead(uint8_t val) {
  	uint8_t ch;

  	while(nRFSPI_GetCharsInTxBuf()!=0) {} /* wait until tx is empty */
  	while(nRFSPI_SendChar(val)!=ERR_OK) {} /* send character */
  	while(nRFSPI_GetCharsInTxBuf()!=0) {} /* wait until data has been sent */
  	while(nRFSPI_GetCharsInRxBuf()==0) {} /* wait until we receive data */
  	while(nRFSPI_RecvChar(&ch)!=ERR_OK) {} /* get data */
  	return ch;
}

static void SPIWriteReadBuffer(uint8_t *bufOut, uint8_t *bufIn, uint8_t bufSize) {
  uint8_t i;

  for(i=0;i<bufSize;i++) {
    bufIn[i] = SPIWriteRead(bufOut[i]);
  }
}

static void SPIWriteBuffer(uint8_t *bufOut, uint8_t bufSize) {
	uint8_t i;

	for(i=0;i<bufSize;i++) {
		(void)SPIWriteRead(bufOut[i]);
	}
}



uint8_t nRFReadRegister(uint8_t reg) {
	uint8_t val;
	CSN_ClrVal();
	(void) SPIWriteRead(reg);
	val = SPIWriteRead(0b00000000); /* write dummy */
	CSN_SetVal();
	WAIT1_Waitus(10);
	return val;
}

void nRFTxPayload(uint8_t *payload, uint8_t payloadSize)
{
	nRFWrite(0xE1); 									// flush old data
	nRFWriteRegisterData(0xA0, payload, payloadSize); // write payload
	CE_ClrVal();											// start transmission
	WAIT1_Waitus(15); 									//* keep signal high for 15 micro-seconds
	CE_SetVal();  									// back to normal
}




void nRFWriteRegister(uint8_t reg, uint8_t val) {
  	uint8_t address = 0x20|reg;		// this will make it so it writes.. (not reads)
  	CSN_ClrVal();								// CSN High-to-Low Starts Command

    (void)SPIWriteRead(address); /* write register command */
    (void)SPIWriteRead(val); /* write value */

	CSN_SetVal();								// CSN Low to High ends command
	WAIT1_Waitus(10); 							//* insert a delay until next command
}

uint8_t nRFEnableDynamicPayloadLength(uint8_t pipeMask) {
  /* note: dynamic payload requires EN_DPL and ENAA_Px set for the pipe */
	if (pipeMask>0x3F) {
		return ERR_FAULT; /* only pipe 0 to 5 allowed */
	}
	nRFWriteRegister(0x1C, pipeMask); /* write number of RX payload for pipe */
	return ERR_OK;
}

uint8_t nRFReadNofRxPayload(uint8_t *nof) {
	*nof = nRFReadRegister(0x60); //* read number of RX payload for pipe
	return ERR_OK;
}


uint8_t nRFWriteFeature(uint8_t featureMask) {
	// nRFFEATURE_EN_DPL|nRFFEATURE_EN_ACK_PAY|nRFFEATURE_EN_DYN_PAY
	if (featureMask>((1<<2)|(1<<1)|(1<<0))) {
		return ERR_FAULT; /* mismatch of feature mask */
	}
	// 0x1D is the feature register. Enabled
	nRFWriteRegister(0x1D, featureMask); /* write number of RX payload for pipe */
	return ERR_OK;
}

uint8_t nRFSetChannel(uint8_t channel) {
	nRFWriteRegister(0x05, channel&0x7F); /* set channel */
	return ERR_OK;
}


void nRFReadRegisterData(uint8_t reg, uint8_t *buf, uint8_t bufSize)
{
	CSN_ClrVal();
	(void)SPIWriteRead(0x00|reg);
	SPIWriteReadBuffer(buf, buf, bufSize);
	CSN_SetVal();
	WAIT1_Waitus(10);
}

void nRFWriteRegisterData(byte reg, uint8_t *buf, uint8_t bufSize) {
	//   100000
	CSN_ClrVal();
	(void)SPIWriteRead(0x20|reg); /* not masking registers as it would conflict with nRFW_TX_PAYLOAD */
	SPIWriteBuffer(buf, bufSize);
	CSN_SetVal();
	WAIT1_Waitus(10);
}

void nRFResetStatusIRQ(uint8_t flags) {

	nRFWriteRegister(0x07, flags); /* reset all IRQ in status register */
}

uint8_t nRFEnableAutoAck(uint8_t pipes) {

	nRFWriteRegister(0x01, pipes&0x3F); /* enable auto acknowledge for the given pipes */
	return ERR_OK;
}

void nRFWrite(uint8_t val) {

	CSN_ClrVal();
	(void)SPIWriteRead(val);
	CSN_SetVal();
	WAIT1_Waitus(10);
}

uint8_t nRFGetFifoStatus(uint8_t *status) {
	*status = nRFReadRegister(0x17); /* read FIFO_STATUS register */
	return ERR_OK;
}

uint8_t nRFGetStatusClrIRQ(void) {
	uint8_t status;
	CSN_ClrVal();
	status = SPIWriteRead(0x20|0x07);
	(void)SPIWriteRead(status&(0x40|0x20|0x10)); //* reset IRQ Bits
	CSN_SetVal();
	WAIT1_Waitus(10);
	return status;
}

void nRFRxPayload(uint8_t *payload, uint8_t payloadSize)
{
	CE_ClrVal(); /* need to disable rx mode during reading RX data */
	nRFReadRegisterData(0x61, payload, payloadSize); /* rx payload */
	CE_SetVal(); /* re-enable rx mode */
}


static portTASK_FUNCTION(RNetTask, pvParameters) {
	uint32_t cntr;
	uint8_t msgCntr;
	(void)pvParameters;
  /*
  if (RAPP_SetThisNodeAddr(RNWK_ADDR_BROADCAST)!=ERR_OK) {
    for(;;);
  }*/

	cntr = 0; /* initialize LED counter */
	msgCntr = 0; /* initialize message counter */
 /// appState = RNETA_INITIAL; /* initialize state machine state */
	for(;;) {
		Wireless_Loop();

		// Process(); /* process state machine */

		cntr++;
		if (cntr==100) { /* with an RTOS 10 ms/100 Hz tick rate, this is every second */
			//LED3_On(); /* blink blue LED for 20 ms */
			// RAPP_SendPayloadDataBlock(&msgCntr, sizeof(msgCntr), RAPP_MSG_TYPE_PING, RNWK_ADDR_BROADCAST, RPHY_PACKET_FLAGS_NONE);
			// RAPP_SendPayloadDataBlock(&msgCntr, sizeof(msgCntr), RAPP_MSG_TYPE_PING, RNWK_ADDR_BROADCAST, RPHY_PACKET_FLAGS_NONE);
			// RAPP_SendPayloadDataBlock(&msgCntr, sizeof(msgCntr), 0x55, 0xff, 0);

			msgCntr++;
			cntr = 0;
			FRTOS1_vTaskDelay(20/portTICK_RATE_MS);
			// LED3_Off(); /* blink	 blue LED */
		}
		FRTOS1_vTaskDelay(10/portTICK_RATE_MS);
	} /* for */
}

void Wireless_Init(void){

	RMSG_MsgRxQueue = FRTOS1_xQueueCreate( 10, sizeof( struct AMessage * )  );
	if (RMSG_MsgRxQueue==NULL) { /* queue creation failed! */
		for(;;) {} /* not enough memory? */
	}

	RMSG_MsgTxQueue = FRTOS1_xQueueCreate( 10, sizeof( struct AMessage * ) );
	if (RMSG_MsgTxQueue==NULL) { /* queue creation failed! */
		for(;;) {} /* not enough memory? */
	}

  if (FRTOS1_xTaskCreate(
        RNetTask,  /* pointer to the task */
        "RNet", /* task name for kernel awareness debugging */
        configMINIMAL_STACK_SIZE, /* task stack size */
        (void*)NULL, /* optional task startup argument */
        tskIDLE_PRIORITY,  /* initial priority */
        (xTaskHandle*)NULL /* optional task handle to create */
      ) != pdPASS) {
    /*lint -e527 */
    for(;;){}; /* error! probably out of memory */
    /*lint +e527 */
  }

}

static uint8_t CheckRx(void) {
	uint8_t res = ERR_OK;
	uint8_t RxDataBuffer[2+32];
	uint8_t status;
	RPHY_PacketDesc packet;
	bool hasRxData, hasRx;

	hasRxData = FALSE;
	packet.flags = 0;
	packet.phyData = &RxDataBuffer[0];
	packet.phySize = sizeof(RxDataBuffer);
	packet.rxtx = RPHY_BUF_PAYLOAD_START(packet.phyData);
	status = nRFGetStatusClrIRQ();
	hasRx = (status&0x40)!=0;

	if (hasRx) {
		hasRxData = TRUE;

		uint8_t payloadSize = 0;

		(void)nRFReadNofRxPayload(&payloadSize);
		if (payloadSize>32) {
			nRFWrite(0xE2);
			return ERR_FAILED;
		} else {
			nRFRxPayload(packet.rxtx, payloadSize);
			RPHY_BUF_SIZE(packet.phyData) = payloadSize;
		}

	}
	if (hasRxData) {

   BaseType_t qRes = FRTOS1_xQueueSendToBack(RMSG_MsgRxQueue, packet.phyData, 200/portTICK_RATE_MS);

	int msgQueueTXamnt = FRTOS1_uxQueueMessagesWaiting(RMSG_MsgRxQueue);
	int msgQueueRXamnt = FRTOS1_uxQueueMessagesWaiting(RMSG_MsgTxQueue);


   if (qRes!=pdTRUE) {
     res = ERR_BUSY;
   }

   // } else {
    //  qRes = FRTOS1_xQueueSendToFront(RMSG_MsgRxQueue, buf, RMSG_QUEUE_PUT_WAIT);
   // }

   /*
    if (res!=ERR_OK) {
      if (res==ERR_OVERFLOW) {
        Err((unsigned char*)"ERR: Rx queue overflow!\r\n");
      } else {
        Err((unsigned char*)"ERR: Rx Queue full?\r\n");
      }
    }
    */

  } else {
    res = ERR_RXEMPTY;
  }

  return res;
}


uint8_t RADIO_PowerDown(void) {

  nRFWrite(0xE1); /* flush old data */
  nRFWrite(0xE2); /* flush old data */

  nRFWriteRegister(0x00, ((1<<3)|(1<<2)) ); //  nRFWriteRegister(nRFCONFIG, nRFCONFIG_SETTINGS)
  return ERR_OK;
}

uint8_t RMSG_GetTxMsg(uint8_t *buf, size_t bufSize) {

	int msgQueueTXamnt = FRTOS1_uxQueueMessagesWaiting(RMSG_MsgRxQueue);
	int msgQueueRXamnt = FRTOS1_uxQueueMessagesWaiting(RMSG_MsgTxQueue);

	if (bufSize<(2+32)) {
		return ERR_OVERFLOW;
	}

	if (FRTOS1_xQueueReceive(RMSG_MsgTxQueue, buf, 0)==pdPASS) {
		/* received message from queue */
		return ERR_OK;
	}
	return ERR_RXEMPTY;
}

static uint8_t CheckTx(void) {
  RPHY_PacketDesc packet;
  uint8_t res = ERR_OK;
  uint8_t TxDataBuffer[2+32]; //RPHY_BUFFER_SIZE];
  RPHY_FlagsType flags;

  if (RMSG_GetTxMsg(TxDataBuffer, sizeof(TxDataBuffer))==ERR_OK) {
    flags = RPHY_BUF_FLAGS(TxDataBuffer);
    if (flags&(1<<2)) {

      (void)RADIO_PowerDown();
      return ERR_DISABLED;
    }

    CE_ClrVal(); 	//   nRFStopRxTx();
    nRFWriteRegister(0x00, ((1<<3)|(1<<2))|(1<<1)|0  );

    packet.phyData = &TxDataBuffer[0];
    packet.flags = flags;
    packet.phySize = sizeof(TxDataBuffer);
    packet.rxtx = RPHY_BUF_PAYLOAD_START(packet.phyData);
//    if (RADIO_isSniffing) {
//      RPHY_SniffPacket(&packet, TRUE);
//    }
//   nRFTxPayload(packet.rxtx, RPHY_BUF_SIZE(packet.phyData));
    nRFWrite(0xE1); /* flush old data */
    nRFWriteRegisterData(0xA0, packet.rxtx, RPHY_BUF_SIZE(packet.phyData)); /* write payload */
    CE_ClrVal(); /* start transmission */
    WAIT1_Waitus(15); /* keep signal high for 15 micro-seconds */
    CE_SetVal();  /* back to normal */

    return ERR_OK;
  } else {
    return ERR_NOTAVAIL;
  }

  return res;
}

void registerReader() {
	uint8_t r_config = nRFReadRegister(0x00);
	uint8_t r_shockburst = nRFReadRegister(0x01);
	uint8_t r_rx = nRFReadRegister(0x02);
	uint8_t r_setup = nRFReadRegister(0x03);
	uint8_t r_autoRetransmit = nRFReadRegister(0x04);
	uint8_t r7 = nRFReadRegister(0x05);
	uint8_t r8 = nRFReadRegister(0x06);
	uint8_t r_status = nRFReadRegister(0x07);
	uint8_t r_transmitobserve = nRFReadRegister(0x08);
	uint8_t r10 = nRFReadRegister(0x09);
	uint8_t r_fifo = nRFReadRegister(0x17);
	uint8_t r_dynpd = nRFReadRegister(0x1c);
	uint8_t r_addr_TX = nRFReadRegister(0x10);
	uint8_t r_addr0_RX = nRFReadRegister(0x0A);
	uint8_t r_addr1_RX = nRFReadRegister(0x0b);
	uint8_t r_addr2_RX = nRFReadRegister(0x0c);
	uint8_t r_addr3_RX = nRFReadRegister(0x0d);
	uint8_t r_addr4_RX = nRFReadRegister(0x0e);
	uint8_t r_addr5_RX  = nRFReadRegister(0x0f);
	uint8_t r0 = nRFReadRegister(0x11);
	uint8_t r1 = nRFReadRegister(0x12);
	uint8_t r2 = nRFReadRegister(0x13);
	uint8_t r3 = nRFReadRegister(0x14);
	uint8_t r4 = nRFReadRegister(0x16);
	uint8_t r5 = nRFReadRegister(0x18);
	uint8_t r6 = nRFReadRegister(0x19);
	return;
}

void receive() {
	CSN_ClrVal();
	nRFWrite( 0b01100001 ); // rx payload
	nRFWrite(0b0); //  dummy
	nRFWrite(0b0);	//  dummy
	nRFWrite(0b0);	// dummy
	CSN_SetVal();
}

void transmit() {
	CSN_ClrVal();
	nRFWrite(0xE1); // flush tx
	CSN_SetVal();

	CSN_ClrVal();
	nRFWrite( 0b10100000 ); // regWrite( 0x10100000 )  // load TX payload command W_TX_PAYLOAD
	nRFWrite(0b1); //  byte 1 - mode
	nRFWrite(0b10);//  byte 2 - pin
	nRFWrite(0b100);//  btye 3 - value
	CSN_SetVal();

	CE_ClrVal(); // CE Low to TX mode
	WAIT1_Waitms(1); // WAIT1_Waitus(10); // Wait_ms(1) delay 1 millisecond
	nRFWriteRegister(0x00, 0b0);  // regWrite(0x00, 0b0); write 0 to bit 0 of config 0x00 reg
	CE_SetVal();

	WAIT1_Waitms(1); // Wait_ms(1)  delay 1 millisecond
	nRFWriteRegister(0x00, 0b1);// regWrite(0x00, 0b1) write 1 to bit 0 of config 0x00 which will go back to RX Mode
}



// (2+32)
void Wireless_Loop(){

	uint8_t status, res;
	static const uint8_t RADIO_TADDR[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
	uint8_t byte1 = (3<<1)| (1<<5);
	uint8_t byte2 = (1<<2)|(1<<1)|(1<<0);
	uint8_t byte3 = 0x40|0x20|0x10;
	uint8_t byte4 = ((1<<3)|(1<<2))|(1<<1)|(1<<0);

	portTickType xTime = FRTOS1_xTaskGetTickCount();

	if (xTime<(150/portTICK_RATE_MS)) {
		/* not powered for 100 ms: wait until we can access the radio transceiver */
		xTime = (150/portTICK_RATE_MS)-xTime; /* remaining ticks to wait */
		FRTOS1_vTaskDelay(xTime);
	}

	///CSN_SetVal();
	//CSN_ClrVal();
	//registerReader();
	// if the device is a transmitter CE is always low accept when the transmit happens
	//
	//
	// CE_ClrVal();   /* CE high: do not send or receive data */
	// CE_SetVal();	// think this is high for rx mode
	CE_ClrVal();
	CSN_SetVal(); /* set high when not talking to device */
	nRFWriteRegister(0x06, 0b00110  );		// 0x06  -- setup register  write 100110   0dbms  (3<<1) | (1<<5)
							// (B0100 << 4) | (B1111 << 0)
	nRFWriteFeature(  (1<<2)|(1<<1)|(1<<0) ); 										// 0x1D
	nRFEnableDynamicPayloadLength(1<<0);     									// 0x1C
	nRFSetChannel(0);															// 0x05
		// 0 - 127 (actuallu 0 - 83 legally)
	registerReader();
	nRFWriteRegisterData(0x0A, (uint8_t*) RADIO_TADDR, sizeof(RADIO_TADDR));		// 0x0A set TADDR
	nRFWriteRegisterData(0x10, (uint8_t*) RADIO_TADDR, sizeof(RADIO_TADDR));		// 0x10 set RADDR
	nRFWriteRegister(0x02, 0x01); 												// 0x02
		// pipe enable register.. seems to enable pipe 1

	nRFResetStatusIRQ(0x40|0x20|0x10);											// 0x07
	//nRFEnableAutoAck(0x01);							// 0x01
		// because auto ack is enabled receive must be the same as send.
	registerReader();
	nRFWriteRegister(0x04, 0x02|0x0F); 											// 0x04
	nRFWriteRegister(0x00, 0b1111);			// 0x00	in transmit or Power up in receiving mode((1<<3)|(1<<2))|(1<<1)|(1<<0)
		// set the config register, PWR_up, PRIM_PX=1 make a receiver
		//
	nRFGetFifoStatus(&status);
	nRFWrite(0xE2);											//* flush RX old data
	nRFWrite(0xE1); 											//* flush TX old data

	registerReader();


	CSN_ClrVal();
	(void) SPIWriteRead(0x10);
//	val = SPIWriteRead(0b00000000);
	//val = SPIWriteRead(0b00000000);
//	val = SPIWriteRead(0b00000000);
	CSN_SetVal();
	WAIT1_Waitus(10);


	// CSN Low
	// 00010000 .. send 5 dummy bytes
	// CSN High
	//nRFGetFifoStatus(&status);
	/*	CE_SetVal(); 										// Start RX TX // nRFStartRxTx(); 		//* Listening for packets  ;
	CE_ClrVal();
	nRFWrite(0b10100000);
	nRFWrite(0b10100000);
	nRFWrite(0b10100000);
	nRFWrite(0b10100000);
	CE_SetVal();*/


//	radioRx.phyData = &radioRxBuf[0];
//	radioRx.phySize = sizeof(radioRxBuf);
//	radioRx.rxtx = &RPHY_BUF_SIZE(radioRx.phyData); /* we transmit the size too */

	//while (1) {	// maybe not be a continuous loop. it should break or something

	CE_ClrVal(); 													// nRFStopRxTx();	Pull CE From High to Low
	nRFWriteRegister(0x00, ((1<<3)|(1<<2)) |(1<<1)|(1<<0)  );		// RX_POWERUP();
	CE_SetVal();          											/// nRFStartRxTx() Listening for packets : Pull CE from Low to High


	// Part to do the receiving. Should be waiting on interrupt
	res = CheckRx(); 												//* get message
	registerReader();
	nRFGetFifoStatus(&status);
	nRFWrite(0xE2);												// Flush RX
	res = CheckTx();
	registerReader();
	nRFGetStatusClrIRQ();
	//}
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  Wireless_Init();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.4 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
