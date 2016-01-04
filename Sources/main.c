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

#define RF1_SPI_Enable()                   SM1_Enable()
#define RF1_SPI_Disable()                  SM1_Disable()
#define RF1_SPI_SetFastMode()              SM1_SetFastMode()
#define RF1_SPI_SetSlowMode()              SM1_SetSlowMode()
#define RF1_SPI_SetShiftClockPolarity(val) (void)SM1_SetShiftClockPolarity(val)
#define RF1_SPI_SetIdleClockPolarity(val)  (void)SM1_SetIdleClockPolarity(val)
#define RF1_SPI_GetCharsInTxBuf()          SM1_GetCharsInTxBuf()
#define RF1_SPI_GetCharsInRxBuf()          SM1_GetCharsInRxBuf()
#define RF1_SPI_SendChar(ch)               SM1_SendChar(ch)
#define RF1_SPI_RecvChar(p)                SM1_RecvChar(p)
#define RF1_SPI_SetBaudRateMode(m)         SM1_SetBaudRateMode(m)


#define RX_POWERUP()         RF1_WriteRegister(0x00, ((1<<3)|(1<<2))|(1<<1)|(1<<0))

// #define RF1_WAIT_US(x)  WAIT1_Waitus(x)          /* wait for the given number of micro-seconds */
#define RF1_WAIT_MS(x)  WAIT1_Waitms(x)          /* wait for the given number of milli-seconds */

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

uint8_t RAPP_SendPayloadDataBlock(uint8_t *appPayload,uint8_t appPayloadSize, uint8_t msgType) {//,
	//	RNWK_ShortAddrType dstAddr,
	//	RPHY_FlagsType flags

	return 0;
}

static uint8_t SPIWriteRead(uint8_t val) {
  	uint8_t ch;

  	while(RF1_SPI_GetCharsInTxBuf()!=0) {} /* wait until tx is empty */
  	while(RF1_SPI_SendChar(val)!=ERR_OK) {} /* send character */
  	while(RF1_SPI_GetCharsInTxBuf()!=0) {} /* wait until data has been sent */
  	while(RF1_SPI_GetCharsInRxBuf()==0) {} /* wait until we receive data */
  	while(RF1_SPI_RecvChar(&ch)!=ERR_OK) {} /* get data */
  	return ch;
}

uint8_t RF1_ReadRegister(uint8_t reg) {
	uint8_t val;
	CSN_ClrVal();
	(void) SPIWriteRead(reg);
	val = SPIWriteRead(0b00000000); /* write dummy */
	CSN_SetVal();
	WAIT1_Waitus(10);
	return val;
}

void RF1_TxPayload(uint8_t *payload, uint8_t payloadSize)
{
	RF1_Write(0xE1); 									// flush old data
	RF1_WriteRegisterData(0xA0, payload, payloadSize); // write payload
	CE_ClrVal();											// start transmission
	WAIT1_Waitus(15); 									//* keep signal high for 15 micro-seconds
	CE_SetVal();  									// back to normal
}

/*
uint8_t val;
CSN_ClrVal();
(void) SPIWriteRead(0x0A);
val = SPIWriteRead(0b00000000);
val = SPIWriteRead(0b00000000);
val = SPIWriteRead(0b00000000);
CSN_SetVal();
WAIT1_Waitus(10);
*/


static void SPIWriteBuffer(uint8_t *bufOut, uint8_t bufSize) {
	uint8_t i;

	for(i=0;i<bufSize;i++) {
		(void)SPIWriteRead(bufOut[i]);
	}
}


void RF1_WriteRegister(uint8_t reg, uint8_t val) {
  	uint8_t status, address = 0x20|reg;
  	CSN_ClrVal();								// CSN High-to-Low Starts Command

  	while(RF1_SPI_GetCharsInTxBuf()!=0) 	{} 	//* wait until tx is empty
  	while(RF1_SPI_SendChar(address)!=ERR_OK) 	{} 	//* send character
  	while(RF1_SPI_GetCharsInTxBuf()!=0) 	{} 	//* wait until data has been sent
  	while(RF1_SPI_GetCharsInRxBuf()==0) 	{} 	//* wait until we receive data
  	while(RF1_SPI_RecvChar(&status)!=ERR_OK) 	{} 	//* get data

  	while(RF1_SPI_GetCharsInTxBuf()!=0) 	{} 	//* wait until tx is empty
  	while(RF1_SPI_SendChar(val)!=ERR_OK) 	{} 	//* send character
  	while(RF1_SPI_GetCharsInTxBuf()!=0) 	{} 	//* wait until data has been sent
  	while(RF1_SPI_GetCharsInRxBuf()==0) 	{} 	//* wait until we receive data
  	while(RF1_SPI_RecvChar(&status)!=ERR_OK) 	{} 	//* get data

	CSN_SetVal();								// CSN Low to High ends command
	WAIT1_Waitus(10); 							//* insert a delay until next command
}

uint8_t RF1_EnableDynamicPayloadLength(uint8_t pipeMask) {
  /* note: dynamic payload requires EN_DPL and ENAA_Px set for the pipe */
	if (pipeMask>0x3F) {
		return ERR_FAULT; /* only pipe 0 to 5 allowed */
	}
	RF1_WriteRegister(0x1C, pipeMask); /* write number of RX payload for pipe */
	return ERR_OK;
}

uint8_t RF1_ReadNofRxPayload(uint8_t *nof)
{
	*nof = RF1_ReadRegister(0x60); //* read number of RX payload for pipe
	return ERR_OK;
}


uint8_t RF1_WriteFeature(uint8_t featureMask) {
	// RF1_FEATURE_EN_DPL|RF1_FEATURE_EN_ACK_PAY|RF1_FEATURE_EN_DYN_PAY
	if (featureMask>((1<<2)|(1<<1)|(1<<0))) {
		return ERR_FAULT; /* mismatch of feature mask */
	}
	// 0x1D is the feature register. Enabled
	RF1_WriteRegister(0x1D, featureMask); /* write number of RX payload for pipe */
	return ERR_OK;
}

uint8_t RF1_SetChannel(uint8_t channel) {
	RF1_WriteRegister(0x05, channel&0x7F); /* set channel */
	return ERR_OK;
}

static void SPIWriteReadBuffer(uint8_t *bufOut, uint8_t *bufIn, uint8_t bufSize) {
  uint8_t i;

  for(i=0;i<bufSize;i++) {
    bufIn[i] = SPIWriteRead(bufOut[i]);
  }
}

void RF1_ReadRegisterData(uint8_t reg, uint8_t *buf, uint8_t bufSize)
{
	CSN_ClrVal();
	(void)SPIWriteRead(0x00|reg);
	SPIWriteReadBuffer(buf, buf, bufSize);
	CSN_SetVal();
	WAIT1_Waitus(10);
}

void RF1_WriteRegisterData(byte reg, uint8_t *buf, uint8_t bufSize) {
	//   100000
	CSN_ClrVal();
	(void)SPIWriteRead(0x20|reg); /* not masking registers as it would conflict with RF1_W_TX_PAYLOAD */
	SPIWriteBuffer(buf, bufSize);
	CSN_SetVal();
	WAIT1_Waitus(10);
}

void RF1_ResetStatusIRQ(uint8_t flags) {

	RF1_WriteRegister(0x07, flags); /* reset all IRQ in status register */
}

uint8_t RF1_EnableAutoAck(uint8_t pipes) {

	RF1_WriteRegister(0x01, pipes&0x3F); /* enable auto acknowledge for the given pipes */
	return ERR_OK;
}

void RF1_Write(uint8_t val) {

	CSN_ClrVal();
	(void)SPIWriteRead(val);
	CSN_SetVal();
	WAIT1_Waitus(10);
}

uint8_t RF1_GetFifoStatus(uint8_t *status) {
	*status = RF1_ReadRegister(0x17); /* read FIFO_STATUS register */
	return ERR_OK;
}

uint8_t RF1_GetStatusClrIRQ(void) {
	uint8_t status;
	CSN_ClrVal();
	status = SPIWriteRead(0x20|0x07);
	(void)SPIWriteRead(status&(0x40|0x20|0x10)); //* reset IRQ Bits
	CSN_SetVal();
	WAIT1_Waitus(10);
	return status;
}

void RF1_RxPayload(uint8_t *payload, uint8_t payloadSize)
{
	CE_ClrVal(); /* need to disable rx mode during reading RX data */
	RF1_ReadRegisterData(0x61, payload, payloadSize); /* rx payload */
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

	int msgQueueTXamnt = FRTOS1_uxQueueMessagesWaiting(RMSG_MsgRxQueue);
	int msgQueueRXamnt = FRTOS1_uxQueueMessagesWaiting(RMSG_MsgTxQueue);

	/*
RSTACK_Init();
if (RAPP_SetMessageHandlerTable(handlerTable)!=ERR_OK) {
  for(;;);
}*/

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
	status = RF1_GetStatusClrIRQ();
	hasRx = (status&0x40)!=0;

	if (hasRx) {
		hasRxData = TRUE;

		uint8_t payloadSize = 0;

		(void)RF1_ReadNofRxPayload(&payloadSize);
		if (payloadSize>32) {
			RF1_Write(0xE2);
			return ERR_FAILED;
		} else {
			RF1_RxPayload(packet.rxtx, payloadSize);
			RPHY_BUF_SIZE(packet.phyData) = payloadSize;
		}

	}
	if (hasRxData) {

	//  uint8_t res = ERR_OK; // RMSG_QueueRxMsg(packet.phyData, packet.phySize, RPHY_BUF_SIZE(packet.phyData), packet.flags);
  //  queue = RMSG_MsgTxQueue;
 // } else {
 //   queue = RMSG_MsgRxQueue;
   // if (toBack) {

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

  RF1_Write(0xE1); /* flush old data */
  RF1_Write(0xE2); /* flush old data */

  RF1_WriteRegister(0x00, ((1<<3)|(1<<2)) ); //  RF1_WriteRegister(RF1_CONFIG, RF1_CONFIG_SETTINGS)
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

    CE_ClrVal(); 	//   RF1_StopRxTx();
    RF1_WriteRegister(0x00, ((1<<3)|(1<<2))|(1<<1)|0  );

    packet.phyData = &TxDataBuffer[0];
    packet.flags = flags;
    packet.phySize = sizeof(TxDataBuffer);
    packet.rxtx = RPHY_BUF_PAYLOAD_START(packet.phyData);
//    if (RADIO_isSniffing) {
//      RPHY_SniffPacket(&packet, TRUE);
//    }
//   RF1_TxPayload(packet.rxtx, RPHY_BUF_SIZE(packet.phyData));
    RF1_Write(0xE1); /* flush old data */
    RF1_WriteRegisterData(0xA0, packet.rxtx, RPHY_BUF_SIZE(packet.phyData)); /* write payload */
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
	uint8_t r_config = RF1_ReadRegister(0x00);
	uint8_t r_shockburst = RF1_ReadRegister(0x01);
	uint8_t r_rx = RF1_ReadRegister(0x02);
	uint8_t r_setup = RF1_ReadRegister(0x03);
	uint8_t r_autoRetransmit = RF1_ReadRegister(0x04);
	uint8_t r7 = RF1_ReadRegister(0x05);
	uint8_t r8 = RF1_ReadRegister(0x06);
	uint8_t r_status = RF1_ReadRegister(0x07);
	uint8_t r_transmitobserve = RF1_ReadRegister(0x08);
	uint8_t r10 = RF1_ReadRegister(0x09);
	uint8_t r_fifo = RF1_ReadRegister(0x17);
	uint8_t r_dynpd = RF1_ReadRegister(0x1c);
	uint8_t r_addr_TX = RF1_ReadRegister(0x10);
	uint8_t r_addr0_RX = RF1_ReadRegister(0x0A);
	uint8_t r_addr1_RX = RF1_ReadRegister(0x0b);
	uint8_t r_addr2_RX = RF1_ReadRegister(0x0c);
	uint8_t r_addr3_RX = RF1_ReadRegister(0x0d);
	uint8_t r_addr4_RX = RF1_ReadRegister(0x0e);
	uint8_t r_addr5_RX  = RF1_ReadRegister(0x0f);
	uint8_t r0 = RF1_ReadRegister(0x11);
	uint8_t r1 = RF1_ReadRegister(0x12);
	uint8_t r2 = RF1_ReadRegister(0x13);
	uint8_t r3 = RF1_ReadRegister(0x14);
	uint8_t r4 = RF1_ReadRegister(0x16);
	uint8_t r5 = RF1_ReadRegister(0x18);
	uint8_t r6 = RF1_ReadRegister(0x19);
	return;
}

void receive() {
	CSN_ClrVal();
	RF1_Write( 0b01100001 ); // rx payload
	RF1_Write(0b0); //  dummy
	RF1_Write(0b0);	//  dummy
	RF1_Write(0b0);	// dummy
	CSN_SetVal();
}

void transmit() {
	CSN_ClrVal();
	RF1_Write(0xE1); // flush tx
	CSN_SetVal();

	CSN_ClrVal();
	RF1_Write( 0b10100000 ); // regWrite( 0x10100000 )  // load TX payload command W_TX_PAYLOAD
	RF1_Write(0b1); //  byte 1 - mode
	RF1_Write(0b10);//  byte 2 - pin
	RF1_Write(0b100);//  btye 3 - value
	CSN_SetVal();

	CE_ClrVal(); // CE Low to TX mode
	WAIT1_Waitms(1); // WAIT1_Waitus(10); // Wait_ms(1) delay 1 millisecond
	RF1_WriteRegister(0x00, 0b0);  // regWrite(0x00, 0b0); write 0 to bit 0 of config 0x00 reg
	CE_SetVal();

	WAIT1_Waitms(1); // Wait_ms(1)  delay 1 millisecond
	RF1_WriteRegister(0x00, 0b1);// regWrite(0x00, 0b1) write 1 to bit 0 of config 0x00 which will go back to RX Mode
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
	RF1_WriteRegister(0x06, 0b00110  );		// 0x06  -- setup register  write 100110   0dbms  (3<<1) | (1<<5)
							// (B0100 << 4) | (B1111 << 0)
	RF1_WriteFeature(  (1<<2)|(1<<1)|(1<<0) ); 										// 0x1D
	RF1_EnableDynamicPayloadLength(1<<0);     									// 0x1C
	RF1_SetChannel(0);															// 0x05
		// 0 - 127 (actuallu 0 - 83 legally)
	registerReader();
	RF1_WriteRegisterData(0x0A, (uint8_t*) RADIO_TADDR, sizeof(RADIO_TADDR));		// 0x0A set TADDR
	RF1_WriteRegisterData(0x10, (uint8_t*) RADIO_TADDR, sizeof(RADIO_TADDR));		// 0x10 set RADDR
	RF1_WriteRegister(0x02, 0x01); 												// 0x02
		// pipe enable register.. seems to enable pipe 1

	RF1_ResetStatusIRQ(0x40|0x20|0x10);											// 0x07
	//RF1_EnableAutoAck(0x01);							// 0x01
		// because auto ack is enabled receive must be the same as send.
	registerReader();
	RF1_WriteRegister(0x04, 0x02|0x0F); 											// 0x04
	RF1_WriteRegister(0x00, 0b1111);			// 0x00	in transmit or Power up in receiving mode((1<<3)|(1<<2))|(1<<1)|(1<<0)
		// set the config register, PWR_up, PRIM_PX=1 make a receiver
		//
	RF1_GetFifoStatus(&status);
	RF1_Write(0xE2);											//* flush RX old data
	RF1_Write(0xE1); 											//* flush TX old data

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
	//RF1_GetFifoStatus(&status);
	/*	CE_SetVal(); 										// Start RX TX // RF1_StartRxTx(); 		//* Listening for packets  ;
	CE_ClrVal();
	RF1_Write(0b10100000);
	RF1_Write(0b10100000);
	RF1_Write(0b10100000);
	RF1_Write(0b10100000);
	CE_SetVal();*/


//	radioRx.phyData = &radioRxBuf[0];
//	radioRx.phySize = sizeof(radioRxBuf);
//	radioRx.rxtx = &RPHY_BUF_SIZE(radioRx.phyData); /* we transmit the size too */

	//while (1) {	// maybe not be a continuous loop. it should break or something

	CE_ClrVal(); 													// RF1_StopRxTx();	Pull CE From High to Low
	RF1_WriteRegister(0x00, ((1<<3)|(1<<2)) |(1<<1)|(1<<0)  );		// RX_POWERUP();
	CE_SetVal();          											/// RF1_StartRxTx() Listening for packets : Pull CE from Low to High


	// Part to do the receiving. Should be waiting on interrupt
	res = CheckRx(); 												//* get message
	registerReader();
	RF1_GetFifoStatus(&status);
	RF1_Write(0xE2);												// Flush RX
	res = CheckTx();
	registerReader();
	RF1_GetStatusClrIRQ();
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



//static void RADIO_HandleStateMachine(void) {
//  uint8_t status, res;
//
//  RF1_StopRxTx();
//  RX_POWERUP();
//  RF1_StartRxTx();
//
//  (void)RF1_GetFifoStatus(&status);
//
//  if (res==ERR_RXEMPTY && !(status & RF1_FIFO_STATUS_RX_EMPTY)) { /* no data, but still flag set? */
//    RF1_Write(RF1_FLUSH_RX); /* flush old data */
//  //  RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* continue listening */
//  } else if (!(status&RF1_FIFO_STATUS_RX_EMPTY) || (status&RF1_FIFO_STATUS_RX_FULL)) { /* Rx not empty? */
//   // RADIO_isrFlag = TRUE; /* stay in current state */
//  } else {
//    // RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* continue listening */
//  }


//  res = CheckTx();
//  if (res==ERR_OK) { /* there was data and it has been sent */
//   // RADIO_AppStatus = RADIO_WAITING_DATA_SENT;
//   // break; /* process switch again */
//  } else if (res==ERR_DISABLED) { /* powered down transceiver */
//  //  RADIO_AppStatus = RADIO_POWER_DOWN;
//  } else {
//  //  RADIO_AppStatus = RADIO_READY_FOR_TX_RX_DATA;
//  }



//  for(;;) { /* will break/return */
//    switch (RADIO_AppStatus) {
//     case RADIO_INITIAL_STATE:
     //   RF1_StopRxTx();  /* will send/receive data later */
     //   RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* turn receive on */
     //   break; /* process switch again */

//     case RADIO_RECEIVER_ALWAYS_ON: /* turn receive on */
      //  RX_POWERUP();
      //  RF1_StartRxTx(); /* Listening for packets */
      //  RADIO_AppStatus = RADIO_READY_FOR_TX_RX_DATA;
      //  break; /* process switch again */

//      case RADIO_READY_FOR_TX_RX_DATA: /* we are ready to receive/send data data */
//#if !RF1_IRQ_PIN_ENABLED
//        RF1_PollInterrupt();
//#if 1 /* experimental */
 //       if (!RADIO_isrFlag) { /* interrupt flag not set, check if we have otherwise data */
 //         (void)RF1_GetFifoStatus(&status);
 //         if (!(status&RF1_FIFO_STATUS_RX_EMPTY) || (status&RF1_FIFO_STATUS_RX_FULL)) { /* Rx not empty? */
 //           RADIO_isrFlag = TRUE;
 //         }
 //       }
//#endif
//#endif
//        if (RADIO_isrFlag) { /* Rx interrupt? */
//          RADIO_isrFlag = FALSE; /* reset interrupt flag */
//          res = CheckRx(); /* get message */
//
//          (void)RF1_GetFifoStatus(&status);
//          if (res==ERR_RXEMPTY && !(status&RF1_FIFO_STATUS_RX_EMPTY)) { /* no data, but still flag set? */
//            RF1_Write(RF1_FLUSH_RX); /* flush old data */
//            RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* continue listening */
//          } else if (!(status&RF1_FIFO_STATUS_RX_EMPTY) || (status&RF1_FIFO_STATUS_RX_FULL)) { /* Rx not empty? */
//            RADIO_isrFlag = TRUE; /* stay in current state */
//          } else {
//            RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* continue listening */
//          }
//
//
//          break; /* process switch again */
//        }
//#if RNET_CONFIG_SEND_RETRY_CNT>0
//        RADIO_RetryCnt=0;
//#endif
//        RADIO_AppStatus = RADIO_CHECK_TX; /* check if we can send something */
 //       break;

//      case RADIO_CHECK_TX:
//        res = CheckTx();
//        if (res==ERR_OK) { /* there was data and it has been sent */
//          RADIO_AppStatus = RADIO_WAITING_DATA_SENT;
//          break; /* process switch again */
//        } else if (res==ERR_DISABLED) { /* powered down transceiver */
//          RADIO_AppStatus = RADIO_POWER_DOWN;
//        } else {
//          RADIO_AppStatus = RADIO_READY_FOR_TX_RX_DATA;
//        }
 //       return;
//
//      case RADIO_POWER_DOWN:
//        return;
//
//      case RADIO_WAITING_DATA_SENT:
//#if !RF1_IRQ_PIN_ENABLED
//        RF1_PollInterrupt();
//#endif
//#if 1 /* experimental */
//        if (!RADIO_isrFlag) { /* check if we missed an interrupt? */
//          RF1_PollInterrupt();
//        }
//#endif
//        if (RADIO_isrFlag) { /* check if we have received an interrupt: this is either timeout or low level ack */
//          RADIO_isrFlag = FALSE; /* reset interrupt flag */
//          status = RF1_GetStatusClrIRQ();
//          if (status&RF1_STATUS_MAX_RT) { /* retry timeout interrupt */
//            RF1_Write(RF1_FLUSH_TX); /* flush old data */
//            RADIO_AppStatus = RADIO_TIMEOUT; /* timeout */
//          } else {
//    #if RNET1_CREATE_EVENTS
//            RNET1_OnRadioEvent(RNET1_RADIO_MSG_SENT);
//    #endif
//            RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* turn receive on */
//          }
//          break; /* process switch again */
//        }
//        return;
//
//      case RADIO_TIMEOUT:
//#if RNET_CONFIG_SEND_RETRY_CNT>0
//        if (RADIO_RetryCnt<RNET_CONFIG_SEND_RETRY_CNT) {
//          Err((unsigned char*)"ERR: Retry\r\n");
//  #if RNET1_CREATE_EVENTS
//          RNET1_OnRadioEvent(RNET1_RADIO_RETRY);
//  #endif
//          RADIO_RetryCnt++;
//          if (RMSG_PutRetryTxMsg(TxDataBuffer, sizeof(TxDataBuffer))==ERR_OK) {
//            RADIO_AppStatus = RADIO_CHECK_TX; /* resend packet */
//            return; /* iterate state machine next time */
//          } else {
//            Err((unsigned char*)"ERR: PutRetryTxMsg failed!\r\n");
//  #if RNET1_CREATE_EVENTS
//            RNET1_OnRadioEvent(RNET1_RADIO_RETRY_MSG_FAILED);
//  #endif
//          }
//        }
//#endif
//        Err((unsigned char*)"ERR: Timeout\r\n");
//#if RNET1_CREATE_EVENTS
//        RNET1_OnRadioEvent(RNET1_RADIO_TIMEOUT);
//#endif
//        RADIO_AppStatus = RADIO_RECEIVER_ALWAYS_ON; /* turn receive on */
//        break; /* process switch again */
//
//      default: /* should not happen! */
//        return;
//    } /* switch */
//  } /* for */
//}
//uint8_t RNWK_OnPacketRx(RPHY_PacketDesc *packet) {
//  RNWK_ShortAddrType addr;
//  RMAC_MsgType type;
//
//  addr = RNWK_BUF_GET_DST_ADDR(packet->phyData);
//  if (addr==RNWK_ADDR_BROADCAST || addr==RNWK_GetThisNodeAddr()) { /* it is for me :-) */
//    type = RMAC_GetType(packet->phyData, packet->phySize); /* get the type of the message */
//    if (RMAC_MSG_TYPE_IS_ACK(type) && RMAC_IsExpectedACK(packet->phyData, packet->phySize)) {
//      /* it is an ACK, and the sequence number matches. Mark it with a flag and return, as no need for further processing */
//      packet->flags |= RPHY_PACKET_FLAGS_IS_ACK;
//      return ERR_OK; /* no need to process the packet further */
//    } else if (RMAC_MSG_TYPE_IS_DATA(type)) { /* data packet received */
//      if (RNWK_AppOnRxCallback!=NULL) { /* do we have a callback? */
//#if RNET_CONFIG_USE_ACK
//        if (RMAC_MSG_TYPE_REQ_ACK(type)) {
//          (void)RNWK_SendACK(packet, RNWK_GetThisNodeAddr()); /* send ack message back */
//        }
//#endif
//        return RNWK_AppOnRxCallback(packet); /* call upper layer */
//      }
//    } else {
//      return ERR_FAULT; /* wrong message type? */
//    }
//  }
//  return ERR_FAILED;
//}




/*
static uint8_t RNETA_HandleRxMessage(RAPP_MSG_Type type, uint8_t size, uint8_t *data, RNWK_ShortAddrType srcAddr, bool *handled, RPHY_PacketDesc *packet) {
  (void)srcAddr;
  (void)packet;
  switch(type) {
    case RAPP_MSG_TYPE_PING: //* <type><size><data
      *handled = TRUE;
      //* to be defined: do something with the ping, e.g blink a LED
      LED2_On();
      FRTOS1_vTaskDelay(20/portTICK_RATE_MS);
      LED2_Off();
      return ERR_OK;
    default:
      break;
  }
  return ERR_OK;
}

static void Process(void) {
  for(;;) {
    switch(appState) {
    case RNETA_INITIAL:
      appState = RNETA_POWERUP;
      continue;

    case RNETA_POWERUP:
      RadioPowerUp();
      appState = RNETA_TX_RX;
      break;

    case RNETA_TX_RX:
      (void)RADIO_Process();
      break;

    default:
      break;
    }
    break;
  }
}
*/

/*
typedef enum {
  RNET1_RADIO_MSG_RECEIVED,        //* message has been received
  RNET1_RADIO_MSG_SENT,            //* message has been sent
  RNET1_RADIO_TIMEOUT,             //* timeout, no response received
  RNET1_RADIO_RETRY,               //* retry, sending message again
  RNET1_RADIO_RETRY_MSG_FAILED,    //* creating retry message failed
  RNET1_RADIO_ACK_RECEIVED         //* acknowledge message received
} RNET1_RadioEvent;
*/
/* User includes (#include below this line is not maintained by Processor Expert) */
/*
typedef enum {
  RNETA_INITIAL, //* initialization state
  RNETA_POWERUP, //* powered up the transceiver
  RNETA_TX_RX //* ready to send and receive data
} RNETA_State;
*/
/*
struct AMessage
 {
    char ucMessageID;
    char ucData[ 20 ];
 };
*/


void receive1() {
	// CSN low
	// 01100001 read rx payload
	// 00000000 data
	// 00000000 data
	// 00000000 data
	// CSN High
}

void transmit1() {
	// CSN Low
	// flush TX
	// CSN High

	// CSN low
	// 0x10100000  // load TX payload command W_TX_PAYLOAD
	//  byte 1 - mode
	//  byte 2 - pin
	//  btye 3 - value
	// CSN High

	// CE Low to tx TX mode
	// delay 1 millisecond
	// write 0 to bit 0 of config 0x00 reg
	// CE High

	// delay 1 millisecond
	// write 1 to bit 0 of conig 0x00 which will go back to RX Mode
}
