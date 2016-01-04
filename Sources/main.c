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
