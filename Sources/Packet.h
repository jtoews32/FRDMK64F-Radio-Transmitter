/*
 * Package.h
 *
 *  Created on: Jan 4, 2016
 *      Author: jtoews
 */

#ifndef PACKET_H_
#define PACKET_H_

#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t FlagsType;

typedef struct {
  FlagsType flags;/*!< flags, see RPHY_PACKET_FLAGS_XXXX above */
  uint8_t phySize;     /*!< size of PHY data buffer */
  uint8_t *phyData;    /*!< pointer to the PHY data buffer */
  uint8_t *rxtx;       /*!< pointer into phyData, start of TX/RX data */
} PacketDesc;

int length(uint8_t *src);
void addPayload(uint8_t *src, uint8_t *dest, int sz);


#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* PACKAGE_H_ */
