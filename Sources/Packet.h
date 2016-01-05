/*
 * Package.h
 *
 *  Created on: Jan 4, 2016
 *      Author: jtoews
 */

#ifndef PACKET_H_
#define PACKET_H_

#ifdef __cplusplus
extern "C" {
#endif

int length(uint8_t *src);
void addPayload(uint8_t *src, uint8_t *dest, int sz);







#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* PACKAGE_H_ */
