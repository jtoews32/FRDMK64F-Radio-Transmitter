#include "Packet.h"

#ifdef __cplusplus
extern "C" {
#endif

int length(uint8_t *src) {
	int i = 0;
	while(1) {
		if(src[i]=='\0') {
			return i+1;
		}
		++i;
		if(i==32)
			break;
	}
	return 0;
}

void addPayload(uint8_t *src, uint8_t *dest, int sz) {
	int i = 0, j = 0;
	while(i < sz) {
		dest[i] = src[i];
		++i;
	}
	for(j=i; j<32; j++) {
		dest[j] = '\0';
	}
}





#ifdef __cplusplus
}  /* extern "C" */
#endif