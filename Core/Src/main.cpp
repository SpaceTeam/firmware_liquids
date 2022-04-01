#include <STRHAL.h>
#include "ECU.h"

#define NODE_ID 1

int main(void) {
	ECU ecu(NODE_ID,0xDEADBEEF,100);

	if(ecu.init() != 0)
		return -1;

	STRHAL_UART_Write("STARTED\n",8);
	ecu.exec();

	while(1);
}

void STRHAL_OofHandler(STRHAL_Oof_t oof, char *msg) {
	do{}while(0);
}
