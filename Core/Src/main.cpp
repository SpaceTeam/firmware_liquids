#include "NodeInfos.h"

#include "ECU_uHb.h"
#include "ECU_Lamarr.h"

#include <STRHAL.h>
#include "PMU.h"
#include "RCU.h"
#include "RCUv2.h"
#include "LCB.h"
#include "git_version.h"

// @formatter:off
#if defined(ECU_LAMARR_BOARD)

const ChannelLookUpEntry channellookUp[]=
{
		[DEVICE_ID_FUEL_ECU_PRESSURANT_VALVE					] = { NODE_ID_LAMARR_FUEL_ECU, 		ECU_LAMARR_SERVO0},
		[DEVICE_ID_FUEL_ECU_VENT_VALVE							] = { NODE_ID_LAMARR_FUEL_ECU, 		ECU_LAMARR_IGNITER0},
		[DEVICE_ID_FUEL_ECU_TANK_PRESSURE						] = { NODE_ID_LAMARR_FUEL_ECU, 		ECU_LAMARR_PRESS0},
		[DEVICE_ID_FUEL_ECU_PRESSURANT_PRESSURE					] = { NODE_ID_LAMARR_FUEL_ECU, 		ECU_LAMARR_PRESS1},
		[DEVICE_ID_FUEL_ECU_PRESSURE_CONTROLLER					] = { NODE_ID_LAMARR_FUEL_ECU, 		ECU_LAMARR_PI_CONTROLLER},
		[DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL						] = { NODE_ID_LAMARR_FUEL_ECU, 		ECU_LAMARR_ROCKET},

		[DEVICE_ID_OX_ECU_PRESSURANT_VALVE						] = { NODE_ID_LAMARR_OX_ECU, 		ECU_LAMARR_SERVO0},
		[DEVICE_ID_OX_ECU_VENT_VALVE							] = { NODE_ID_LAMARR_OX_ECU, 		ECU_LAMARR_IGNITER0},
		[DEVICE_ID_OX_ECU_TANK_PRESSURE							] = { NODE_ID_LAMARR_OX_ECU, 		ECU_LAMARR_PRESS0},
		[DEVICE_ID_OX_ECU_PRESSURANT_PRESSURE					] = { NODE_ID_LAMARR_OX_ECU, 		ECU_LAMARR_PRESS1},
		[DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER					] = { NODE_ID_LAMARR_OX_ECU, 		ECU_LAMARR_PI_CONTROLLER},
		[DEVICE_ID_OX_ECU_ROCKET_CHANNEL						] = { NODE_ID_LAMARR_OX_ECU, 		ECU_LAMARR_ROCKET},

		[DEVICE_ID_ENGINE_ECU_FUEL_MAIN_VALVE					] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_SERVO0},
		[DEVICE_ID_ENGINE_ECU_OX_MAIN_VALVE						] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_SERVO1},
		[DEVICE_ID_ENGINE_ECU_FUEL_VENTURI_PRESSURE				] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_PRESS0},
		[DEVICE_ID_ENGINE_ECU_OX_VENTURI_PRESSURE				] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_PRESS1},
		[DEVICE_ID_ENGINE_ECU_CHAMBER_PRESSURE					] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_PRESS2},
		[DEVICE_ID_ENGINE_ECU_INTERNAL_IGNITER					] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_IGNITER0},
		[DEVICE_ID_ENGINE_ECU_ROCKET_CHANNEL					] = { NODE_ID_LAMARR_ENGINE_ECU, 	ECU_LAMARR_ROCKET},

		[DEVICE_ID_GSE_PNEU_PRESSURANT_TANKING_VALVE			] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_IGNITER0},
		[DEVICE_ID_GSE_PNEU_PRESSURANT_VENT_VALVE				] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_IGNITER1},
		[DEVICE_ID_GSE_PNEU_OX_TANKING_VALVE					] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_IGNITER2},
		[DEVICE_ID_GSE_PNEU_HOLDDOWN							] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_IGNITER3},
		[DEVICE_ID_GSE_PNEU_PRESSURANT_TANKING_VALVE_FEEDBACK	] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_PRESS0},
		[DEVICE_ID_GSE_PNEU_PRESSURANT_VENT_VALVE_FEEDBACK		] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_PRESS1},
		[DEVICE_ID_GSE_PNEU_OX_TANKING_VALVE_FEEDBACK			] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_PRESS2},
		[DEVICE_ID_GSE_PNEU_HOLDDOWN_FEEDBACK					] = { NODE_ID_LAMARR_GSE_PNEU,		ECU_LAMARR_PRESS3},
		[DEVICE_ID_GSE_ELEC_OX_DECOUPLER						] = { NODE_ID_LAMARR_GSE_ELEC,		ECU_LAMARR_IGNITER0},		//12V
		[DEVICE_ID_GSE_ELEC_STRONGBACK							] = { NODE_ID_LAMARR_GSE_ELEC,		ECU_LAMARR_IGNITER1},		//12V
		[DEVICE_ID_GSE_ELEC_DEWAR_PRESS_VALVE					] = { NODE_ID_LAMARR_GSE_ELEC,		ECU_LAMARR_IGNITER2},		//24V RBF
		[DEVICE_ID_GSE_ELEC_EXTERNAL_IGNITER					] = { NODE_ID_LAMARR_GSE_ELEC,		ECU_LAMARR_IGNITER3},		//24V RBF
		[DEVICE_ID_LCB_ROCKET_WEIGHT							] = { NODE_ID_LAMARR_LCB,			LCB_INPUT0},
		[DEVICE_ID_LCB_DEWAR_WEIGHT								] = { NODE_ID_LAMARR_LCB,			LCB_INPUT1},
		[DEVICE_ID_RCU_CAM_1									] = { NODE_ID_LAMARR_RCU_V2,		RCUv2_OUT2},
		[DEVICE_ID_RCU_CAM_2									] = { NODE_ID_LAMARR_RCU_V2,		RCUv2_OUT1}

};

#endif
// // @formatter:on

int main(void)
{

#ifdef ECU_UHB_BOARD
#ifdef UART_DEBUG
	ECU_uHb ecu(6,GIT_COMMIT_HASH_VALUE,1000); //6 ECU, 7 PMU, 9 TW
#else
	ECU_uHb ecu(6,GIT_COMMIT_HASH_VALUE, 1);
#endif

	if(ecu.init() != 0)
		return -1;

	STRHAL_UART_Debug_Write_Blocking("ECU STARTED\n", 12, 50);
	ecu.exec();
#elif defined(ECU_LAMARR_BOARD)
#ifdef UART_DEBUG
	ECU_Lamarr ecu(LAMARR_ECU_NODE_ID,GIT_COMMIT_HASH_VALUE,1000); //6 ECU, 7 PMU, 9 TW
#else
	ECU_Lamarr ecu(LAMARR_ECU_NODE_ID, GIT_COMMIT_HASH_VALUE, 1);
#endif

	if (ecu.init() != 0)
		return -1;

	STRHAL_UART_Debug_Write_Blocking("ECU STARTED\n", 12, 50);
	ecu.exec();
#elif defined(PMU_BOARD)
#ifdef UART_DEBUG
	PMU pmu(7,GIT_COMMIT_HASH_VALUE,100);
#else
	PMU pmu(7,GIT_COMMIT_HASH_VALUE,2);
#endif
	if(pmu.init() != 0)
			return -1;

	STRHAL_UART_Debug_Write_Blocking("PMU STARTED\n", 12, 50);
	pmu.exec();
#elif defined(RCU_BOARD)
#ifdef UART_DEBUG
	RCU rcu(8,GIT_COMMIT_HASH_VALUE,100);
#else
	RCU rcu(8, GIT_COMMIT_HASH_VALUE, 4);
#endif

	if (rcu.init() != 0)
		return -1;

	STRHAL_UART_Debug_Write_Blocking("RCU STARTED\n", 12, 50);
	rcu.exec();
#elif defined(RCU_V2_BOARD)
#ifdef UART_DEBUG
	RCUv2 rcu_v2(8,GIT_COMMIT_HASH_VALUE,100);
#else
	RCUv2 rcu_v2(NODE_ID_LAMARR_RCU_V2, GIT_COMMIT_HASH_VALUE, 4);
#endif

	if (rcu_v2.init() != 0)
		return -1;

	STRHAL_UART_Debug_Write_Blocking("RCU STARTED\n", 12, 50);
	rcu_v2.exec();
#elif defined(IOBv1_BOARD)
	IOBv1 iob(10,0xDEADBEEF,100); // TODO disregard node ID and read dipswitches in IOB/LCB class

	if(iob.init() != 0)
			return -1;

	STRHAL_UART_Debug_Write_Blocking("IOB STARTED\n", 12, 50);
	iob.exec();
#elif defined(IOBv3_BOARD)
#ifdef UART_DEBUG
	IOBv3 iob(10,GIT_COMMIT_HASH_VALUE,100); // TODO disregard node ID and read dipswitches in IOB/LCB class
#else
	IOBv3 iob(10,GIT_COMMIT_HASH_VALUE,1000); // TODO disregard node ID and read dipswitches in IOB/LCB class
#endif
	if(iob.init() != 0)
			return -1;

	STRHAL_UART_Debug_Write_Blocking("IOB STARTED\n", 12, 50);
	iob.exec();
#elif defined(IOBv4_BOARD)
#ifdef UART_DEBUG
	// Ox 10, Fuel 11
	IOBv4 iob(11,GIT_COMMIT_HASH_VALUE,100); // TODO disregard node ID and read dipswitches in IOB/LCB class
#else
	IOBv4 iob(11,GIT_COMMIT_HASH_VALUE,1); // TODO disregard node ID and read dipswitches in IOB/LCB class
#endif
	if(iob.init() != 0)
			return -1;

	STRHAL_UART_Debug_Write_Blocking("IOBv4 STARTED\n", 14, 50);
	iob.exec();
#elif defined(LCB_BOARD)
	LCB lcb(10,0xDEADBEEF,100);

	if(lcb.init() != 0)
			return -1;

	STRHAL_UART_Debug_Write_Blocking("LCB STARTED\n", 12, 50);
	lcb.exec();
#endif

	while (1)
		;
}

void STRHAL_OofHandler(STRHAL_Oof_t oof, char *msg)
{
	do
	{
	} while (0);
}
