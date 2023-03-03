#ifndef _TRACEALYZERSETUP_H_
#define _TRACEALYZERSETUP_H_

// #define TRACEALYZER 	1

#if defined (TRACEALYZER) && (TRACEALYZER != 0)
#define TRC_BOARD 							0		// if != 0 then tracing is enabled
#define TRC_BOARD_ISR						0
#define TRC_ANA 								0
#define TRC_ANA_CHANNEL						0
#define TRC_ANA_ISR							0
#define TRC_DIG 								0
#define TRC_DIG_ISR 							0
#define TRC_CAN 								1
#define TRC_BRUSH 							0
#define TRC_SUCTION 							0
#define TRC_LIFT 								0
#define TRC_LIFT_SHOW_MEAS_CURR 			0
#define TRC_PWM 								0
#define TRC_MOTOR 							0
#define TRC_RAMP								0
#define TRC_TMP								0
#define TRC_PID								0
#define TRC_SAFETY 							0
#define TRC_CLEAN 							1
#define TRC_CANNODE 							0
#define TRC_WATER								0

#endif

#endif /* _TRACEALYZERSETUP_H_ */
