/*
 * CommandDefs.h
 *
 *  Created on: Aug 12, 2020
 *      Author: martin
 */

#ifndef COMMANDDEFS_H_
#define COMMANDDEFS_H_

#include <stdint.h>

// Command Errors
#define CMD_NO_COMMAND_AVAILABLE          0
#define CMD_OK                            1
#define CMD_ERR_COMMAND_PENDING           -1
#define CMD_ERR_INVALID_LENGTH            -2
#define CMD_ERR_INVALID_PACKET            -3
#define CMD_ERR_INVALID_OW_DEVICE         -4
#define CMD_ERR_INVALID_OW_RESPONSE       -5
#define CMD_ERR_UNKNOWN_CMD               -6
#define CMD_ERR_UNKNOWN_SUBCMD            -7
#define CMD_ERR_COMMAND_FAILED            -8
#define CMD_ERR_INVALID_ADDRESS           -9
#define CMD_ERR_BUFFER_OVERFLOW           -10
#define CMD_ERROR_MUTEX_SYNC              -11
#define CMD_ERR_NO_MORE_DEVICES           -12

#define CMD_RETRY                         10
#define CMD_RETRY_WAIT                    50

#define CMD_ACK                           0x01                 //!< COMMAND: Acknowledge
#define CMD_NAK                           0x02                 //!< COMMAND: No Acknowledge

#define CMD_NONE                          0x00                 //!< COMMAND: Unknown command
#define CMD_TX                            0x01                 //!< COMMAND: TX command, no responsed besides ACK/NAK
#define CMD_RX                            0x02                 //!< COMMAND: RX command, ther is a responsed besides ACK/NAK

#define CMD_NO_CMD                        0x00                 //!< COMMAND: No Command specified
#define CMD_NO_SUB                        0x00                 //!< COMMAND: No Subcommand specified

// Main Commands
#define CMD_INFO                          0x01                 //!< COMMAND: Info Command
#define CMD_SYSTEM                        0x02                 //!< COMMAND: System Command
#define CMD_MEASUREMENT                   0x03                 //!< COMMAND: Measurement Command
#define CMD_DEVICE								0x04						//!< COMMAND: Device Command

// System Subcommands
#define SUB_SYS_SET_DEVICE_OWN_ADDR			0x01						//!< SUBCOMMAND: Set the Address of the Board
#define SUB_SYS_REQUEST_PING              0x02                 //!< SUBCOMMAND: Request ping command
#define SUB_SYS_GET_DEVICE_TYPE				0x03                 //!< SUBCOMMAND: Get the Device Type
#define SUB_SYS_GET_CLOCKS					 	0x04						//!< SUBCOMMAND: Get All System Clocks

#define SUB_SYS_SET_DIG_VALUE					0x20						//!< SUBCOMMAND: Set a digital output port
#define SUB_SYS_SET_DIG_SECURITY_VALUE		0x21						//!< SUBCOMMAND: Set a digital security port
#define SUB_SYS_SET_TEST_MUX					0x22						//!< SUBCOMMAND: Set a Test Multiplexer
#define SUB_SYS_SET_GPIO						0x23						//!< SUBCOMMAND: Set the value of a GPIO
#define SUB_SYS_SET_PWM_CONTROL				0x24						//!< SUBCOMMAND: Set the values of a PWM channel
#define SUB_SYS_SET_CAN_PARAMETERS			0x25						//!< SUBCOMMAND: Set the CAN parameters
#define SUB_SYS_SET_CAN_ACCEPTANCE			0x26						//!< SUBCOMMAND: Set the CAN acceptance filter
#define SUB_SYS_SET_CAN_ID						0x27						//!< SUBCOMMAND: Set the CAN ID
#define SUB_SYS_CLEAR_ERRORS					0x28						//!< SUBCOMMAND: Clears the CAN error flags
#define SUB_SYS_SEND_CAN_MESSAGE				0x29						//!< SUBCOMMAND: Sends a CAN message
#define SUB_SYS_SET_CAN_BAUDRATE				0x2A						//!< SUBCOMMAND: Set the CAN baudrate
#define SUB_SYS_SET_SECURITY_GPIOS			0x2B						//!< SUBCOMMAND: Set the CAN baudrate
#define SUB_SYS_SET_ALL_CUC_TASKS_ENA		0x2C						//!< SUBCOMMAND: Resume or Suspend all CUC Tasks
#define SUB_SYS_SET_TEST_EXT_WD				0x2D						//!< SUBCOMMAND: Enables or disables the External Watchdog Test
#define SUB_SYS_SET_PUMP_PULSE				0x2E						//!< SUBCOMMAND: Sets the Frequency and Pulse Duration for a Pump
#define SUB_SYS_SET_PUMP_ENABLE				0x2F						//!< SUBCOMMAND: Sets the On/Off-Status of a Pump
#define SUB_SYS_RESTART_SAFETYMNGR			0x30						//!< SUBCOMMAND: Restarts the Safety Manager Task
#define SUB_SYS_SET_SAFETYMNGR_CHECKS		0x31						//!< SUBCOMMAND: Enables / Disables the Safety Manager Checks
#define SUB_SYS_SET_LIFT_STATUS				0x32						//!< SUBCOMMAND: Sets the status of a Lift Device
#define SUB_SYS_SET_VB_ENABLE_STATUS		0x33						//!< SUBCOMMAND: Sets the Status of the VB Enable Lines (VB Enable 1/2, Charge VB)
#define SUB_SYS_SET_UNBLOCK_STATUS			0x34						//!< SUBCOMMAND: Sets the Status of the Unblock Lines (Unblock-L-BR, Unblock-L-Suc)
#define SUB_SYS_SWITCH_RELAY					0x35						//!< SUBCOMMAND: Switches a Relay On or Off
#define SUB_SYS_ENA_LIFT_HALL_CNTR			0x36						//!< SUBCOMMAND: Enables the Lift Motor Hall Counters for Debugging
#define SUB_SYS_RESET_LIFT_HALL_CNTR		0x37						//!< SUBCOMMAND: Resets the Lift Motor Hall Counters for Debugging
#define SUB_SYS_ENA_FLOWMETER_CNTR			0x38						//!< SUBCOMMAND: Enables the Flow Meter Counter for Debugging
#define SUB_SYS_RESET_FLOWMETER_CNTR		0x39						//!< SUBCOMMAND: Resets the Flow Meter Counter for Debugging
#define SUB_SYS_RESET_ENDSWITCH_STATE		0x3A						//!< SUBCOMMAND: Resets the End Switch State of the Lift Devices
#define SUB_SYS_RESET_CLMGR_FSM_ERR			0x3B						//!< SUBCOMMAND: Resets the Cleaning Manager FSM Error State
#define SUB_SYS_SET_PUMP_CL_MGR				0x3C						//!< SUBCOMMAND: Sets the Cleaning Manager Waterpump Parameters
#define SUB_SYS_SEND_CL_MGR_CMD				0x3D						//!< SUBCOMMAND: Sends a Command to the Cleaning Manager
#define SUB_SYS_DISABLE_LIFT_ADJUST			0x3E						//!< SUBCOMMAND: Disables or Enables the Lift Device Adjust
#define SUB_SYS_SET_LIFT_MAX_CURRENT      0x3F						//!< SUBCOMMAND: Sets the Maximum Current for the lifts

#define SUB_SYS_GET_ADC_VALUE				 	0x40						//!< SUBCOMMAND: Get the actual value of an ADC channel
#define SUB_SYS_GET_ADC_VALUE_FLOAT		 	0x41						//!< SUBCOMMAND: Get the actual value of an ADC channel as floating point values
#define SUB_SYS_GET_ALL_ADC_VALUES			0x42						//!< SUBCOMMAND: Get all actual ADC values
#define SUB_SYS_GET_ALL_ADC_VAL_FLOAT		0x43						//!< SUBCOMMAND: Get all actual ADC values as floating point values
#define SUB_SYS_GET_DIG_VALUE					0x44						//!< SUBCOMMAND: Get a digital input port
#define SUB_SYS_GET_DIG_READBACK				0x45						//!< SUBCOMMAND: Readback a digital output port
#define SUB_SYS_GET_ALL_DIG_VALUES			0x46						//!< SUBCOMMAND: Get all digital values
#define SUB_SYS_GET_ALL_DIG_READBACKS		0x47						//!< SUBCOMMAND: Get all digital readback values
#define SUB_SYS_GET_DIG_SECURITY_VALUE		0x48						//!< SUBCOMMAND: Get a digital security port value
#define SUB_SYS_GET_ALL_DIG_SEC_VALUES		0x49						//!< SUBCOMMAND: Get a digital security port value
#define SUB_SYS_GET_TEST_MUX					0x4A						//!< SUBCOMMAND: Get the setting of the Test Multiplexer
#define SUB_SYS_GET_GPIO						0x4B						//!< SUBCOMMAND: Get the value of a GPIO
#define SUB_SYS_GET_PWM_CONTROL				0x4C						//!< SUBCOMMAND: Get the values of a PWM channel
#define SUB_SYS_GET_ALL_DIG_IOS				0x4D						//!< SUBCOMMAND: Get the values of all digital IOs
#define SUB_SYS_GET_CAN_PARAMETERS			0x4E						//!< SUBCOMMAND: Get the CAN parameters
#define SUB_SYS_GET_CAN_ACCEPTANCE			0x4F						//!< SUBCOMMAND: Get the CAN acceptance filter
#define SUB_SYS_GET_CAN_ID						0x50						//!< SUBCOMMAND: Get the CAN ID
#define SUB_SYS_GET_CAN_ERROR_FLAGS			0x51						//!< SUBCOMMAND: Get the CAN Error Flags
#define SUB_SYS_GET_CAN_N_RX_MSGS			0x52						//!< SUBCOMMAND: Get the number of CAN RX messages available
#define SUB_SYS_GET_CAN_MESSAGE				0x53						//!< SUBCOMMAND: Gets a CAN message
#define SUB_SYS_GET_CAN_BAUDRATE				0x54						//!< SUBCOMMAND: Set the CAN baudrate
#define SUB_SYS_GET_TEMPERATURE_SENSOR		0x55						//!< SUBCOMMAND: Gets the temperature from a temperature sensor
#define SUB_SYS_GET_SYSTEM_TIME_NS			0x56						//!< SUBCOMMAND: Gets the system clock in ns
#define SUB_SYS_GET_ADC_VAL_OFFSET			0x57						//!< SUBCOMMAND: Get the actual ADC value and its calibrated offset
#define SUB_SYS_GET_ALL_ADC_VAL_OFFSET		0x58						//!< SUBCOMMAND: Get all actual ADC values and their calibrated offset
#define SUB_SYS_GET_PUMP_PULSE				0x59						//!< SUBCOMMAND: Gets the Frequency and Pulse Duration of a Pump
#define SUB_SYS_GET_PUMP_ENABLE				0x5A						//!< SUBCOMMAND: Gets the On/Off-Status of a Pump
#define SUB_SYS_GET_ALL_ADC_AND_TEMP_F		0x5B						//!< SUBCOMMAND: Get all actual ADC values and the Temperature values as floats
#define SUB_SYS_GET_SAFETYMNGR_STATUS		0x5C						//!< SUBCOMMAND: Gets the Safety Manager Status
#define SUB_SYS_GET_LIFT_HALL_COUNT			0x5D						//!< SUBCOMMAND: Gets the Hall Sensor Pulse Count of a Lift Device
#define SUB_SYS_GET_CLEANINGMNGR_STATUS	0x5E						//!< SUBCOMMAND: Gets the Cleaning Manager Status
#define SUB_SYS_GET_LIFT_DEVICE_STATUS		0x5F						//!< SUBCOMMAND: Gets the Lift Device Status
#define SUB_SYS_GET_ENDSWITCHES				0x60						//!< SUBCOMMAND: Gets the Status of the End Switches
#define SUB_SYS_GET_VB_ENABLE_STATUS		0x61						//!< SUBCOMMAND: Gets the Status of the VB Enable Lines (VB Enable 1/2, Charge VB)
#define SUB_SYS_GET_UNBLOCK_STATUS			0x62						//!< SUBCOMMAND: Gets the Status of the Unblock Lines (Unblock-L-BR, Unblock-L-Suc)
#define SUB_SYS_GET_RELAY_STATUS				0x63						//!< SUBCOMMAND: Gets the Status of the Relays
#define SUB_SYS_GET_LIFT_HALL_CNTR			0x64						//!< SUBCOMMAND: Gets the Lift Motor Hall Counter Values
#define SUB_SYS_GET_FLOWMETER_CNTR			0x65						//!< SUBCOMMAND: Gets the Flow Meter Motor Counter Value
#define SUB_SYS_GET_HEAP_INFO					0x66						//!< SUBCOMMAND: Gets information about the System Heap and the FREERTOS Heap
#define SUB_SYS_GET_FLOWMETER_PULSES		0x67						//!< SUBCOMMAND: Get the number of FlowMeter Pulses
#define SUB_SYS_GET_CAN_STATUS				0x68						//!< SUBCOMMAND: Gets the CAN status (CANopen)
#define SUB_SYS_GET_CAN_PROVIDER_DATA		0x69						//!< SUBCOMMAND: Gets the data of a CANopen provider
#define SUB_SYS_GET_CAN_PROVIDER_INFO		0x70						//!< SUBCOMMAND: Gets information about a CANopen provider
#define SUB_SYS_GET_CANOPEN_CTRS				0x71						//!< SUBCOMMAND: Gets the CANopen TX- and RX-Counter
#define SUB_SYS_GET_ENDSWITCH_STATE			0x72						//!< SUBCOMMAND: Gets the End Switch State of the Lift Devices
#define SUB_SYS_GET_LIFT_STATE_POS			0x73						//!< SUBCOMMAND: Gets the State and Position of a Lift Devices
#define SUB_SYS_WRITE_EEPROM					0x74						//!< SUBCOMMAND: Writes a Block of Data (max 32 Bytes) to the EEPROM
#define SUB_SYS_READ_EEPROM					0x75						//!< SUBCOMMAND: Reads a Block of Data (max 32 Bytes) to the EEPROM
#define SUB_SYS_GET_CLMNGR_MAX_CUR			0x76						//!< SUBCOMMAND: Reads the Maximum Current of the Cleaning Manager Devices
#define SUB_SYS_RESET_CLMNGR_MAX_CUR		0x77						//!< SUBCOMMAND: Resets the Maximum Current of the Cleaning Manager Devices
#define SUB_SYS_SET_DRYRUN						0x78						//!< SUBCOMMAND: Sets or Resets the Dry Run Mode
#define SUB_SYS_GET_SAFETYMNGR_INT_STATUS	0x79						//!< SUBCOMMAND: Gets the Safety Manager Int Status
#define SUB_SYS_GET_PWM_STATUS				0x80						//!< SUBCOMMAND: Gets the Status of the PWM drivers

#define SUB_SYS_GET_SAFETY_ERR_CNTR      	0xA0						//!< SUBCOMMAND: Gets the Error Counters of the Safety Manager
#define SUB_SYS_CLR_SAFETY_ERR_CNTR      	0xA1						//!< SUBCOMMAND: Clears the Error Counters of the Safety Manager
#define SUB_SYS_CAN_SET_SELF_RX				0xA2						//!< SUBCOMMAND: Enables / Disables the CAN self receptions
#define SUB_SYS_INJECT_LIFT_ERROR			0xA3						//!< SUBCOMMAND: Injects a Lift Device Error

#define SUB_SYS_SET_RAMP_SLOPE				0xA4						//!< SUBCOMMAND: Sets the Ramp Slope of the Brush or Suction Device

// Info Subcommands
#define SUB_INFO_GET_SYSTEM_INFO          0x01                 //!< SUBCOMMAND: Get System Info

// Device Subcommands
#define SUB_DEVICE_HANDLE_TASK        		0x01                 //!< SUBCOMMAND: Suspends or Resumes a specific Task
#define SUB_DEVICE_HANDLE_ALL_CUC_TASKS	0x02                 //!< SUBCOMMAND: Suspends or Resumes all Tasks
#define SUB_DEVICE_GET_TASK_STATE        	0x03                 //!< SUBCOMMAND: Gets the Status of a Task
#define SUB_DEVICE_GET_TASK_NAME        	0x04                 //!< SUBCOMMAND: Gets the Name of a Task
#define SUB_DEVICE_GET_NUMBER_OF_TASKS   	0x05                 //!< SUBCOMMAND: Gets the Number of Tasks
#define SUB_DEVICE_EEPROM_INIT   			0x06                 //!< SUBCOMMAND: Initializes the EEPROM
#define SUB_DEVICE_CHECK_MAGIC_NUMBER   	0x07                 //!< SUBCOMMAND: Checks the Magic Number Field of the EEPROM
#define SUB_DEVICE_EEPROM_CLEAR_PARAMS   	0x08                 //!< SUBCOMMAND: Clear al Param Entries and resets the Param Entry Count to 0
#define SUB_DEVICE_EEPROM_WRITE_VERSION   0x09                 //!< SUBCOMMAND: Writes the Version Entry
#define SUB_DEVICE_EEPROM_READ_VERSION    0x0A                 //!< SUBCOMMAND: Reads the Version Entry
#define SUB_DEVICE_EEPROM_WRITE_PARAM   	0x0B                 //!< SUBCOMMAND: Writes a Param Entry
#define SUB_DEVICE_EEPROM_READ_PARAM    	0x0C                 //!< SUBCOMMAND: Reads a Param Entry
#define SUB_DEVICE_EEPROM_GET_PARAM_CNT   0x0D                 //!< SUBCOMMAND: Gets the number of Param Entries

// Measurement Subcommands

/*! @} */

//! <a href="CommandDescription.htm"> Detailed Command Description</a>

/**@}*/

typedef struct __attribute__((packed))
{
   int16_t        length;                    //!< Length of the packet
   uint16_t       device;                    //!< Device Number
   uint8_t        command;                   //!< Command
   uint8_t        subcommand;                //!< Subcommand
   uint8_t        acknak;                    //!< ACK or NAK
   uint8_t        flags;                     //!< Flags;
}  t_ProtocolPacketHeaderResponse;

typedef struct __attribute__((packed))
{
   int16_t        length;                    //!< Length of the packet
   uint16_t       device;                    //!< Device Number
   uint16_t       deviceclass;               //!< Device Class
   uint8_t        command;                   //!< Command
   uint8_t        subcommand;                //!< Subcommand
   uint8_t        flags;                     //!< Flags;
}  t_ProtocolPacketHeaderRequest;

#endif /* COMMANDDEFS_H_ */
