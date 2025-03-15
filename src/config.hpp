#ifndef CONFIG_HPP
#define CONFIG_HPP

// UART config
#define UART_ID uart1
#define BAUD_RATE 115200

// Commands
#define CMD_GET_VOLTAGE         0x01
#define CMD_GET_CURRENT         0x02

#define CMD_READ_SWITCH         0x03

#define CMD_SET_LED             0x04
#define CMD_SET_LEDS            0x05

#define CMD_SET_SERVO_PULSE     0x06
#define CMD_SET_SERVO_PULSES    0x07

// Control bits
#define COMMAND_START           0x55
#define COMMAND_END             0xAA

#endif  // CONFIG_HPP
