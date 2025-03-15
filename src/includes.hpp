#ifndef INCLUDES_HPP
#define INCLUDES_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "servo2040.hpp"
#include "CommandHandler.hpp"
#include "utils/analog_reader.hpp"
#include "config.hpp"

// Command headers
#include "commands/CMD_GET_Voltage.hpp"
#include "commands/CMD_GET_Current.hpp"
#include "commands/CMD_GET_Switch_state.hpp"
#include "commands/CMD_SET_Led.hpp"
#include "commands/CMD_SET_MULTIPLE_Leds.hpp"
#include "commands/CMD_SET_Pulse_To_Servo.hpp"
#include "commands/CMD_SET_Pulse_To_Multiple_Servo.hpp"

using namespace plasma;
using namespace servo;

#endif // INCLUDES_HPP
