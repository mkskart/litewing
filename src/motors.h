#pragma once

#include "pid.h"

void motors_init();
void motor_write(const motor_commands_t& cmd);
