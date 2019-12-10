/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "PWMDeviceBase.h"

namespace linux_pwm_output
{

/**
 ** class NavioSysfsPWMOut
 * PWM output class for Navio Sysfs
 */
class NavioSysfsPWMOut : public PWMDeviceBase
{
public:
	NavioSysfsPWMOut() = default;
	~NavioSysfsPWMOut() override;

	int init();

	int send_output_pwm(const uint16_t *pwm, int num_outputs);

	int deviceConfigure(int argc, char **argv) override;

	int deviceInit() override;

	int deviceDeinit() override;

	int updatePWM(const uint16_t *outputs, unsigned num_outputs) override;

	int setFreq(int freq) override;

private:
	int pwm_write_sysfs(char *path, int value);

	static const int MAX_NUM_PWM = 14;
	static int FREQUENCY_PWM;

	int _pwm_fd[MAX_NUM_PWM] = {};
	int _pwm_num = MAX_NUM_PWM;

	char _device[64] = "/sys/class/pwm/pwmchip0";
};

}
