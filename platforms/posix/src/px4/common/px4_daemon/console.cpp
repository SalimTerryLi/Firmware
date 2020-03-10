/****************************************************************************
 *
 *   Copyright (C) 2016-2018 PX4 Development Team. All rights reserved.
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
/**
 * @file console.cpp
 *
 * @author SalimTerryLi <lhf2613@gmail.com>
 */

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <px4_daemon/px4_console.h>
#include "console.h"

namespace px4_console
{
int _shell_in_fd[2] = {-1, -1};
int _shell_out_fd[2] = {-1, -1};
int _system_stdin_backup_fd = -1;

int init()
{
	int ret = 0;
	ret = pipe2(_shell_in_fd, O_NONBLOCK);

	if (ret == -1) {
		return -errno;
	}

	ret = pipe(_shell_out_fd);

	if (ret == -1) {
		close(_shell_in_fd[0]);
		close(_shell_in_fd[1]);
		return -errno;
	}

	_system_stdin_backup_fd = dup(0);

	//dup2(_shell_in_fd[0],0);

	return 0;
}

void stop()
{
	//dup2(_system_stdin_backup_fd,0);

	if (_system_stdin_backup_fd >= 0) {
		close(_system_stdin_backup_fd);
	}

	for (int i = 0; i < 2; ++i) {
		if (_shell_in_fd[i] >= 0) {
			close(_shell_in_fd[i]);
		}

		if (_shell_out_fd[i] >= 0) {
			close(_shell_out_fd[i]);
		}
	}
}

int get_get_shell_in_fd()
{
	return _shell_in_fd[0];
}

int get_get_shell_out_fd()
{
	return _shell_out_fd[1];
}
}

int get_console_input_fd(int index)
{
	return px4_console::_shell_in_fd[index];
}

int get_console_output_fd(int index)
{
	return px4_console::_shell_out_fd[index];
}