/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_shell.cpp
 * A shell to be used via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>

#include "mavlink_shell.h"
#include <px4_platform_common/defines.h>

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>


#ifdef __PX4_NUTTX
#include <nshlib/nshlib.h>
#endif /* __PX4_NUTTX */

#ifdef __PX4_LINUX
#include <poll.h>
#include <fcntl.h>
#include <stdlib.h>
#endif /* __PX4LINUX */

#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif

int MavlinkShell::pipe_mavlink_read[2] = {-1, -1};
int MavlinkShell::pipe_stdin_fake[2] = {-1, -1}, MavlinkShell::pipe_stdout_fake[2] = {-1, -1};
int MavlinkShell::_std_backup_fd[3] = {-1, -1, -1};

int MavlinkShell::pipe_signal_to_stop[2] = {-1, -1};

int MavlinkShell::sock = -1;

MavlinkShell::MavlinkShell()
{
	struct sockaddr_in serv_addr;
	sock = socket(AF_INET, SOCK_STREAM, 0);
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(1066);
	inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
	connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
}

MavlinkShell::~MavlinkShell()
{
	// not being called at all
	char data[128];
	sprintf(data, "~MavlinkShell()\n");
	send(sock, data, strlen(data), 0);
}

int MavlinkShell::start()
{
	PX4_INFO("Starting mavlink shell");

	int ret = 0;
	int flags = -1;

	ret = pipe(pipe_mavlink_read);

	if (ret != 0) {
		goto err;
	}

	ret = pipe(pipe_stdin_fake);

	if (ret != 0) {
		goto err;
	}

	//ret=pipe2(pipe_stdout_fake,O_NONBLOCK);
	ret = pipe(pipe_stdout_fake);
	fcntl(pipe_stdout_fake[0], F_SETFL, O_NONBLOCK);

	if (ret != 0) {
		goto err;
	}

	for (int i = 0; i < 3; ++i) {
		_std_backup_fd[i] = dup(i);

		if (_std_backup_fd[i] == -1) {
			goto err;
		}
	}

	flags = fcntl(_std_backup_fd[0], F_GETFL, 0);

	if (flags == -1) {
		goto err;
	}

	flags |= O_NONBLOCK;
	ret = fcntl(_std_backup_fd[0], F_SETFL, flags);

	if (ret == -1) {
		goto err;
	}

	ret = dup2(pipe_stdin_fake[0], 0);

	if (ret == -1) {
		goto err;
	}

	ret = dup2(pipe_stdout_fake[1], 1);

	if (ret == -1) {
		goto err;
	}

	ret = dup2(1, 2);

	if (ret == -1) {
		goto err;
	}

	ret = pipe(pipe_signal_to_stop);

	if (ret != 0) {
		goto err;
	}

	ret = atexit(&MavlinkShell::fds_cleanup);

	_task = px4_task_spawn_cmd("mavlink_shell",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT,
				   2048,
				   &MavlinkShell::shell_start_thread,
				   nullptr);

	if (_task < 0) {
		ret = -1;
	}

	return ret;

err:
	PX4_ERR("shell start failed");
	fds_cleanup();
	return ret;
}

int MavlinkShell::shell_start_thread(int argc, char *argv[])
{
#ifdef __PX4_LINUX
	char data[128];
	/*
	 * fds[0]: program writes to stdout, redirect it to mavshell and real stdout
	 * fds[1]: data comes from real stdin, redirect it to fake stdin pipe
	 * 	       data from mavshell should directly write to fake stdin pipe
	 */
	pollfd fds[3] = {};
	fds[0].fd = pipe_stdout_fake[0];
	fds[0].events = POLLIN;
	fds[1].fd = _std_backup_fd[0];
	fds[1].events = POLLIN;
	fds[2].fd = pipe_signal_to_stop[0];
	fds[2].events = POLLIN;

	while (true) {
		int event_count = poll(fds, 3, -1);
		sprintf(data, "pollin loop %d %d\n", event_count, errno);
		send(sock, data, strlen(data), 0);

		if (event_count == -1) {
			// undefined behavior
			break;
		}

		if (fds[2].revents & POLLIN) {
			sprintf(data, "stop sig\n");
			send(sock, data, strlen(data), 0);
			break;
		}

		uint8_t buffer[16];
		int ret;

		if (fds[0].revents & POLLIN) {	// new output from program, redirect it
			sprintf(data, "pollin: stdout\n");
			send(sock, data, strlen(data), 0);
			int data_size = 0;

			while (true) {
				data_size =::read(fds[0].fd, buffer, 16);

				if ((data_size == -1) && (errno == EAGAIN)) {
					break;
				}

				if (data_size == 0) {
					PX4_ERR("undefined fd behavior");
					return -1;
				}

				ret =::write(pipe_mavlink_read[1], buffer, data_size);

				if (ret == -1) {
					PX4_ERR("error");
				}

				ret =::write(_std_backup_fd[1], buffer, data_size);

				if (ret == -1) {
					//PX4_ERR("error");
				}
			}
		}

		if (fds[1].revents & POLLIN) {	// new input from real stdin, redirect it
			sprintf(data, "pollin: stdin\n");
			send(sock, data, strlen(data), 0);
			int data_size = 0;

			while (true) {
				data_size =::read(fds[1].fd, buffer, 16);

				if ((data_size == -1) && (errno == EAGAIN)) {
					break;
				}

				if (data_size == 0) {
					PX4_ERR("undefined fd behavior");
					return -1;
				}

				ret =::write(pipe_stdin_fake[1], buffer, data_size);

				if (ret == -1) {
					//PX4_ERR("error");
				}
			}
		}
	}

	sprintf(data, "poll exit\n");
	send(sock, data, strlen(data), 0);

	close(pipe_signal_to_stop[0]);
	close(pipe_signal_to_stop[1]);

#endif

	return 0;
}

void MavlinkShell::fds_cleanup()
{
	char tempbuffer = '\0';
	int ret =::write(pipe_signal_to_stop[1], &tempbuffer, 1);	// stop polling.

	if (ret) {}

	char data[128];
	sprintf(data, "cleanup fds\n");
	send(sock, data, strlen(data), 0);

	if (pipe_mavlink_read[0] >= 0) {
		close(pipe_mavlink_read[0]);
		sprintf(data, "cleanup fds 0\n");
		send(sock, data, strlen(data), 0);
	}

	if (pipe_mavlink_read[1] >= 0) {
		close(pipe_mavlink_read[1]);
		sprintf(data, "cleanup fds 1\n");
		send(sock, data, strlen(data), 0);
	}

	if (pipe_stdin_fake[0] >= 0) {
		close(pipe_stdin_fake[0]);
		sprintf(data, "cleanup fds 2\n");
		send(sock, data, strlen(data), 0);
	}

	if (pipe_stdin_fake[1] >= 0) {
		close(pipe_stdin_fake[1]);
		sprintf(data, "cleanup fds 3\n");
		send(sock, data, strlen(data), 0);
	}

	if (pipe_stdout_fake[0] >= 0) {
		close(pipe_stdout_fake[0]);
		sprintf(data, "cleanup fds 4\n");
		send(sock, data, strlen(data), 0);
	}

	if (pipe_stdout_fake[1] >= 0) {
		close(pipe_stdout_fake[1]);
		sprintf(data, "cleanup fds 5\n");
		send(sock, data, strlen(data), 0);
	}

	int flags = fcntl(_std_backup_fd[0], F_GETFL, 0);
	flags &= (~O_NONBLOCK);
	fcntl(_std_backup_fd[0], F_SETFL, flags);

	for (int i = 0; i < 3; ++i) {
		if (_std_backup_fd[i] >= 0) {
			dup2(_std_backup_fd[i], i);
			close(_std_backup_fd[i]);
			sprintf(data, "cleanup fds 6 %d\n", i);
			send(sock, data, strlen(data), 0);
		}
	}

	PX4_INFO("Cleanup fds used by MavlinkShell");
}

size_t MavlinkShell::write(uint8_t *buffer, size_t len)
{
	char data[128];
	sprintf(data, "Mavshellwrite: %d\n", len);
	send(sock, data, strlen(data), 0);
	return ::write(pipe_stdin_fake[1], buffer, len);	// write to fake stdin
}

size_t MavlinkShell::read(uint8_t *buffer, size_t len)
{
	size_t count =::read(pipe_mavlink_read[0], buffer, len);
	char data[128];
	sprintf(data, "request: %d get %d\n", len, count);
	send(sock, data, strlen(data), 0);
	return count;
}

size_t MavlinkShell::available()
{
	int ret = 0;

	if (ioctl(pipe_mavlink_read[0], FIONREAD, (unsigned long)&ret) == OK) {
		return ret;
	}

	return 0;
}
