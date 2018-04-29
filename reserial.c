/*
 * Revogi SOW323 power controller handler
 *
 * Copyright (C) 2018 Marek Vasut <marek.vasut@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Compile using:
 * /path/to/rtl819x-toolchain/toolchain/rsdk-1.3.6-4181-EB-2.6.30-0.9.30/bin/mips-linux-gcc \
 * 	-I/path/to/rtl819x-toolchain/toolchain/rsdk-1.3.6-4181-EB-2.6.30-0.9.30/include/ \
 * 	-Wall -static -o reserial reserial.c
 *
 * Run on board with:
 * $ tftp -g -r /srv/tftp/serial -l /tmp/serial 10.0.0.300
 * $ chmod a+x /tmp/serial
 * $ /tmp/serial
 */

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static const char *port = "/dev/ttyS0";
static unsigned char socket_status;

static int serial_set(int fd)
{
	struct termios tty = { 0 };
	int ret;

	ret = tcgetattr(fd, &tty);
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	cfsetospeed(&tty, B4800);
	cfsetispeed(&tty, B4800);

	tty.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
	tty.c_cflag |= CS8 | CLOCAL | CREAD;

	tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);

	tty.c_lflag = ECHOKE | ECHOCTL;

	tty.c_oflag = ONLCR;

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 10;

	ret = tcsetattr(fd, TCSANOW, &tty);
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	return 0;
}

static int serial_open(const char *port, int *rfd)
{
	int fd, ret;

	fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, fd);
		return fd;
	}

	ret = serial_set(fd);
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	*rfd = fd;
	return 0;
}

static int tcp_open(unsigned short port, int *rfd)
{
	struct sockaddr_in serveraddr = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = htonl(INADDR_ANY),
		.sin_port = htons(port),
	};
	int fd, ret, optval = 1;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, fd);
		return fd;
	}

	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR,
		   (const void *)&optval, sizeof(int));

	ret = bind(fd, (struct sockaddr *)&serveraddr, sizeof(serveraddr));
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = listen(fd, 1);	/* 1 request */
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	*rfd = fd;
	return 0;
}

static int serial_getinfo(int fd, unsigned char *ursp)
{
	/* Most likely power consumption info, format unknown */
	unsigned char msg[9] = {
		0x0f, 0x05, 0x02, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff
	};
	unsigned char rsp;
	int i, len, ret;

	printf("Get detailed socket information\n");

	ret = write(fd, msg, sizeof(msg));
	if (ret != sizeof(msg)) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	printf("rsp:\n");
	for (i = 0; i < 45; i++) {
		len = read(fd, &rsp, 1);
		if (len != 1)
			return -1;
		if (i && !(i % 6))
			printf("\n");
		if (ursp)
			*ursp++ = rsp;
		printf(" 0x%02x", rsp);
	}
	printf("\n");

	return 0;
}

static int serial_getsock(int fd)
{
	/* Socket information, byte 4 contains on/off status */
	unsigned char msg[8] = {
		0x0f, 0x04, 0x05, 0x00, 0x00, 0x00, 0xff, 0xff
	};
	unsigned char rsp;
	unsigned int csum = 0;
	int i, len, ret;

	printf("Get socket information\n");

	/* Checksum */
	for (i = 0; i < sizeof(msg); i++)
		csum += msg[i];
	msg[5] = csum & 0xf;

	ret = write(fd, msg, sizeof(msg));
	if (ret != sizeof(msg)) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	printf("rsp:\n");
	for (i = 0; i < sizeof(msg); i++) {
		len = read(fd, &rsp, 1);
		if (len != 1)
			return -1;
		printf(" 0x%02x", rsp);
	}
	printf("\n");

	return 0;
}

static int serial_setsock(int fd, unsigned char sockmask)
{
	/* Set socket on/off */
	unsigned char msg[8] = {
		0x0f, 0x04, 0x03, 0x00, sockmask & 0xf, 0x00, 0xff, 0xff
	};
	unsigned char rsp;
	unsigned int csum = 0;
	int i, len, ret;

	printf("Set detailed socket information, mask = %02x\n", sockmask);

	/* Checksum */
	for (i = 0; i < sizeof(msg); i++)
		csum += msg[i];
	msg[5] = csum & 0xf;

	/* Port 5 and 6 are special */
	msg[4] |= sockmask & 0xf0;
	msg[5] |= (sockmask + 0x4) & 0xf0;

	ret = write(fd, msg, sizeof(msg));
	if (ret != sizeof(msg)) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	for (i = 0; i < sizeof(msg); i++) {
		len = read(fd, &rsp, 1);
		if (len != 1)
			return -1;
	}

	return 0;
}

static int serial_tcphandle(int fd, unsigned char buf[16], char *rspstr)
{
	unsigned char mask = 0, val = 0, newstatus;
	unsigned char rsp[45];
	int i, ret;

	/* The packet format is o:mmmmmm:vvvvvv */
	if (buf[1] != ':')
		return -1;

	if (buf[8] != ':')
		return -2;

	if (buf[0] != 't')	/* Only supported opcode, toggle */
		return -3;

	for (i = 0; i < 6; i++) {
		if (buf[i + 2] == '1')
			mask |= 1 << i;
		else if (buf[i + 2] != '0')
			return -4;

		if (buf[i + 9] == '1')
			val |= 1 << i;
		else if (buf[i + 9] != '0')
			return -5;
	}

	printf("Set socket, mask=0x%02x val=0x%02x\n", mask, val);

	newstatus = (socket_status & ~mask) | (val & mask);
	if (socket_status != newstatus) {
		ret = serial_setsock(fd, newstatus);
		if (ret) {
			printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
			return ret;
		}
		socket_status = newstatus;
	}

	memset(rsp, 0, sizeof(rsp));
	ret = serial_getinfo(fd, rsp);
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	memset(rspstr, 0, sizeof(rspstr));
	for (i = 0; i < sizeof(rsp); i++) {
		if (i && !(i % 6))
			*rspstr++ = '\n';
		sprintf(rspstr, ":%02x", rsp[i]);
		rspstr += 3;
	}
	*rspstr++ = '\n';

	return 0;
}

static int serial_tcploop(int fd, int tcpport)
{
	struct sockaddr_in clientaddr;
	unsigned char buf[16], rspstr[45 * 3 + 8 + 1];
	int sockfd, cfd, ret, len, clientlen;

	/* Reset the sockets */
	socket_status = 0;
	ret = serial_setsock(fd, 0);
	if (ret) {
		printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = tcp_open(tcpport, &sockfd);
	if (ret < 0)
		return ret;

	while (1) {
		clientlen = sizeof(clientaddr);

		cfd = accept(sockfd, (struct sockaddr *)&clientaddr, &clientlen);
		if (cfd < 0) {
			printf("%s[%i] ret=%i\n", __func__, __LINE__, cfd);
			continue;
		}

		memset(buf, 0, sizeof(buf));

		len = read(cfd, buf, sizeof(buf));
		if (len != 15 && len != 16) {
			printf("%s[%i] ret=%i\n", __func__, __LINE__, len);
			continue;
		}

		ret = serial_tcphandle(fd, buf, rspstr);
		if (ret) {
			printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
			continue;
		}

		len = write(cfd, rspstr, sizeof(rspstr));
		if (len < 0) {
			printf("%s[%i] ret=%i\n", __func__, __LINE__, len);
			continue;
		}

		close(cfd);
	}

	close(sockfd);

	return 0;
}

enum opmode {
	MODE_INFO = 0,
	MODE_GET,
	MODE_SET,
	MODE_TEST,
	MODE_TCP,
};

static void usage(char *name)
{
	printf("Usage: %s [ARGS]\n", name);
	printf("         -i ......... get information\n");
	printf("         -g ......... get socket status\n");
	printf("         -s <val> ... set socket status\n");
	printf("             val is hex mask to set on the switches\n");
	printf("         -l <port> .. listen for commands on TCP port\n");
}

int main(int argc, char *argv[])
{
	enum opmode mode;
	unsigned long val, tcpport;
	int fd, ret;

	if (argc == 2 && !strcmp(argv[1], "-i"))
		mode = MODE_INFO;
	else if (argc == 2 && !strcmp(argv[1], "-g"))
		mode = MODE_GET;
	else if (argc == 3 && !strcmp(argv[1], "-s")) {
		mode = MODE_SET;
		val = strtoul(argv[2], NULL, 16) & 0x3f;
	} else if (argc == 2 && !strcmp(argv[1], "-t")) {
		mode = MODE_TEST;
	} else if (argc == 3 && !strcmp(argv[1], "-l")) {
		mode = MODE_TCP;
		tcpport = strtoul(argv[2], NULL, 10);
	} else {
		usage(argv[0]);
		return 0;
	}

	ret = serial_open(port, &fd);
	if (ret < 0)
		return ret;

	if (mode == MODE_INFO)
		return serial_getinfo(fd, NULL);
	if (mode == MODE_GET)
		return serial_getsock(fd);
	if (mode == MODE_SET)
		return serial_setsock(fd, val);
	if (mode == MODE_TEST) {
		for (val = 0; val <= 0x3f; val++) {
			ret = serial_setsock(fd, val);
			if (ret) {
				printf("%s[%i] ret=%i\n", __func__, __LINE__,
				       ret);
				return ret;
			}
			sleep(1);
		}
	}
	if (mode == MODE_TCP)
		return serial_tcploop(fd, tcpport);

	return 0;
}
