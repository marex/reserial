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

static const char *port = "/dev/ttyS0";

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

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10;

	ret = tcsetattr(fd, TCSANOW, &tty);
        if (ret) {
                printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
                return ret;
        }

        return 0;
}

static int serial_getinfo(int fd)
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
		if (i && !(i % 8))
			printf("\n");
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
	msg[5] |= sockmask & 0xf0;

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

enum opmode {
	MODE_INFO = 0,
	MODE_GET,
	MODE_SET
};

static void usage(char *name)
{
	printf("Usage: %s [ARGS]\n", name);
	printf("         -i ......... get information\n");
	printf("         -g ......... get socket status\n");
	printf("         -s <val> ... set socket status\n");
	printf("             val is hex mask to set on the switches\n");
}

int main(int argc, char *argv[])
{
	enum opmode mode;
	unsigned long val;
	int fd, ret;

	if (argc == 2 && !strcmp(argv[1], "-i"))
		mode = MODE_INFO;
	else if (argc == 2 && !strcmp(argv[1], "-g"))
		mode = MODE_GET;
	else if (argc == 3 && !strcmp(argv[1], "-s")) {
		mode = MODE_SET;
		val = strtoul(argv[2], NULL, 16) & 0x3f;
	} else {
		usage(argv[0]);
		return 0;
	}

	fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
                printf("%s[%i] ret=%i\n", __func__, __LINE__, fd);
		return 1;
	}

	ret = serial_set(fd);
        if (ret) {
                printf("%s[%i] ret=%i\n", __func__, __LINE__, ret);
                return ret;
        }

	if (mode == MODE_INFO)
		return serial_getinfo(fd);
	if (mode == MODE_GET)
		return serial_getsock(fd);
	if (mode == MODE_SET)
		return serial_setsock(fd, val);

	return 0;
}
