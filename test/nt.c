#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

int main (int argc, char * argv[])
{
	int fd;
	unsigned char buf[65];
	int res = 0;
	int retry = 3;


	fd = open(argv[1], O_RDWR | O_NONBLOCK);
	if(!fd) {
		printf("Failed to open device, exiting.. \n");
		return -1;
	}

	printf(" Successfully opened device for communication\n");

	usleep(100000);

	memset(buf, 0x0, sizeof(buf));
	// Send an Output report to the device. The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x80;
	buf[2] = 0x80;
	buf[3] = 0x80;
	buf[4] = 0x00;
	
/*	buf[0] = 0x0;
	buf[1] = 0x81;
	buf[2] = 0x01;
	buf[3] = 0x02;
	buf[4] = 0x00;
	*/
	printf("Writing....\n");
	res = write(fd, buf, sizeof(buf));
	if (res < 0) {
		printf("Unable to write() (2) %d \n", res);
		goto exit;
	} else	{
		printf("Buffer sent.\n");
	}
	res=0;
	usleep(100000);
	while (res <= 0 && retry--) {
		printf("Reading buffer form dev..\n");
		res = read(fd, buf, sizeof(buf));
		if(res <= 0)
			printf("Received nothing.. res = %d retrying - %d\n", res, retry);
		usleep(500000);
	}
	if(res > 0)
		for (int i = 0; i < res; i++)
			printf("%02hhx ", buf[i]);

exit:
	close(fd);

	return 0;
}
