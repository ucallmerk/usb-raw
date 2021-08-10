#include <stdio.h>
#include <stdlib.h>

int main (int argc, char *argv[])
{
	FILE *fd;
	char buf[100];
	int i, buf_len = 64;
	char *file = argv[1];

	fd = fopen(file, "r");
	if(!fd)
		printf("** failed to open device");

	fread(buf, 1, buf_len, fd);

	for (i=0; i<buf_len; i++)
		printf("%02hhx ", buf[i]);

	printf("\n");

	fclose(fd);
	return 0;

}

