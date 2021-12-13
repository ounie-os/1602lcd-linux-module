#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>

#define LCD1602_DEVNAME "/dev/lcd1602"

#define LCD_CLR _IO('L',0)
#define LCD_ONE _IO('L',1)
#define LCD_TWO _IO('L',2)
#define LCD_SET _IO('L',3)
#define LCD_COM _IO('L',4)
#define LCD_AC  _IO('L',5)

static int fd;

void cfinish(int sig)
{
	signal(SIGINT, NULL);
	printf("\n");
	close(fd);
	exit(1);
}

int main(int argc, char **argv)
{
	char input_buf[512];
	struct sigaction sa;
	char *ins;
	fd = open(LCD1602_DEVNAME, O_RDWR);
	if (fd < 0)
	{
		printf("can't open!\n");
		exit(-1);
	}

    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = cfinish;
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

	while (1)
	{
		printf("enter command > ");
		fgets(input_buf, sizeof(input_buf) - 1, stdin);
		if (0 == strncmp(input_buf, "clr", 3))
		{
			ioctl(fd,LCD_CLR);
		}
		else if (0 == strncmp(input_buf, "set", 3))
		{
			ins = &input_buf[0];
			ins = ins + 3;
			ioctl(fd, LCD_SET, atoi(ins));
		}
		else if (0 == strncmp(input_buf, "com", 3))
		{
			ins = &input_buf[0];
			ins = ins + 3;
			ioctl(fd, LCD_COM, atoi(ins));
		}
		else if (0 == strncmp(input_buf, "getac", 5))
		{
			printf("cur ac addr = 0x%x\n", ioctl(fd, LCD_AC));
		}
		else
		{
			write(fd,input_buf,strlen(input_buf)-1);
		}
	}

	close(fd);
	return 0;
}