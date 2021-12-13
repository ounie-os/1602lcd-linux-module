#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>

/**
*	指令功能	<清屏>
*	执行时间 	1.64ms
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0
*		0	0	0	0	0	0	0	0	0	1
*	功能说明	
*		1、清除液晶屏，即将DDRAM的内容全部填入“空白”的ASCII码20H
*		2、光标归位，即将光标撤回液晶显示屏的左上方
*		3、将地址计数器(AC)的值设为0
*	
*	
*	指令功能	<光标归位>
*	执行时间 	1.64ms
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	0	0	0	0	0	0	1	X	
*	功能说明	
*		1、把光标撤回到显示器的左上方
*		2、将地址计数器(AC)的值设为0
*		3、保持DDRAM的内容不变
*	
*	
*	指令功能	<进入模式设置>
*	执行时间 	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	0	0	0	0	0	1	N	S
*	功能说明	
*	设定每次写入1个字符后光标的移位方向，并且设定每次写入的一个字符是否移动
*		位名	设置
*		N	0=写入新数据后光标左移，1=写入数据后光标右移
*		S	0=写入数据后显示屏不移动，1=写入数据后整屏显示左移(N=1)或右移(N=0)
*	
*	
*	指令功能	<显示开关控制>
*	执行时间 	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	0	0	0	0	1	D	C	B
*	功能说明	
*		D=0：关显示功能，D=1：开显示功能
*		C=0：无光标，C=1：有光标
*		B=0：光标不闪烁，B=1：光标闪烁
*	
*	
*	指令功能	<设定显示屏或光标移动方向>
*	执行时间 	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	0	0	0	1	S/C	R/L	X	X
*	功能说明
*	    S/C	R/L	设置
*		0	0	光标左移1格，且AC值减1
*		0	1	光标右移1格，且AC值加1
*		1	0	显示器上字符全部左移1格，但光标不动
*		1	1	显示器上字符全部右移1格，但光标不动
*	
*	
*	指令功能	<功能设定>
*	执行时间	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	0	0	1	DL	N	F	X	X
*										
*	功能说明	
*		位名	设置
*		DL		0=数据总线4位，1=数据总线8位
*		N		0=显示1行，1=显示2行
*		F		0=5x7点阵/每字符，1=5x10点阵/每字符
*	
*	
*	指令功能	<设定CGRAM地址>
*	执行时间	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	0	1	   CGRAM的地址(6位)
*	功能说明	
*		设定下一个要存入数据的CGRAM地址：0x40+address
*	
*	
*	指令功能	<设定DDRAM地址>
*	执行时间	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	0	1	DDRAM的地址(7位)
*	功能说明	
*		设定下一个要存入数据的DDRAM地址：0x80+address
*	
*	
*	指令功能	<读取忙碌信号或AC地址>
*	执行时间	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		0	1	BF			AC内容(7位)
*	功能说明	
*		BF=1表示显示器忙，BF=0表示显示器可以接收数据或指令
*		读取地址计数器（AC）对应的内容

*	指令功能	<从CGRAM或DDRAM中读取内容>
*	执行时间	40us
*	指令编码
*		RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
*		1	1			要读出的数据
*	功能说明	
*		
*		
*/

#define LCD1602_MODULE_NAME "lcd1602"

#define D7 26			// P8_14
#define D6 44           // P8_12     
#define D5 68           // P8_10    
#define D4 67           // P8_8
#define D3 22           // P8_19
#define D2 27           // P8_17
#define D1 47           // P8_15
#define D0 23           // P8_13
#define EN 45           // P8_11     1:读取信息  	1->0(下降沿):执行指令
#define RW 69           // P8_9      0:写入LCD 		1:读取LCD
#define RS 66           // P8_7 	 0:指令 		1:数据

#define LCD1602_MODE_INIT (0x38) // 显示模式设置

/*
	设置数据指针
	80H + 地址码(0-27H, 40H-67H)
*/
#define LCD1602_SET_DATA_POINTER(addr) (0x80+addr)

#define LCD1602_CLEAR_LCD (0x01)
#define LCD1602_SHOW_CR	  (0x02)

#define LCD1602_BUSY_MASK 	(0x80)
#define LCD1602_AC_MASK 	(0x7f)

#define LCD1602_MAGIC 'L'
#define LCD_CLR _IO(LCD1602_MAGIC, 0)
#define LCD_ONE _IO(LCD1602_MAGIC, 1)
#define LCD_TWO _IO(LCD1602_MAGIC, 2)
#define LCD_SET _IO(LCD1602_MAGIC, 3)
#define LCD_COM _IO(LCD1602_MAGIC, 4)
#define LCD_AC  _IO(LCD1602_MAGIC, 5)

static struct class *lcd_class;
static struct device *lcd_drv;
static unsigned int major;
static const unsigned int port_index_data[] = {D0,D1,D2,D3,D4,D5,D6,D7};
static const unsigned int port_index[] = {D0,D1,D2,D3,D4,D5,D6,D7,EN,RW,RS};


static int read8port(void)
{
	int i;
	int value = 0;
	for (i=0;i<8;i++)
		value += (gpio_get_value(port_index_data[i]) << i);
	return value;
}

static void write8port(char value)
{
	int i, bit_value;
	for (i=0;i<8;i++)
	{
		bit_value = value & (1 << i) ? 1 : 0;
		gpio_set_value(port_index_data[i], bit_value);
	}
}

static int lcd1602_busy(void)
{
	gpio_set_value(RW,true); 	// 读取LCD
	gpio_set_value(RS,false); 	// RS=0:指令
	gpio_set_value(EN,true);  	// 拉高EN
	udelay(1); 					// 拉高EN之后需要一个端口数据建立时间
	return read8port() & LCD1602_BUSY_MASK ? 1 : 0;
}

static int lcd1602_get_ac(void)
{
	int count = 10000;
	//while (lcd1602_busy() && count--);
	//while (lcd1602_busy());
	msleep(2);
	gpio_set_value(EN,false); 	// 使EN保持一个状态
	gpio_set_value(RW,true); 	// 读取LCD
	gpio_set_value(RS,false); 	// RS=0:指令
	gpio_set_value(EN,true);  	// 拉高EN
	udelay(1); 					// 拉高EN之后需要一个端口数据建立时间
	return read8port() & LCD1602_AC_MASK;
}

static void wcom(char com)
{
	int count = 10000;
	//while (lcd1602_busy() && count--);
	//while (lcd1602_busy());
	msleep(2);
	gpio_set_value(EN,false); // 使EN保持一个状态
	gpio_set_value(RW,false); // 写入LCD
	gpio_set_value(RS,false); // RS=0:写入指令
	write8port(com);
	udelay(1);
	gpio_set_value(EN,true);  // 拉高EN
	udelay(1);
	gpio_set_value(EN,false); // 拉低EN，制造下降沿
}

static void wdat(char dat)
{
	int count = 10000;
	//while (lcd1602_busy() && count--) ;
	//while (lcd1602_busy());
	msleep(2);
	gpio_set_value(EN,false); // 使EN保持一个状态
	gpio_set_value(RW,false); // 写入LCD
	gpio_set_value(RS,true);  // RS=1:写入数据
	write8port(dat);
	udelay(1);
	gpio_set_value(EN,true);  // 拉高EN
	udelay(1);
	gpio_set_value(EN,false); // 拉低EN，制造下降沿
}

/**
	
*/
static int lcd_open(struct inode *inode, struct file *file)
{
	printk("1602lcd will open.\n");
	wcom(LCD1602_MODE_INIT); // 设置lcd的显示模式
	msleep(5);
	wcom(0x0e); // 开启显示，开启光标，光标闪烁
	msleep(5);
	wcom(LCD1602_CLEAR_LCD); // 清屏
	msleep(5);
	wcom(0x06); // 当读或写一个字符后地址指针加一，且光标加一。整屏显示不移动
	msleep(5);
	wcom(0x80); // 设置数据指针到起始位置
	msleep(5);
}

static ssize_t lcd_write (struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char val[17];
	char i = 0;
	if (count > 16)
	{
		count = 16;
	}
	copy_from_user(val, buf, count);

	while ((val[i] != '\0') && (i < count))
	{
		wdat(val[i]);
		i++;
	}
    return i;
}


static ssize_t lcd_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return 0;
}

static long lcd_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
	switch(cmd)
    {
		case LCD_CLR:
			wcom(LCD1602_CLEAR_LCD);break;
		case LCD_ONE:
			wcom(0x80);break;
		case LCD_TWO:
			wcom(0xc0);break;
		case LCD_SET:
			if (((arg >= 0) && (arg <= 0x27)) || ((arg >= 0x40) && (arg <= 0x67)))
				wcom(0x80+arg);
			break;
		case LCD_COM:
			wcom(arg);
			break;
		case LCD_AC:
			ret = lcd1602_get_ac();
			break;
		default:
			printk("unknown command.\n");
    }
        return ret;
}


static struct file_operations lcd_fops = {
        .owner = THIS_MODULE,
        .open = lcd_open,
        .write = lcd_write,       
        .read = lcd_read,
        .unlocked_ioctl = lcd_ioctl,
       
};

static int __init lcd_init(void){
	int result = 0;
	int i;
	printk(KERN_INFO "1602lcd init...\n");
	// Is the GPIO a valid GPIO number (e.g., the BBB has 4x32 but not all available)

	for (i=0; i<sizeof(port_index)/sizeof(unsigned int); i++)
	{
		if (!gpio_is_valid(port_index[i]))
		{
			printk(KERN_INFO "1602lcd: invalid gpio %d\n", port_index[i]);
			return -ENODEV;
		}
	}

	for (i=0; i<sizeof(port_index)/sizeof(unsigned int); i++)
	{
		gpio_request(port_index[i], "sysfs");
		gpio_direction_output(port_index[i], false);
	}

	major = register_chrdev(0, LCD1602_MODULE_NAME, &lcd_fops); //注册字符驱动

	lcd_class = class_create(THIS_MODULE, LCD1602_MODULE_NAME);
	lcd_drv = device_create(lcd_class, NULL, MKDEV(major, 0), NULL, LCD1602_MODULE_NAME);

	return result;
}

static void __exit lcd_exit(void){
	int i;
	for (i=0; i<sizeof(port_index)/sizeof(unsigned int); i++)
	{
		gpio_set_value(port_index[i], false);
		gpio_free(port_index[i]);
	}                      
	unregister_chrdev(major, LCD1602_MODULE_NAME);        //卸载字符驱动
	device_destroy(lcd_class, MKDEV(major, 0));  
	class_destroy(lcd_class); 
	printk(KERN_INFO "lcd1602 deinit\n");
}


module_init(lcd_init);
module_exit(lcd_exit);
MODULE_LICENSE("GPL");
