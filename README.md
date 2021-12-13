# 项目说明

**Linux下1602 LCD屏幕驱动**

# 参数说明

## LCD引脚定义 ##

    1 	VSS 	电源地
    2 	VDD 	电源正极
    3 	VL  	液晶显示偏压信号
    4 	RS  	数据/命令选择
    5 	RW  	读/写选择
    6 	EN  	使能
    7 	D0  	数据io口
    8 	D1  	数据io口
    9 	D2  	数据io口
    10 	D3  	数据io口
    11 	D4  	数据io口
    12 	D5  	数据io口
    13 	D6  	数据io口
    14 	D7  	数据io口
    15 	LED+ 	背光正极
    16 	LED- 	背光负极

## LCD与BeagleBone Black的连接 ##

    LCD		BBB
    4		P8-7
    5		P8-9
    6		P8-11
    7		P8-13
    8		P8-15
    9		P8-17
    10		P8-19
    11		P8-8
    12		P8-10
    13		P8-12
    14		P8-14

## 指令编码 ##
### <清屏> ###
    执行时间 	1.64ms
    指令编码
     	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0
     	0	0	0	0	0	0	0	0	0	1
     功能说明	
     	1、清除液晶屏，即将DDRAM的内容全部填入“空白”的ASCII码20H
     	2、光标归位，即将光标撤回液晶显示屏的左上方
     	3、将地址计数器(AC)的值设为0

----------


###  <光标归位> ###
    执行时间 	1.64ms
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	0	0	0	0	0	0	1	X	
    功能说明	
    	1、把光标撤回到显示器的左上方
    	2、将地址计数器(AC)的值设为0
    	3、保持DDRAM的内容不变

----------


###  <进入模式设置> ###
    执行时间 	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	0	0	0	0	0	1	N	S
    功能说明	
    设定每次写入1个字符后光标的移位方向，并且设定每次写入的1个字符是否移动
    	位名	设置
    	N	0=写入新数据后光标左移，1=写入数据后光标右移
    	S	0=写入数据后显示屏不移动，1=写入数据后整屏显示左移(N=1)或右移(N=0)

----------

###  <显示开关控制> ###
    执行时间 	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	0	0	0	0	1	D	C	B
    功能说明	
    	D=0：关显示功能，D=1：开显示功能
    	C=0：无光标，C=1：有光标
    	B=0：光标不闪烁，B=1：光标闪烁

----------

### <设定显示屏或光标移动方向>

    执行时间 	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	0	0	0	1	S/C	R/L	X	X
    功能说明
    	S/C	R/L	设置
    	0	0	光标左移1格，且AC值减1
    	0	1	光标右移1格，且AC值加1
    	1	0	显示器上字符全部左移1格，但光标不动
    	1	1	显示器上字符全部右移1格，但光标不动


​    

----------

### <功能设定> ###
    执行时间	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	0	0	1	DL	N	F	X	X
    									
    功能说明	
    	位名	设置
    	DL		0=数据总线4位，1=数据总线8位
    	N		0=显示1行，1=显示2行
    	F		0=5x7点阵/每字符，1=5x10点阵/每字符

----------


### <设定CGRAM地址> ###
    执行时间	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	0	1	   CGRAM的地址(6位)
    功能说明	
    	设定下一个要存入数据的CGRAM地址：0x40+address

----------


### <设定DDRAM地址> ###
    执行时间	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	0	1		  DDRAM的地址(7位)      
    功能说明	
    	设定下一个要存入数据的DDRAM地址：0x80+address


​    

----------

### <读取忙碌信号或AC地址> ###
    执行时间	40us
    指令编码
    	RS	R/W	DB7	DB6	DB5	DB4	DB3	DB2	DB1	DB0	
    	0	1	BF			AC内容(7位)
    功能说明	
    	BF=1表示显示器忙，BF=0表示显示器可以接收数据或指令
    	读取地址计数器（AC）的内容