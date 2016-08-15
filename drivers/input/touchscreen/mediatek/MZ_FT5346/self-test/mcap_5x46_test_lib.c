#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>

#include "mcap_5x46_test_lib.h"
#include "ini.h"

# define FT5X46_TEST 		1 /*1: Enable, 0: Disable*/

/*-----------------------------------------------------------
Error Code Define
-----------------------------------------------------------*/
#define		ERROR_CODE_OK						0x00
#define		ERROR_CODE_CHECKSUM_ERROR			0x01
#define		ERROR_CODE_INVALID_COMMAND			0x02
#define		ERROR_CODE_INVALID_PARAM			0x03
#define		ERROR_CODE_IIC_WRITE_ERROR			0x04
#define		ERROR_CODE_IIC_READ_ERROR			0x05
#define		ERROR_CODE_WRITE_USB_ERROR			0x06
#define		ERROR_CODE_WAIT_RESPONSE_TIMEOUT	0x07
#define		ERROR_CODE_PACKET_RE_ERROR			0x08
#define		ERROR_CODE_NO_DEVICE				0x09
#define		ERROR_CODE_WAIT_WRITE_TIMEOUT		0x0a
#define		ERROR_CODE_READ_USB_ERROR			0x0b
#define		ERROR_CODE_COMM_ERROR				0x0c
#define		ERROR_CODE_ALLOCATE_BUFFER_ERROR	0x0d
#define		ERROR_CODE_DEVICE_OPENED			0x0e
#define		ERROR_CODE_DEVICE_CLOSED			0x0f


struct Test_ConfigParam_FT5X46{
	short RawDataTest_Low_Min;
	short RawDataTest_Low_Max;
	short RawDataTest_High_Min;
	short RawDataTest_High_Max;

	boolean RawDataTest_LowFreq;
	boolean RawDataTest_HighFreq;

	short WeakShortTest_CG_Min;
	short WeakShortTest_CC_Min;

	short valid_node[TX_NUM_MAX][RX_NUM_MAX];
	short RawDataTest_Low_Min_node[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_Low_Max_node[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_High_Min_node[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_High_Max_node[TX_NUM_MAX][RX_NUM_MAX];	

};

/*test section*/
#define Section_TestItem 	"TestItem"
#define Section_BaseSet "Basic_Threshold"
#define Section_SpecialSet "SpecialSet"
#define Section_INVALID_NODE "INVALID_NODE"

#define Item_RawDataTest_Low_Min "RawDataTest_Low_Min"
#define Item_RawDataTest_Low_Max "RawDataTest_Low_Max"
#define Item_RawDataTest_High_Min "RawDataTest_High_Min"
#define Item_RawDataTest_High_Max "RawDataTest_High_Max"
#define Item_RawDataTest_LowFreq "RawDataTest_LowFreq"
#define Item_RawDataTest_HighFreq "RawDataTest_HighFreq"

#define Item_WeakShortTest_CG_Min "WeakShortTest_CG"
#define Item_WeakShortTest_CC_Min "WeakShortTest_CC"/*综合测试软件命名有误*/

#define Special_RawDataTest_Low_Min "RawData_Min_Low_Tx"
#define Special_RawDataTest_Low_Max "RawData_Max_Low_Tx"
#define Special_RawDataTest_High_Min "RawData_Min_High_Tx"
#define Special_RawDataTest_High_Max "RawData_Max_High_Tx"

#define InvalidNode "InvalidNode"

static FTS_I2c_Read_Function focal_I2C_Read;
static FTS_I2c_Write_Function focal_I2C_write;

static boolean bRawdataTest = true;

static short iTxNum = 0;
static short iRxNum = 0;

static unsigned char Reg_VolAddr = 0x05;

static struct Test_ConfigParam_FT5X46 g_TestParam_FT5X46;

static char *g_testparamstring = NULL;

static short rawdata[TX_NUM_MAX][RX_NUM_MAX];

static char sr_data[2024] = {0};
static char srtempbuf[256] = {0};
static int sr_lenth = 0;
static int sr_flag = 0;

static char error_data[2024] = {0};
static char tempbuf[256] = {0};
static int error_lenth = 0;

static void TestTp(void);
static int StartScan(void);
static int SetDriverVol(unsigned char vol);
static char GetDriverVol(void);
static void SetTxRxNum(short txnum,short rxnum);
static short GetTxNum(void);
static short GetRxNum(void);
static char GetOffsetTx(unsigned char txindex);
static char GetOffsetRx(unsigned char rxindex);
static void SetOffsetTx(unsigned char txindex,unsigned char offset);
static void SetOffsetRx(unsigned char rxindex,unsigned char offset);
static void GetRawData(short RawData[TX_NUM_MAX][RX_NUM_MAX]);
//static boolean StartTestTP(void);

static boolean FT5X46_TestItem(void);
static boolean TestItem_RawDataTest_FT5X46(void);

static void GetTestParam(void);

static void focal_msleep(int ms)
{
	msleep(ms);
}
int SetParamData(char * TestParamData)
{
	g_testparamstring = TestParamData;
	GetTestParam();
	return 0;
}
void FreeTestParamData(void)
{
	if(g_testparamstring)
		g_testparamstring = NULL;
}
/*
static short focal_abs(short value)
{
short absvalue = 0;
if(value > 0)
absvalue = value;
else
absvalue = 0 - value;

return absvalue;
}
*/
static int GetParamValue(char *section, char *ItemName, int defaultvalue) 
{
	int paramvalue = defaultvalue;
	char value[512];
	memset(value , 0x00, sizeof(value));
	if(ini_get_key(g_testparamstring, section, ItemName, value) < 0) {
		return paramvalue;
	} else {
		paramvalue = atoi(value);
	}

	return paramvalue;
}

static int GetParamString(char *section, char *ItemName, char *defaultvalue) {
	char value[512];
	int len = 0;
	memset(value , 0x00, sizeof(value));
	if(ini_get_key(g_testparamstring, section, ItemName, value) < 0) {
		return 0;
	} else {
		len = sprintf(defaultvalue, "%s", value);
	}

	return len;
}

static void GetTestParam(void)
{
	char str_tmp[128], str_node[64], str_value[512];
	int j, index, valuelen = 0, i = 0, k = 0;

	memset(str_tmp, 0, sizeof(str_tmp));
	memset(str_node, 0, sizeof(str_node));
	memset(str_value, 0, sizeof(str_value));
#if (FT5X46_TEST == 1)
	g_TestParam_FT5X46.RawDataTest_Low_Min = GetParamValue(Section_BaseSet, Item_RawDataTest_Low_Min, 3000);
	g_TestParam_FT5X46.RawDataTest_Low_Max = GetParamValue(Section_BaseSet, Item_RawDataTest_Low_Max, 15000);
	g_TestParam_FT5X46.RawDataTest_High_Min = GetParamValue(Section_BaseSet, Item_RawDataTest_High_Min, 3000);
	g_TestParam_FT5X46.RawDataTest_High_Max = GetParamValue(Section_BaseSet, Item_RawDataTest_High_Max, 15000);

	g_TestParam_FT5X46.RawDataTest_LowFreq = GetParamValue(Section_BaseSet, Item_RawDataTest_LowFreq, 0);
	g_TestParam_FT5X46.RawDataTest_HighFreq = GetParamValue(Section_BaseSet, Item_RawDataTest_HighFreq, 0);

	g_TestParam_FT5X46.WeakShortTest_CG_Min = GetParamValue(Section_BaseSet, Item_WeakShortTest_CG_Min, 2000);
	g_TestParam_FT5X46.WeakShortTest_CC_Min = GetParamValue(Section_BaseSet, Item_WeakShortTest_CC_Min, 2000);

	for(i=0; i<TX_NUM_MAX; i++) 
	{
		for(j=0; j<RX_NUM_MAX; j++) 
		{
			g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Min;
			g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Max;
			g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Min;
			g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Max;

			g_TestParam_FT5X46.valid_node[i][j] = 1;
		}
	}

	for(i=0; i<TX_NUM_MAX; i++) 
	{
		for(j=0; j<RX_NUM_MAX; j++) 
		{
			memset(str_tmp, 0x00, sizeof(str_tmp));
			sprintf(str_tmp, "%s[%d][%d]",InvalidNode,(i+1),(j+1));
			g_TestParam_FT5X46.valid_node[i][j] = GetParamValue(Section_INVALID_NODE, str_tmp, 1);
		}
	}

	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_Low_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Min;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_Low_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Max;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	


	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_High_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_High_Min_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Min;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	


	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_High_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_High_Max_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Max;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

#endif

}

int Init_I2C_Read_Func(FTS_I2c_Read_Function fpI2C_Read)
{
	focal_I2C_Read = fpI2C_Read;
	return 0;
}

int Init_I2C_Write_Func(FTS_I2c_Write_Function fpI2C_Write)
{
	focal_I2C_write = fpI2C_Write;
	return 0;
}

static int ReadReg(unsigned char RegAddr, unsigned char *RegData)
{
	return focal_I2C_Read(&RegAddr, 1, RegData, 1);
}

static int WriteReg(unsigned char RegAddr, unsigned char RegData)
{
	unsigned char cmd[2] = {0};
	cmd[0] = RegAddr;
	cmd[1] = RegData;
	return focal_I2C_write(cmd, 2);
}

static int StartScan(void)
{
	int err = 0, i = 0;
	unsigned char regvalue = 0x00;

	err = ReadReg(0x00,&regvalue);
	if (err < 0) 
	{
		FTS_DBG("Enter StartScan Err. RegValue: %d \n", regvalue);
		return err;
	}
	else 
	{
		regvalue |= 0x80;
		err = WriteReg(0x00,regvalue);
		if (err < 0) 
		{
			return err;
		}
		else 
		{
			for(i=0; i<20; i++) {
				focal_msleep(8);
				err = ReadReg(0x00,&regvalue);
				if (err < 0) 
				{
					return err;
				} 
				else 
				{
					if (0 == (regvalue >> 7)) {
						break;
					}
				}
			}
			if (i >= 20) 
			{
				return -5;
			}
		}
	}

	return 0;
}	

static int SetDriverVol(unsigned char vol)
{
	return WriteReg(Reg_VolAddr,vol);
}

static char GetDriverVol(void)
{
	char vol = 0;
	unsigned char regvalue = 0x00;

	ReadReg(Reg_VolAddr,&regvalue);
	vol = (char)regvalue;

	return vol;
}

static void SetTxRxNum(short txnum,short rxnum)
{
	iTxNum = txnum;
	iRxNum = rxnum;
}

static short GetTxNum(void)
{
	short txnum = 0;
	unsigned char regvalue = 0x00;

	if(WriteReg(0x00, 0x40) >= 0)
	{
		ReadReg(0x02,&regvalue);
		txnum = (short)regvalue;
	}
	else
	{
		return TX_NUM_MAX;
	}

	return txnum;
}

static short GetRxNum(void)
{
	short rxnum = 0;
	unsigned char regvalue = 0x00;

	if(WriteReg(0x00, 0x40) >= 0)
	{
		ReadReg(0x03,&regvalue);
		rxnum = (short)regvalue;
	}
	else
	{
		return RX_NUM_MAX;
	}

	return rxnum;
}

static char GetOffsetTx(unsigned char txindex)
{
	char txoffset = 0;
	char regvalue = 0x00;

	ReadReg((0xad + txindex),&regvalue);
	txoffset = regvalue;

	return txoffset;
}

static char GetOffsetRx(unsigned char rxindex)
{
	char rxoffset = 0;
	char regvalue = 0x00;

	ReadReg((0xd6 + rxindex),&regvalue);
	rxoffset = regvalue;

	return rxoffset;
}

static void SetOffsetTx(unsigned char txindex,unsigned char offset)
{
	WriteReg((0xad + txindex),offset);
}

static void SetOffsetRx(unsigned char rxindex,unsigned char offset)
{  
	WriteReg((0xd6 + rxindex),offset);
}

static void GetRawData(short RawData[TX_NUM_MAX][RX_NUM_MAX])
{
	//unsigned char LineNum = 0;
	unsigned char I2C_wBuffer[3];
	unsigned char rrawdata[iTxNum*iRxNum*2];
	short len = 0, i = 0;	
	short ByteNum = 0;
	int ReCode = 0;

	if(WriteReg(0x00,0x40) >= 0)
	{
		if(StartScan() >= 0)
		{	
			I2C_wBuffer[0] = 0x01;
			I2C_wBuffer[1] = 0xaa;
			focal_msleep(10);
			ReCode = focal_I2C_write(I2C_wBuffer, 2);
			I2C_wBuffer[0] = (unsigned char)(0x36);
			focal_msleep(10);
			ReCode = focal_I2C_write(I2C_wBuffer, 1);		

			ByteNum = iTxNum*iRxNum * 2;
			if (ReCode >= 0) {					
				len = ByteNum;

				memset(rrawdata, 0x00, sizeof(rrawdata));
				focal_msleep(10);
				ReCode = focal_I2C_Read(NULL, 0, rrawdata, len);	
				if (ReCode >= 0) 
				{
					for (i = 0; i < (len >> 1); i++) 
					{						
						RawData[i/iRxNum][i%iRxNum] = (short)((unsigned short)(rrawdata[i << 1]) << 8) \
							+ (unsigned short)rrawdata[(i << 1) + 1];
					}
				}
				else 
				{
					FTS_DBG("Get Rawdata failure\n");					
				}					
			}

		}
	}
	//kfree(rrawdata);        
}

boolean StartTestTP(void) 
{
	bRawdataTest = true;

	TestTp();

	return bRawdataTest;
}

static void TestTp(void) {
	int i = 0;//min = 0, max = 0;
	//unsigned char regvalue = 0x00;

	bRawdataTest = true;

	if(WriteReg(0x00, 0x40) < 0) {
		//FTS_DBG("Enter factory failure\n");
		bRawdataTest = false;
		goto Enter_WorkMode;
	}
	else
	{
		//FTS_DBG("Enter factory Successful\n");
	}
	focal_msleep(200);

	iTxNum = GetTxNum();
	focal_msleep(100);
	iRxNum = GetRxNum();

	bRawdataTest = FT5X46_TestItem();

Enter_WorkMode:	
	//the end, return work mode
	for (i = 0; i < 3; i++) {
		if (WriteReg(0x00, 0x00) >=0)
			break;
		else {
			focal_msleep(200);
		}
	}
}

static boolean FT5X46_TestItem(void)
{
	boolean bTestReuslt = true;

	bTestReuslt = bTestReuslt & TestItem_RawDataTest_FT5X46();

	return bTestReuslt;		
}

static boolean TestItem_RawDataTest_FT5X46(void)
{
	boolean bTestReuslt = true;
	boolean bUse = false;
	int i = 0, j = 0;
	int len = 0;
	int srlen = 0;
	int over = 0;
	int srover = 0;
	int err0 = 0;
	int err1 = 0;
	int err2 = 0;
	int err3 = 0;
	short min_value, max_value;

	error_lenth = 0;
	sr_lenth = 0;
	sr_flag = 0;

	if(g_TestParam_FT5X46.RawDataTest_LowFreq == 1)
	{
		bUse = true;
		sr_flag = 1;
		WriteReg(0x0a, 0x80);
		WriteReg(0xFB, 0x01);
		focal_msleep(10);

		if( iTxNum == 0 || iRxNum == 0)
		{
			bTestReuslt  = false;
			goto TEST_END;
		}

		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);

		srlen = 0;
		srover = 0;
		sr_lenth = 0;

		if(sr_data[0] != 0x00)
			sr_data[0] = 0x00;

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				srlen = sprintf(srtempbuf,"%d,",rawdata[i][j]);
				sr_lenth += srlen;
				if(sr_lenth >= 2022)
				{
					srover = 1;
				}
				if(srover == 0)
				{
					strncat(sr_data,srtempbuf,srlen);
				}

			}
			sr_lenth += 2;
			if(sr_lenth >= 2022)
			{
				srover = 1;
			}
			if(srover == 0)
			{
				strcat(sr_data,"\r\n");
			}
		}

		if(error_data[0] != 0x00)
			error_data[0] = 0x00;

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;
				min_value = g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					err0 = 1;
					len = sprintf(tempbuf,"%d,%d,%d;",i+1,j+1,rawdata[i][j]);
					error_lenth += len;
					if(error_lenth >= 2022)
					{
						over = 1;
					}
					if(over == 0)
					{
						strncat(error_data,tempbuf,len);
					}
				}
			}
		}

		if(err0 ==1)
		{
			len = sprintf(tempbuf,"Frequecy Low FIR State: ON");
			error_lenth += len;
			if(error_lenth >= 2022)
			{
				over = 1;
			}
			if(over == 0)
			{
				strncat(error_data,tempbuf,len);
			}
						
			error_lenth += 2;
			if(error_lenth >= 2022)
			{
				over = 1;
			}
			if(over == 0)
			{
				strcat(error_data,"\r\n");
			}
		}
		//FTS_DBG("=========FIR State: OFF\n");	
		WriteReg(0xFB, 0x00);	
		focal_msleep(10);

		//Read Raw Data
		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;

				min_value = g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					err1 = 1;
					len = sprintf(tempbuf,"%d,%d,%d;",i+1,j+1,rawdata[i][j]);
					error_lenth += len;
					if(error_lenth >= 2022)
					{
						over = 1;
					}
					if(over == 0)
					{
						strncat(error_data,tempbuf,len);
					}
				}
			}
		}	

	}

	if(err1 ==1)
	{
		len = sprintf(tempbuf,"Frequecy Low FIR State: OFF");
		error_lenth += len;
		if(error_lenth >= 2022)
		{
			over = 1;
		}
		if(over == 0)
		{
			strncat(error_data,tempbuf,len);
		}
					
		error_lenth += 2;
		if(error_lenth >= 2022)
		{
			over = 1;
		}
		if(over == 0)
		{
			//strncat(error_data,tempbuf,len);
			strcat(error_data,"\r\n");
		}
	}

	if(g_TestParam_FT5X46.RawDataTest_HighFreq== 1)
	{
		bUse = true;
		WriteReg(0x0a, 0x81);
		FTS_DBG("=========FIR State: ON\n");	
		focal_msleep(10);

		if( iTxNum == 0 || iRxNum == 0)
		{
			bTestReuslt  = false;
			goto TEST_END;
		}

		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);

		if(sr_flag == 0)
		{
			srlen = 0;
			srover = 0;
			sr_lenth = 0;

			if(sr_data[0] != 0x00)
				sr_data[0] = 0x00;

			if(error_data[0] != 0x00)
				error_data[0] = 0x00;

			for(i = 0;i < iTxNum;i++)
			{
				for(j = 0;j < iRxNum;j++)
				{
					//printk("%5d  ", rawdata[i][j]);
					srlen = sprintf(srtempbuf,"%d,",rawdata[i][j]);
					sr_lenth += srlen;
					if(sr_lenth >= 2022)
					{
						srover = 1;
					}
					if(srover == 0)
					{
						strncat(sr_data,srtempbuf,srlen);
					}

				}
				sr_lenth += 2;
				if(sr_lenth >= 2022)
				{
					srover = 1;
				}
				if(srover == 0)
				{
					strcat(sr_data,"\r\n");
				}
			}
		}

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;

				min_value = g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					err2 = 1;
					len = sprintf(tempbuf,"%d,%d,%d;",i+1,j+1,rawdata[i][j]);
					error_lenth += len;
					if(error_lenth >= 2022)
					{
						over = 1;
					}
					if(over == 0)
					{
						strncat(error_data,tempbuf,len);
					}
				}
			}
		}

		if(err2 ==1)
		{
			len = sprintf(tempbuf,"Frequecy High FIR State: ON");
			error_lenth += len;
			if(error_lenth >= 2022)
			{
				over = 1;
			}
			if(over == 0)
			{
				strncat(error_data,tempbuf,len);
			}
						
			error_lenth += 2;
			if(error_lenth >= 2022)
			{
				over = 1;
			}
			if(over == 0)
			{
				strcat(error_data,"\r\n");
			}
		}
	
		WriteReg(0xFB, 0x00);	
		focal_msleep(10);

		//Read Raw Data
		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;

				min_value = g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					err3 = 1;
					len = sprintf(tempbuf,"%d,%d,%d;",i+1,j+1,rawdata[i][j]);
					error_lenth += len;
					if(error_lenth >= 2022)
					{
						over = 1;
					}
					if(over == 0)
					{
						strncat(error_data,tempbuf,len);
					}
				}
			}
		}

		if(err3 ==1)
		{
			len = sprintf(tempbuf,"Frequecy High FIR State: OFF");
			error_lenth += len;
			if(error_lenth >= 2022)
			{
				over = 1;
			}
			if(over == 0)
			{
				strncat(error_data,tempbuf,len);
			}
						
			error_lenth += 2;
			if(error_lenth >= 2022)
			{
				over = 1;
			}
			if(over == 0)
			{
				//strncat(error_data,tempbuf,len);
				strcat(error_data,"\r\n");
			}
		}


	}

TEST_END:

	if(bUse)
	{
		if( bTestReuslt)
		{
			//FTS_DBG("//RawData Test is OK.\n");
		}
		else
		{
			//FTS_DBG("//RawData Test is NG.\n");
		}
	}

	return bTestReuslt;
}

int focal_save_raw_data(char * rdbuf, int rdbuflen)
{
	int len = 0;
	
	if((sr_lenth - 2) >= (rdbuflen - 1))
	{
		//len = databuflen - error_lenth - 1;
		len = rdbuflen - 2;
		strncat(rdbuf,sr_data,len);
		return (rdbuflen - 2);
	}
	else
	{
		strncat(rdbuf,sr_data,sr_lenth);
		return (sr_lenth);
		//datalen += fwritelen;
	}
}

int focal_save_error_data(char * databuf, int databuflen)
{
	int len = 0;
	
	if((error_lenth - 2) >= (databuflen - 1))
	{
		//len = databuflen - error_lenth - 1;
		len = databuflen - 2;
		strncat(databuf,error_data,len);
		return (databuflen - 2);
	}
	else
	{
		strncat(databuf,error_data,error_lenth);
		return (error_lenth);
		//datalen += fwritelen;
	}
}

