#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <string.h>
#include <platform/mt_i2c.h> 
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifndef LCM_DEBUG_LOG
#define LCM_DEBUG_LOG
#endif
#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)
#define REGFLAG_DELAY 0xffe
#define REGFLAG_END_OF_TABLE 0xfff

#ifdef LCM_DEBUG_LOG
#ifndef BUILD_LK
#define lcm_print(fmt, args...) printk(fmt, ##args)
#else
#define lcm_print(fmt, args...) printf(fmt, ##args)
#endif
#else
#ifndef BUILD_LK
#define lcm_print(fmt, args...) 
#else
#define lcm_print(fmt, args...) 
#endif
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
//extern unsigned int mz_get_hw_version(void);
 
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

//#define POWER_MODE_5
#ifndef ESD_SUPPORT
#define ESD_SUPPORT
#endif
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
//static unsigned int lcm_esd_check(void);

#define   LCM_DSI_CMD_MODE	1
static unsigned int lcm_compare_id(void);
struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table auo_init_code[] = {
	{REGFLAG_DELAY, 15,  {}}, //2 wait more than 10ms
		{0xF0,	2,	{0x5A,0x5A}},
		{0xF1,	2,	{0x5A,0x5A}},
		{0xFC,	2,	{0xA5,0xA5}},
		{0xB4,	1,	{0x41}},

		{REGFLAG_DELAY, 10,{}}, //2 wait more than 10ms
		{0xB4,	1,	{0x00}},
		{0xFE,	3,	{0x41,0x00,0x02}},
#ifdef S6D_V1
		{0xDE,	0x0E,	{0x11,0x08,0x10,0x00,0xAA,0x80,0x00,0x00,0x08,0x91,0x08,0x00,0x10,0xAA}},
#else
		{0xDE,	0x0E,	{0x11,0x08,0x10,0x00,0xFF,0x80,0x00,0x00,0x08,0x91,0x08,0x00,0x10,0xFF}},
#endif		
		{0xB0,    1,    {0x04}},  
		{0xEB,    2,    {0x5F,0x5F}}, //GVDDP & GVDDN = ¡À 3.8V
		{0xB0,    1,    {0x08}},  
		{0xDF,    2,    {0x1A,0x16}}, //LTPS timing
		{0x51,	1,	{0x7F}},
		{0x53,	1,	{0x2C}},
		{0x55,	1,	{0x00}},
		//{0x36,	1,	{0x14}},
		{0x35,	1,	{0x01}},
		{REGFLAG_DELAY, 10,{}}, //2 wait more than 10ms
		{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 120,{}}, //2 wait more than 120ms
		{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 60,	{}}, //2 wait more than 40ms
	{REGFLAG_END_OF_TABLE, 0x00, {}}		
};

static struct LCM_setting_table auo_slpout[] = {
	{0xF0,	2,	{0x5A,0x5A}},
	{0xF1,	2,	{0x5A,0x5A}},
	{0xFC,	2,	{0xA5,0xA5}},
	{0xB4,	1,	{0x41}},
	{REGFLAG_DELAY, 10,{}}, //2 wait more than 10ms
	{0xB4,	1,	{0x00}},
	{0xB0,	1,	{0x02}},
	{0xFE,	1,	{0x02}},
	{0xB3,	1,	{0x00}},
	//{0xE6,	1,	{0xAE}},//0x22
	//{0xFE,	3,	{0x41,0x00,0x02}},
	/*sleep out*/
	{0x11, 1, {0}},
	{REGFLAG_DELAY, 100, {}},
		/*display on*/
	//{0xBA,	4,	{0x10,0x00,0x00,0x08}},
	{0xBA,	4,	{0x00,0x00,0x00,0x00}},
	{0xBB,	1,	{0x0E}},
	{0x35, 1, {0x00}},
	//{0x36,	1,	{0x00}},
	{0x44, 2, {0x06,0xB8}},
	//{0x34, 0, {}},
	//{0xBB, 1, {0x0E}},
	{0x29, 1, {0}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table auo_slpin_dispoff[] = {
    // Display off sequence
    {0x28, 0, {}},
    {REGFLAG_DELAY, 30, {}},
    {0x10, 0, {}},
    //{REGFLAG_DELAY, 10, {}},
    //{0xEC, 1, {0x01}},
    {REGFLAG_DELAY, 120,  {}}, //2 wait 120 ms
    //{0xFF, 1, {0x00}},
     {REGFLAG_END_OF_TABLE, 0x00, {}}         
};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
        case REGFLAG_DELAY :
			if (table[i].count == 0)
				break;
            if(table[i].count <= 10) 
                UDELAY(table[i].count * 1000);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE :
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_power_switch_V2_stage(bool enable)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

	if (enable) 
		{
		//VSP5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
		MDELAY(1);
		//VSN5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
		MDELAY(1);
		SET_RESET_PIN(0);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(5);
		} 
	else 
		{
		SET_RESET_PIN(1);
		MDELAY(1);
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
		MDELAY(1);
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		MDELAY(1);
		}
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;
		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		params->dsi.mode   = CMD_MODE;
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;
		params->dsi.intermediat_buffer_num = 0;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count = FRAME_WIDTH * 3;
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 14;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		params->dsi.horizontal_sync_active				= 8; //+ backprot = 36
		params->dsi.horizontal_backporch				= 16;
		params->dsi.horizontal_frontporch				= 72;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.PLL_CLOCK = 500;
}

static void lcm_init(void)
{
	lcm_print("s6d1fa2--cmd mode init\n");
#if 0
	lcm_power_switch_V2_stage(true);
	push_table(auo_init_code,
		sizeof(auo_init_code)/sizeof(struct LCM_setting_table), 1);
#else
lcm_power_switch_V2_stage(true);
	push_table(auo_slpout,
		sizeof(auo_slpout)/sizeof(struct LCM_setting_table), 1);
#endif
}

#define LCM_POWER_ENABLE
static void lcm_suspend(void)
{
lcm_print("s6d1fa2 cmd lcm_suspend \n");
	push_table(auo_slpin_dispoff,
		sizeof(auo_slpin_dispoff)/sizeof(struct LCM_setting_table), 1);
#ifdef LCM_POWER_ENABLE
		lcm_power_switch_V2_stage(false);
#else
	MDELAY(60);
#endif
}

static void lcm_suspend_power(void)
{
		lcm_power_switch_V2_stage(false);
}
static void lcm_resume(void)
{
lcm_print("s6d1fa2 lcm_resume \n");
	lcm_init();
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif


#ifdef ESD_SUPPORT 
static unsigned int lcm_esd_check(void)
{
    unsigned int result = TRUE;
    unsigned int data_array[16];
    unsigned char buffer[16] = {0};

    data_array[0] = 0x00013700;
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x0A, buffer, 1);
    if (buffer[0] == 0x9C)
        result = FALSE;
//lcm_print("s6d1fa2 lcm_esd_check %d---\n",buffer[0]);
    return result;
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();

    return TRUE;
}
#endif


static unsigned int lcm_compare_id(void)
{
  	unsigned int ret = 0;
	mt_set_gpio_mode(GPIO81, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO81, GPIO_DIR_IN);

	ret = mt_get_gpio_in(GPIO81);
	lcm_print("------s6d1fa2 lcm_compare_id ret=%d \n",ret);
	return (ret == 0)?1:0; 
}

LCM_DRIVER s6d1fa2_fhd_dsi_cmd_auo_lcm_drv = 
{
    .name			= "s6d1fa2_fhd_dsi_cmd_auo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
#ifdef BUILD_LK
	.suspend_power  = lcm_suspend_power,
#endif
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif

#ifdef ESD_SUPPORT
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,
#endif  

};
