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
#define lcm_print(string, args...) printk("[LCM:AUO NT35596]"string, ##args)
#else
#define lcm_print(fmt, args...) printf(fmt, ##args)
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
#else
#ifndef BUILD_LK
#define lcm_print(fmt, args...)
#else
#define lcm_print(fmt, args...)
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
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

#define   LCM_DSI_CMD_MODE	0
static unsigned int lcm_compare_id(void);
struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table auo_init_code[] = {
{0XFF, 1, {0XEE}},
{0XFB, 1, {0X01}},
{0x18, 1, {0x40}},
{REGFLAG_DELAY, 10, {}},
{0x18, 1, {0x00}},
{REGFLAG_DELAY, 20, {}},
{0x7C, 1, {0x31}},



{0xFF,1,{0x03}},
{REGFLAG_DELAY,	1,	{}},
{0xFB,1,{0x01}},

//VIVID_C2
{0x00,1,{0x23}},
{0x01,1,{0x26}},
{0x02,1,{0x2B}},
{0x03,1,{0x2C}},
{0x04,1,{0x2D}},
{0x05,1,{0x2D}},
{0x06,1,{0x30}},
{0x07,1,{0x30}},
{0x08,1,{0x32}},
{0x09,1,{0x32}},
{0x0A,1,{0x32}},
{0x0B,1,{0x32}},
{0x0C,1,{0x37}},
{0x0D,1,{0x32}},
{0x0E,1,{0x28}},
{0x0F,1,{0x20}},
{0x10,1,{0x0A}},
{0x11,1,{0x04}},
{0x12,1,{0x03}},
{0x13,1,{0x02}},

//SKIN_VIVID_C2
{0x1B,1,{0x19}},
{0x1C,1,{0x1C}},
{0x1D,1,{0x21}},
{0x1E,1,{0x22}},
{0x1F,1,{0x23}},
{0x20,1,{0x23}},
{0x21,1,{0x26}},
{0x22,1,{0x28}},
{0x23,1,{0x28}},
{0x24,1,{0x28}},
{0x25,1,{0x28}},
{0x26,1,{0x1E}},
{0x27,1,{0x14}},
{0x28,1,{0x0A}},
{0x29,1,{0x0A}},
{0x2A,1,{0x08}},
{0x2B,1,{0x08}},
{0x2F,1,{0x08}},
{0x30,1,{0x08}},
{0x31,1,{0x08}},

//SKIN_HUE and HUE_RATIO
{0x32,1,{0x16}},
{0x33,1,{0x1A}},
{0x34,1,{0x1F}},
{0x35,1,{0x1F}},
{0x36,1,{0x16}},
{0x37,1,{0x1A}},
{0x38,1,{0x14}},
{0x39,1,{0x0F}},
{0x3A,1,{0x0C}},
{0x3B,1,{0x11}},
{0x3F,1,{0x16}},
{0x40,1,{0x18}},
{0x41,1,{0x1A}},
{0x42,1,{0x18}},
{0x43,1,{0x16}},
{0x44,1,{0x14}},
{0x45,1,{0x10}},
{0x46,1,{0x0E}},
{0x47,1,{0x0C}},
{0x48,1,{0x0A}},
{0x49,1,{0x0A}},
{0x4A,1,{0x0C}},
{0x4B,1,{0x14}},
{0x4C,1,{0x16}},
{0x4D,1,{0x04}},
{0x4E,1,{0x0A}},
{0x4F,1,{0x0F}},
{0x1A,1,{0x00}},
{0x53,1,{0x07}},
{0x54,1,{0x00}},
{0x55,1,{0x77}},

//EN_SRE
{0x5B,1,{0x70}},

//SC_Dark_Light
{0x58,1,{0xEF}},

//SC_THD
{0x5F,1,{0x24}},
//EDGE
{0x63,1,{0x00}},
{0xFF,1,{0x00}},

{REGFLAG_DELAY,	1,	{}},
{0xFB,1,{0x01}},
{0xD3,1,{0x06}},
{0xD4,1,{0x04}},
{0x36,1,{0x00}},
{0x35,1,{0x00}},
{0x55,1,{0xB0}},

	{REGFLAG_DELAY,	10,	{}}, //2 wait more than 100ms
		{0x11,	0,	{}},
	{REGFLAG_DELAY, 120,{}}, //2 wait more than 100ms
		{0x29,	0,	{}},
	{REGFLAG_DELAY, 40,	{}}, //2 wait more than 40ms
	{REGFLAG_END_OF_TABLE, 0x00, {}}		
};

static struct LCM_setting_table auo_slpin_dispoff[] = {
    // Display off sequence
    {REGFLAG_DELAY, 1, {}},
    {0x28, 0, {}},
    {REGFLAG_DELAY, 10, {}},
   // Sleep Mode On
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
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
	SET_RESET_PIN(0);
	MDELAY(4);
	if (enable) 
		{
           //reset --> L
           //lcm_reset(0);
           SET_RESET_PIN(0);
           //MDELAY(1);
           //VSP5V
           mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
           //VSN5V
           mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
           MDELAY(10);
           //reset ---> H
           //lcm_reset(1);
           SET_RESET_PIN(1);
           MDELAY(5);
           SET_RESET_PIN(0);
           MDELAY(5);
           SET_RESET_PIN(1);
           MDELAY(5);
           SET_RESET_PIN(0);
           MDELAY(5);
           SET_RESET_PIN(1);
           MDELAY(20);
		} 
	else 
		{
		SET_RESET_PIN(0);
		MDELAY(17);
		//VSN5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
		MDELAY(1);
		//VSP5V
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
#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;
		params->dsi.intermediat_buffer_num = 0;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count = FRAME_WIDTH * 3;
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 4;
		params->dsi.vertical_frontporch					= 4;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		params->dsi.horizontal_sync_active				= 2; //+ backprot = 36
		params->dsi.horizontal_backporch				= 16;
		params->dsi.horizontal_frontporch				= 16;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.PLL_CLOCK = 400;
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;
}

static void lcm_init(void)
{
	lcm_print("nt35596----auo---video-mode   lcm_init\n");
	lcm_power_switch_V2_stage(true);
	push_table(auo_init_code,
	sizeof(auo_init_code)/sizeof(struct LCM_setting_table), 1);
}
#define LCM_POWER_ENABLE
static void lcm_suspend(void)
{
	lcm_print("nt35596 lcm_suspend \n");
	push_table(auo_slpin_dispoff,
		sizeof(auo_slpin_dispoff)/sizeof(struct LCM_setting_table), 1);
	lcm_power_switch_V2_stage(false);
	//MDELAY(60);
}

static void lcm_suspend_power(void)
{
		lcm_power_switch_V2_stage(false);
}
static void lcm_resume(void)
{
	lcm_print("nt35596 lcm_resume \n");
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
//lcm_print("nt35596 lcm_esd_check %d---\n",buffer[0]);
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
	mt_set_gpio_mode(GPIO_LCD_ID_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_ID_PIN, GPIO_DIR_IN);

	ret = mt_get_gpio_in(GPIO_LCD_ID_PIN);
#ifdef BUILD_LK
	dprintf("------nt35596 lcm_compare_id ret=%d \n",ret);
#else
	printk("------nt35596 lcm_compare_id ret=%d \n",ret);
#endif
	return (ret == 0)?1:0; 
}

LCM_DRIVER nt35596_fhd_dsi_vdo_auo_lcm_drv = 
{
    .name			= "nt35596_fhd_dsi_vdo_auo",
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
