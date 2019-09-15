#include "cmdq_platform.h"
#include "cmdq_core.h"
#include "cmdq_reg.h"

#include <linux/vmalloc.h>
#include <mach/mt_clkmgr.h>
#include <linux/seq_file.h>

#include <../../smi/smi_debug.h>

#define MMSYS_CONFIG_BASE  cmdq_dev_get_module_base_VA_MMSYS_CONFIG()

typedef struct RegDef {
	int offset;
	const char *name;
} RegDef;

const bool cmdq_core_support_sync_non_suspendable(void)
{
	return true;
}

const bool cmdq_core_support_wait_and_receive_event_in_same_tick(void)
{
	return true;
}

const uint32_t cmdq_core_get_subsys_LSB_in_argA(void)
{
	return 16;
}

int32_t cmdq_subsys_from_phys_addr(uint32_t physAddr)
{
#define DISP_PWM_BASE 0x1100E000

	const int32_t msb = (physAddr & 0x0FFFF0000) >> 16;

#undef DECLARE_CMDQ_SUBSYS
#define DECLARE_CMDQ_SUBSYS(addr, id, grp, base) case addr: return id;
	switch (msb) {
#include "cmdq_subsys.h"
	}

	/* Extra handle for HW registers which not in GCE subsys table, should check and add by case */
	if (DISP_PWM_BASE == (physAddr & 0xFFFFF000)){
		CMDQ_MSG("Special handle subsys (PWM), physAddr:0x%08x\n", physAddr);
		return CMDQ_SPECIAL_SUBSYS_ADDR;
	}
	CMDQ_ERR("unrecognized subsys, msb=0x%04x, physAddr:0x%08x\n", msb, physAddr);
	return -1;
#undef DECLARE_CMDQ_SUBSYS
}

void cmdq_core_fix_command_desc_scenario_for_user_space_request(cmdqCommandStruct *pCommand)
{
	if ((CMDQ_SCENARIO_USER_DISP_COLOR == pCommand->scenario) || (CMDQ_SCENARIO_USER_MDP == pCommand->scenario)) {
		CMDQ_VERBOSE("user space request, scenario:%d\n", pCommand->scenario);
	} else {
		CMDQ_VERBOSE("[WARNING]fix user space request to CMDQ_SCENARIO_USER_SPACE\n");
		pCommand->scenario = CMDQ_SCENARIO_USER_SPACE;
	}
}

bool cmdq_core_is_request_from_user_space(const CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_USER_DISP_COLOR:
	case CMDQ_SCENARIO_USER_MDP:
	case CMDQ_SCENARIO_USER_SPACE: /* phased out */
		return true;
	default:
		return false;
	}
	return false;
}

bool cmdq_core_is_disp_scenario(const CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_MEMOUT:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_RDMA0_COLOR0_DISP:
	case CMDQ_SCENARIO_RDMA1_DISP:
	case CMDQ_SCENARIO_TRIGGER_LOOP:
	case CMDQ_SCENARIO_DISP_ESD_CHECK:
	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
	/* color path */
	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
	/* secure path */
	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
		return true;
	default:
		return false;
	}
	/* freely dispatch */
	return false;
}

bool cmdq_core_should_enable_prefetch(CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:	/* HACK: force debug into 0/1 thread */
		/* any path that connects to Primary DISP HW */
		/* should enable prefetch. */
		/* MEMOUT scenarios does not. */
		/* Also, since thread 0/1 shares one prefetch buffer, */
		/* we allow only PRIMARY path to use prefetch. */
		return true;

	default:
		return false;
	}

	return false;
}

bool cmdq_core_should_profile(CMDQ_SCENARIO_ENUM scenario)
{
#ifdef CMDQ_GPR_SUPPORT
	switch (scenario) {
	default:
		return false;
	}
	return false;
#else
	/* note command profile method depends on GPR */
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return false;
#endif
}

const bool cmdq_core_is_a_secure_thread(const int32_t thread)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	if ((CMDQ_MIN_SECURE_THREAD_ID <= thread) &&
		(CMDQ_MIN_SECURE_THREAD_ID + CMDQ_MAX_SECURE_THREAD_COUNT > thread)){
		return true;
	}
#endif

	return false;
}

const bool cmdq_core_is_valid_notify_thread_for_secure_path(const int32_t thread)
{
#if defined(CMDQ_SECURE_PATH_SUPPORT) && !defined(CMDQ_SECURE_PATH_NORMAL_IRQ)
	return (15 == thread) ? (true) : (false);
#else
	return false;
#endif
}

int cmdq_core_get_thread_index_from_scenario_and_secure_data(CMDQ_SCENARIO_ENUM scenario, const bool secure)
{
#if defined(CMDQ_SECURE_PATH_SUPPORT) && !defined(CMDQ_SECURE_PATH_NORMAL_IRQ)
	if (!secure && CMDQ_SCENARIO_SECURE_NOTIFY_LOOP == scenario) {
		return 15;
	}
#endif

	if (!secure) {
		return cmdq_core_disp_thread_index_from_scenario(scenario);
	}

	/* dispatch secure thread according to scenario */
	switch (scenario) {
	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		 /* CMDQ_MIN_SECURE_THREAD_ID */
		return 12;
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_MHL_DISP:
		/* because mirror mode and sub disp never use at the same time in secure path, */
		/* dispatch to same HW thread */
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
	case CMDQ_SCENARIO_DISP_COLOR:
		return 13;
	case CMDQ_SCENARIO_USER_MDP:
	case CMDQ_SCENARIO_USER_SPACE:
	case CMDQ_SCENARIO_DEBUG:
		/* because there is one input engine for MDP, reserve one secure thread is enough */
		return 14;
	default:
		CMDQ_ERR("no dedicated secure thread for senario:%d\n", scenario);
		return CMDQ_INVALID_THREAD;
	}
}

int cmdq_core_disp_thread_index_from_scenario(CMDQ_SCENARIO_ENUM scenario)
{
	if (cmdq_core_should_enable_prefetch(scenario)) {
		return 0;
	}

	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:	/* HACK: force debug into 0/1 thread */
		/* primary config: thread 0 */
		return 0;

	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_SUB_MEMOUT:
		/* when HW thread 0 enables pre-fetch, any thread 1 operation will let HW thread 0's behavior abnormally */
		/* forbid thread 1 */
		return 5;

	case CMDQ_SCENARIO_DISP_ESD_CHECK:
		return 2;

	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
		return 3;

	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
		return 4;
	default:
		/* freely dispatch */
		return CMDQ_INVALID_THREAD;
	}
	/* freely dispatch */
	return CMDQ_INVALID_THREAD;
}

CMDQ_HW_THREAD_PRIORITY_ENUM cmdq_core_priority_from_scenario(CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_PRIMARY_DISP:
	case CMDQ_SCENARIO_PRIMARY_ALL:
	case CMDQ_SCENARIO_SUB_DISP:
	case CMDQ_SCENARIO_SUB_ALL:
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_RDMA0_DISP:
	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
	case CMDQ_SCENARIO_SUB_MEMOUT:
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
	/* color path */
	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
	/* secure path **/
	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
		/* currently, a prefetch thread is always in high priority. */
		return CMDQ_THR_PRIO_DISPLAY_CONFIG;

		/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		return CMDQ_THR_PRIO_DISPLAY_CONFIG;

	case CMDQ_SCENARIO_DISP_ESD_CHECK:
	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
		return CMDQ_THR_PRIO_DISPLAY_ESD;

	default:
		/* other cases need exta logic, see below. */
		break;
	}

	if (cmdq_platform_is_loop_scenario(scenario, true)) {
		return CMDQ_THR_PRIO_DISPLAY_TRIGGER;
	} else {
		return CMDQ_THR_PRIO_NORMAL;
	}
}

bool cmdq_platform_force_loop_irq_from_scenario(CMDQ_SCENARIO_ENUM scenario)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	if (CMDQ_SCENARIO_SECURE_NOTIFY_LOOP == scenario) {
		/* For secure notify loop, we need IRQ to update secure task */
		return true;
	}
#endif
	return false;
}

bool cmdq_platform_is_loop_scenario(CMDQ_SCENARIO_ENUM scenario, bool displayOnly)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	if (!displayOnly && CMDQ_SCENARIO_SECURE_NOTIFY_LOOP == scenario) {
		return true;
	}
#endif
	if (CMDQ_SCENARIO_TRIGGER_LOOP == scenario) {
		return true;
	}
	return false;
}

void cmdq_core_get_reg_id_from_hwflag(uint64_t hwflag, CMDQ_DATA_REGISTER_ENUM *valueRegId,
				      CMDQ_DATA_REGISTER_ENUM *destRegId,
				      CMDQ_EVENT_ENUM *regAccessToken)
{
	*regAccessToken = CMDQ_SYNC_TOKEN_INVALID;

	if (hwflag & (1LL << CMDQ_ENG_JPEG_ENC)) {
		*valueRegId = CMDQ_DATA_REG_JPEG;
		*destRegId = CMDQ_DATA_REG_JPEG_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_0;
	} else if (hwflag & (1LL << CMDQ_ENG_MDP_TDSHP0)) {
		*valueRegId = CMDQ_DATA_REG_2D_SHARPNESS_0;
		*destRegId = CMDQ_DATA_REG_2D_SHARPNESS_0_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_1;
	} else if (hwflag & (1LL << CMDQ_ENG_DISP_COLOR0)) {
		*valueRegId = CMDQ_DATA_REG_PQ_COLOR;
		*destRegId = CMDQ_DATA_REG_PQ_COLOR_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_3;
	} else {
		/* assume others are debug cases */
		*valueRegId = CMDQ_DATA_REG_DEBUG;
		*destRegId = CMDQ_DATA_REG_DEBUG_DST;
		*regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_4;
	}

	return;
}

const char *cmdq_core_module_from_event_id(CMDQ_EVENT_ENUM event, uint32_t instA, uint32_t instB)
{
	const char *module = "CMDQ";

	switch (event) {
	case CMDQ_EVENT_DISP_RDMA0_SOF:
	case CMDQ_EVENT_DISP_RDMA1_SOF:
	case CMDQ_EVENT_DISP_RDMA0_EOF:
	case CMDQ_EVENT_DISP_RDMA1_EOF:
	case CMDQ_EVENT_DISP_RDMA0_UNDERRUN:
	case CMDQ_EVENT_DISP_RDMA1_UNDERRUN:
		module = "DISP_RDMA";
		break;

	case CMDQ_EVENT_DISP_WDMA0_SOF:
	case CMDQ_EVENT_DISP_WDMA0_EOF:
		module = "DISP_WDMA";
		break;

	case CMDQ_EVENT_DISP_OVL0_SOF:
	case CMDQ_EVENT_DISP_OVL0_EOF:
		module = "DISP_OVL";
		break;

	case CMDQ_EVENT_DSI_TE:
	case CMDQ_EVENT_DISP_COLOR_SOF...CMDQ_EVENT_DISP_PWM0_SOF:
	case CMDQ_EVENT_DISP_COLOR_EOF...CMDQ_EVENT_DISP_DPI0_EOF:
	case CMDQ_EVENT_MUTEX0_STREAM_EOF...CMDQ_EVENT_MUTEX4_STREAM_EOF:
	case CMDQ_SYNC_TOKEN_CONFIG_DIRTY:
	case CMDQ_SYNC_TOKEN_STREAM_EOF:
		module = "DISP";
		break;

	case CMDQ_EVENT_MDP_RDMA0_SOF...CMDQ_EVENT_MDP_RSZ1_SOF:
	case CMDQ_EVENT_MDP_TDSHP_SOF:
	case CMDQ_EVENT_MDP_WDMA_SOF...CMDQ_EVENT_MDP_WROT_SOF:
	case CMDQ_EVENT_MDP_RDMA0_EOF...CMDQ_EVENT_MDP_WROT_READ_EOF:
	case CMDQ_EVENT_MUTEX5_STREAM_EOF...CMDQ_EVENT_MUTEX9_STREAM_EOF:
		module = "MDP";
		break;

	case CMDQ_EVENT_ISP_PASS2_2_EOF:
	case CMDQ_EVENT_ISP_PASS2_1_EOF:
	case CMDQ_EVENT_ISP_PASS2_0_EOF:
	case CMDQ_EVENT_ISP_PASS1_1_EOF:
	case CMDQ_EVENT_ISP_PASS1_0_EOF:
	case CMDQ_EVENT_ISP_CAMSV_2_PASS1_DONE:
	case CMDQ_EVENT_ISP_CAMSV_1_PASS1_DONE:
	case CMDQ_EVENT_ISP_SENINF_CAM1_2_3_FULL:
	case CMDQ_EVENT_ISP_SENINF_CAM0_FULL:
		module = "ISP";
		break;

	case CMDQ_EVENT_JPEG_ENC_EOF:
	case CMDQ_EVENT_JPEG_DEC_EOF:
		module = "JPGE";
		break;

	case CMDQ_EVENT_VENC_EOF:
	case CMDQ_EVENT_VENC_MB_DONE:
	case CMDQ_EVENT_VENC_128BYTE_CNT_DONE:
		module = "VENC";
		break;

	default:
		module = "CMDQ";
		break;
	}

	return module;
}

const char *cmdq_core_parse_module_from_reg_addr(uint32_t reg_addr)
{
	const uint32_t addr_base_and_page = (reg_addr & 0xFFFFF000);
	const uint32_t addr_base_shifted = (reg_addr & 0xFFFF0000) >> 16;
	const char *module = "CMDQ";

	/* for well-known base, we check them with 12-bit mask */
	/* defined in mt_reg_base.h */
	/* TODO: comfirm with SS if IO_VIRT_TO_PHYS workable when enable device tree? */
#define DECLARE_REG_RANGE(base, name) case base: return #name;
	switch (addr_base_and_page) {
		DECLARE_REG_RANGE(0x14001000, MDP);	/* MDP_RDMA */
		DECLARE_REG_RANGE(0x14002000, MDP);	/* MDP_RSZ0 */
		DECLARE_REG_RANGE(0x14003000, MDP);	/* MDP_RSZ1 */
		DECLARE_REG_RANGE(0x14004000, MDP);	/* MDP_WDMA */
		DECLARE_REG_RANGE(0x14005000, MDP);	/* MDP_WROT */
		DECLARE_REG_RANGE(0x14006000, MDP);	/* MDP_TDSHP */

		DECLARE_REG_RANGE(0x1400C000, COLOR);	/* DISP_COLOR */
		DECLARE_REG_RANGE(0x1400D000, CCORR);	/* DISP_CCORR */
		DECLARE_REG_RANGE(0x14007000, OVL0);	/* DISP_OVL0 */
		DECLARE_REG_RANGE(0x14008000, OVL1);	/* DISP_OVL1 */
		DECLARE_REG_RANGE(0x1400E000, AAL);	/* DISP_AAL */
		DECLARE_REG_RANGE(0x1400F000, AAL);	/* DISP_GAMMA */
		DECLARE_REG_RANGE(0x17002FFF, VENC);	/* VENC */
		DECLARE_REG_RANGE(0x17003FFF, JPGENC);	/* JPGENC */
		DECLARE_REG_RANGE(0x17004FFF, JPGDEC);	/* JPGDEC */
	}
#undef DECLARE_REG_RANGE

	/* for other register address we rely on GCE subsys to group them with */
	/* 16-bit mask. */
#undef DECLARE_CMDQ_SUBSYS
#define DECLARE_CMDQ_SUBSYS(msb, id, grp, base) case msb: return #grp;
	switch (addr_base_shifted) {
#include "cmdq_subsys.h"
	}
#undef DECLARE_CMDQ_SUBSYS
	return module;
}

const int32_t cmdq_core_can_module_entry_suspend(EngineStruct *engineList)
{
	int32_t status = 0;
	int i;
	CMDQ_ENG_ENUM e = 0;

	CMDQ_ENG_ENUM mdpEngines[] = {
		CMDQ_ENG_ISP_IMGI,
		CMDQ_ENG_MDP_RDMA0,
		CMDQ_ENG_MDP_RSZ0,
		CMDQ_ENG_MDP_RSZ1,
		CMDQ_ENG_MDP_TDSHP0,
		CMDQ_ENG_MDP_WROT0,
		CMDQ_ENG_MDP_WDMA
	};

	for (i = 0; i < (sizeof(mdpEngines) / sizeof(CMDQ_ENG_ENUM)); ++i) {
		e = mdpEngines[i];
		if (0 != engineList[e].userCount) {
			CMDQ_ERR("suspend but engine %d has userCount %d, owner=%d\n",
				 e, engineList[e].userCount, engineList[e].currOwner);
			status = -EBUSY;
		}
	}

	return status;
}

ssize_t cmdq_core_print_status_clock(char *buf)
{
	int32_t length = 0;
	char *pBuffer = buf;

#ifdef CMDQ_PWR_AWARE
	/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
	pBuffer += sprintf(pBuffer, "MT_CG_INFRA_GCE: %d\n", clock_is_on(MT_CG_INFRA_GCE));
#endif
	length = pBuffer - buf;
	return length;
}

void cmdq_core_print_status_seq_clock(struct seq_file *m)
{
#ifdef CMDQ_PWR_AWARE
	/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
	seq_printf(m, "MT_CG_INFRA_GCE: %d\n", clock_is_on(MT_CG_INFRA_GCE));
#endif
}

void cmdq_core_enable_common_clock_locked_impl(bool enable)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		CMDQ_VERBOSE("[CLOCK] Enable SMI & LARB0 Clock\n");
		enable_clock(MT_CG_DISP0_SMI_COMMON, "CMDQ_MDP");
		enable_clock(MT_CG_DISP0_SMI_LARB0, "CMDQ_MDP");

		#if 0
		/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
		CMDQ_LOG("[CLOCK] enable MT_CG_DISP0_MUTEX_32K\n");
		enable_clock(MT_CG_DISP0_MUTEX_32K, "CMDQ_MDP");
		#endif
	} else {
		CMDQ_VERBOSE("[CLOCK] Disable SMI & LARB0 Clock\n");
		/* disable, reverse the sequence */
		disable_clock(MT_CG_DISP0_SMI_LARB0, "CMDQ_MDP");
		disable_clock(MT_CG_DISP0_SMI_COMMON, "CMDQ_MDP");

		#if 0
		/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
		CMDQ_LOG("[CLOCK] disable MT_CG_DISP0_MUTEX_32K\n");
		disable_clock(MT_CG_DISP0_MUTEX_32K, "CMDQ_MDP");
		#endif
	}
#endif /* CMDQ_PWR_AWARE */
}

void cmdq_core_enable_gce_clock_locked_impl(bool enable)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		CMDQ_VERBOSE("[CLOCK] Enable CMDQ(GCE) Clock\n");
		cmdq_core_enable_cmdq_clock_locked_impl(enable, CMDQ_DRIVER_DEVICE_NAME);
	} else {
		CMDQ_VERBOSE("[CLOCK] Disable CMDQ(GCE) Clock\n");
		cmdq_core_enable_cmdq_clock_locked_impl(enable, CMDQ_DRIVER_DEVICE_NAME);
	}
#endif /* CMDQ_PWR_AWARE */
}

void cmdq_core_enable_cmdq_clock_locked_impl(bool enable, char *deviceName)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		enable_clock(MT_CG_INFRA_GCE, deviceName);

	} else {
		disable_clock(MT_CG_INFRA_GCE, deviceName);
	}
#endif /* CMDQ_PWR_AWARE */
}

const char* cmdq_core_parse_error_module_by_hwflag_impl(struct TaskStruct *pTask)
{
	const char *module = NULL;
	const uint32_t ISP_ONLY[2] = {
		((1LL << CMDQ_ENG_ISP_IMGI) | (1LL << CMDQ_ENG_ISP_IMG2O)),
		((1LL << CMDQ_ENG_ISP_IMGI) | (1LL << CMDQ_ENG_ISP_IMG2O) | (1LL << CMDQ_ENG_ISP_IMGO))};

	/* common part for both normal and secure path */
	/* for JPEG scenario, use HW flag is sufficient */
	if (pTask->engineFlag & (1LL << CMDQ_ENG_JPEG_ENC)) {
		module = "JPGENC";
	} else if (pTask->engineFlag & (1LL << CMDQ_ENG_JPEG_DEC)) {
		module = "JPGDEC";
	} else if ((ISP_ONLY[0] == pTask->engineFlag) || (ISP_ONLY[1] == pTask->engineFlag)) {
		module = "ISP_ONLY";
	} else if (cmdq_core_is_disp_scenario(pTask->scenario)) {
		module = "DISP";
	}

	/* for secure path, use HW flag is sufficient */
	do {
		if (NULL != module) {
			break;
		}

		if (false == pTask->secData.isSecure) {
			/* normal path, need parse current running instruciton for more detail */
			break;
		}

		else if (CMDQ_ENG_MDP_GROUP_FLAG(pTask->engineFlag)) {
			module = "MDP";
			break;
		}

		module = "CMDQ";
	} while(0);

	/* other case, we need to analysis instruction for more detail */
	return module;
}

void cmdq_core_dump_mmsys_config(void)
{
	int i = 0;
	uint32_t value = 0;

	const static struct RegDef configRegisters[] = {
		{0x01c, "ISP_MOUT_EN"},
		{0x020, "MDP_RDMA_MOUT_EN"},
		{0x024, "MDP_PRZ0_MOUT_EN"},
		{0x028, "MDP_PRZ1_MOUT_EN"},
		{0x02C, "MDP_TDSHP_MOUT_EN"},
		{0x030, "DISP_OVL0_MOUT_EN"},
		{0x034, "DISP_OVL1_MOUT_EN"},
		{0x038, "DISP_DITHER_MOUT_EN"},
		{0x03C, "DISP_UFOE_MOUT_EN"},
		/* {0x040, "MMSYS_MOUT_RST"}, */
		{0x044, "MDP_PRZ0_SEL_IN"},
		{0x048, "MDP_PRZ1_SEL_IN"},
		{0x04C, "MDP_TDSHP_SEL_IN"},
		{0x050, "MDP_WDMA_SEL_IN"},
		{0x054, "MDP_WROT_SEL_IN"},
		{0x058, "DISP_COLOR_SEL_IN"},
		{0x05C, "DISP_WDMA_SEL_IN"},
		{0x060, "DISP_UFOE_SEL_IN"},
		{0x064, "DSI0_SEL_IN"},
		{0x068, "DPI0_SEL_IN"},
		{0x06C, "DISP_RDMA0_SOUT_SEL_IN"},
		{0x070, "DISP_RDMA1_SOUT_SEL_IN"},
		{0x0F0, "MMSYS_MISC"},
		/* ACK and REQ related */
		{0x8a0, "DISP_DL_VALID_0"},
		{0x8a4, "DISP_DL_VALID_1"},
		{0x8a8, "DISP_DL_READY_0"},
		{0x8ac, "DISP_DL_READY_1"},
		{0x8b0, "MDP_DL_VALID_0"},
		{0x8b4, "MDP_DL_READY_0"}
	};

	for (i = 0; i < sizeof(configRegisters) / sizeof(configRegisters[0]); ++i) {
		value = CMDQ_REG_GET16(MMSYS_CONFIG_BASE + configRegisters[i].offset);
		CMDQ_ERR("%s: 0x%08x\n", configRegisters[i].name, value);
	}

	return;
}

void cmdq_core_dump_clock_gating(void)
{
	uint32_t value[3] = { 0 };

	value[0] = CMDQ_REG_GET32(MMSYS_CONFIG_BASE + 0x100);
	value[1] = CMDQ_REG_GET32(MMSYS_CONFIG_BASE + 0x110);
	/* value[2] = CMDQ_REG_GET32(MMSYS_CONFIG_BASE + 0x890); */
	CMDQ_ERR("MMSYS_CG_CON0(deprecated): 0x%08x, MMSYS_CG_CON1: 0x%08x\n", value[0], value[1]);
	/* CMDQ_ERR("MMSYS_DUMMY_REG: 0x%08x\n", value[2]); */
#ifndef CONFIG_MTK_FPGA
	CMDQ_ERR("ISPSys clock state %d\n", subsys_is_on(SYS_ISP));
	CMDQ_ERR("DisSys clock state %d\n", subsys_is_on(SYS_DIS));
	CMDQ_ERR("VDESys clock state %d\n", subsys_is_on(SYS_VDE));
#endif
}

int cmdq_core_dump_smi(const int showSmiDump)
{
	int isSMIHang = 0;

#ifndef CONFIG_MTK_FPGA
	/* isSMIHang = smi_debug_bus_hanging_detect(
						SMI_DBG_DISPSYS | SMI_DBG_VDEC | SMI_DBG_IMGSYS | SMI_DBG_VENC | SMI_DBG_MJC,
						showSmiDump); */
	/* isSMIHang = smi_debug_bus_hanging_detect_ext(
						SMI_DBG_DISPSYS | SMI_DBG_VDEC | SMI_DBG_IMGSYS | SMI_DBG_VENC | SMI_DBG_MJC,
						showSmiDump, 1); */
	CMDQ_ERR("SMI Hang? = %d\n", isSMIHang);
#endif

	return isSMIHang;
}

void cmdq_core_dump_secure_metadata(cmdqSecDataStruct *pSecData)
{
	uint32_t i = 0;
	cmdqSecAddrMetadataStruct *pAddr = NULL;

	if (NULL == pSecData) {
		return;
	}

	pAddr = (cmdqSecAddrMetadataStruct *)(CMDQ_U32_PTR(pSecData->addrMetadatas));

	CMDQ_LOG("========= pSecData: %p dump =========\n", pSecData);
	CMDQ_LOG("count:%d(%d), enginesNeedDAPC:0x%llx, enginesPortSecurity:0x%llx\n",
		pSecData->addrMetadataCount, pSecData->addrMetadataMaxCount,
		pSecData->enginesNeedDAPC, pSecData->enginesNeedPortSecurity);

	if (NULL == pAddr) {
		return;
	}

	for (i = 0; i < pSecData->addrMetadataCount; i++) {
		CMDQ_LOG("idx:%d, type:%d, baseHandle:%x, offset:%d, size:%d, port:%d\n",
			i, pAddr[i].type, pAddr[i].baseHandle, pAddr[i].offset, pAddr[i].size, pAddr[i].port);
	}
}

uint64_t cmdq_rec_flag_from_scenario(CMDQ_SCENARIO_ENUM scn)
{
	uint64_t flag = 0;

	switch (scn) {
	case CMDQ_SCENARIO_JPEG_DEC:
		flag = (1LL << CMDQ_ENG_JPEG_DEC);
		break;
	case CMDQ_SCENARIO_PRIMARY_DISP:
		flag = (1LL << CMDQ_ENG_DISP_OVL0) |
		    (1LL << CMDQ_ENG_DISP_COLOR0) |
		    (1LL << CMDQ_ENG_DISP_AAL) |
		    (1LL << CMDQ_ENG_DISP_GAMMA) |
		    (1LL << CMDQ_ENG_DISP_RDMA0) |
		    (1LL << CMDQ_ENG_DISP_UFOE) | (1LL << CMDQ_ENG_DISP_DSI0_CMD);
		break;
	case CMDQ_SCENARIO_PRIMARY_MEMOUT:
		flag = 0LL;
		break;
	case CMDQ_SCENARIO_PRIMARY_ALL:
		flag = ((1LL << CMDQ_ENG_DISP_OVL0) |
			(1LL << CMDQ_ENG_DISP_WDMA0) |
			(1LL << CMDQ_ENG_DISP_COLOR0) |
			(1LL << CMDQ_ENG_DISP_AAL) |
			(1LL << CMDQ_ENG_DISP_GAMMA) |
			(1LL << CMDQ_ENG_DISP_RDMA0) |
			(1LL << CMDQ_ENG_DISP_UFOE) | (1LL << CMDQ_ENG_DISP_DSI0_CMD));
		break;
	case CMDQ_SCENARIO_SUB_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) |
			(1LL << CMDQ_ENG_DISP_RDMA1) | (1LL << CMDQ_ENG_DISP_DPI));
		break;
	case CMDQ_SCENARIO_SUB_MEMOUT:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) | (1LL << CMDQ_ENG_DISP_WDMA1));
		break;
	case CMDQ_SCENARIO_SUB_ALL:
		flag = ((1LL << CMDQ_ENG_DISP_OVL1) |
			(1LL << CMDQ_ENG_DISP_WDMA1) |
			(1LL << CMDQ_ENG_DISP_RDMA1) | (1LL << CMDQ_ENG_DISP_DPI));
		break;
	case CMDQ_SCENARIO_RDMA0_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_RDMA0) | (1LL << CMDQ_ENG_DISP_DSI0_CMD));
		break;
	case CMDQ_SCENARIO_RDMA0_COLOR0_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_RDMA0) |
			(1LL << CMDQ_ENG_DISP_COLOR0) |
			(1LL << CMDQ_ENG_DISP_AAL) |
			(1LL << CMDQ_ENG_DISP_GAMMA) |
			(1LL << CMDQ_ENG_DISP_UFOE) | (1LL << CMDQ_ENG_DISP_DSI0_CMD));
		break;
	case CMDQ_SCENARIO_MHL_DISP:
	case CMDQ_SCENARIO_RDMA1_DISP:
		flag = ((1LL << CMDQ_ENG_DISP_RDMA1) | (1LL << CMDQ_ENG_DISP_DPI));
		break;
	case CMDQ_SCENARIO_DISP_CONFIG_OD:
		flag = 0LL;
		break;
	case CMDQ_SCENARIO_TRIGGER_LOOP:
		/* Trigger loop does not related to any HW by itself. */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_USER_SPACE:
		/* user space case, engine flag is passed seprately */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DEBUG:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_ESD_CHECK:
	case CMDQ_SCENARIO_DISP_SCREEN_CAPTURE:
		/* ESD check uses separate thread (not config, not trigger) */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_COLOR:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
		/* color path */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_MIRROR_MODE:
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH:
	case CMDQ_SCENARIO_DISP_SUB_DISABLE_SECURE_PATH:
		/* secure path */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_SECURE_NOTIFY_LOOP:
		flag = 0LL;
		break;

	default:
		CMDQ_ERR("Unknown scenario type %d\n", scn);
		flag = 0LL;
		break;
	}

	return flag;
}

void cmdq_core_gpr_dump(void)
{
	int i = 0;
	long offset = 0;
	uint32_t value = 0;

	CMDQ_LOG("========= GPR dump ========= \n");
	for (i = 0; i < 16; i++) {
		offset = CMDQ_GPR_R32(i);
		value = CMDQ_REG_GET32(offset);
		CMDQ_LOG("[GPR %2d]+0x%lx = 0x%08x\n", i, offset, value);
	}
	CMDQ_LOG("========= GPR dump ========= \n");

	return;

}

void cmdq_test_setup(void)
{
	/* unconditionally set CMDQ_SYNC_TOKEN_CONFIG_ALLOW and mutex STREAM_DONE */
	/* so that DISPSYS scenarios may pass check. */
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX0_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX1_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX2_STREAM_EOF);
	cmdqCoreSetEvent(CMDQ_EVENT_MUTEX3_STREAM_EOF);
}

void cmdq_test_cleanup(void)
{
	return; /* do nothing */
}
