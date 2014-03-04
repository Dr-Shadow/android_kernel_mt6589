#include <ccci_common.h>
#include <ccci_rpc.h>
#include <ccci_fs.h>
#include <ccci_tty.h>
#include <ccci_ipc.h>
#include <ccmni_net.h>
#include <ccci_md.h>
#include <linux/dma-mapping.h>



//==============================================================//
// macro and variable define about md sys settings
//==============================================================//
#define MD1_DEV_MAJOR		(184)
#define MD2_DEV_MAJOR		(169)
#define MD_SYS1_NET_VER		(2)
#define MD_SYS2_NET_VER		(1)

//#define CCCI_PLATFORM_L 		0x3536544D
//#define CCCI_PLATFORM_H 		0x31453537
#define CCCI_PLATFORM 			"MT6589E1"
#define CCCI1_DRIVER_VER 		0x20121001
#define CCCI2_DRIVER_VER 		0x20121001


static unsigned int ccci_drv_ver[MAX_MD_NUM] = {CCCI1_DRIVER_VER, CCCI2_DRIVER_VER};

static int net_ver_cfg[MAX_MD_NUM] = {MD_SYS1_NET_VER, MD_SYS2_NET_VER};
static int net_v2_dl_ctl_mem_size[MAX_MD_NUM] = {CCMNI_DL_CTRL_MEM_SIZE, 0};
static int net_v2_ul_ctl_mem_size[MAX_MD_NUM] = {CCMNI_UL_CTRL_MEM_SIZE, 0};


//==============================================================//
// macro and variable define about md memory layout settings
//==============================================================//
// Common
#define CCCI_MD_EX_EXP_INFO_SMEM_SIZE		(sizeof(modem_exception_exp_t))//reserve for dump md info when AP no physical channel
//#define CCCI_MISC_INFO_SMEM_SIZE			(2*1024)

// For MD1
#define CCCI1_RPC_SMEM_SIZE		((sizeof(RPC_BUF)+sizeof(unsigned char)*RPC1_MAX_BUF_SIZE) * RPC1_REQ_BUF_NUM)
#define CCCI1_TTY_SMEM_SIZE		(sizeof(char)*(CCCI1_TTY_BUF_SIZE*2)+sizeof(shared_mem_tty_t))
#define CC1MNI_V1_SMEM_SIZE		(sizeof(char)*CCCI1_CCMNI_BUF_SIZE*2 + sizeof(shared_mem_tty_t))
#define CC1MNI_V2_SMEM_UL_SIZE	CCCI_CCMNI_SMEM_UL_SIZE
#define CC1MNI_V2_SMEM_DL_SIZE	CCCI_CCMNI_SMEM_DL_SIZE
#define CC1MNI_V2_SMEM_SIZE		CCCI_CCMNI_SMEM_SIZE

// For MD2
#define CCCI2_RPC_SMEM_SIZE		((sizeof(RPC_BUF)+sizeof(unsigned char)*RPC2_MAX_BUF_SIZE) * RPC2_REQ_BUF_NUM)
#define CCCI2_TTY_SMEM_SIZE		(sizeof(char)*(CCCI2_TTY_BUF_SIZE*2)+sizeof(shared_mem_tty_t))
#define CC2MNI_V1_SMEM_SIZE		(sizeof(char)*CCCI2_CCMNI_BUF_SIZE*2 + sizeof(shared_mem_tty_t))
#define CC2MNI_V2_SMEM_UL_SIZE	0
#define CC2MNI_V2_SMEM_DL_SIZE	0
#define CC2MNI_V2_SMEM_SIZE		0

static unsigned int tty_smem_size[MAX_MD_NUM] = {CCCI1_TTY_SMEM_SIZE, CCCI2_TTY_SMEM_SIZE};
static unsigned int pcm_smem_size[MAX_MD_NUM] = {CCCI1_PCM_SMEM_SIZE, CCCI2_PCM_SMEM_SIZE};
static unsigned int rpc_smem_size[MAX_MD_NUM] = {CCCI1_RPC_SMEM_SIZE, CCCI2_RPC_SMEM_SIZE};
static unsigned int md_log_smem_size[MAX_MD_NUM] = {CCCI1_MD_LOG_SIZE, CCCI2_MD_LOG_SIZE};
static unsigned int net_v1_smem_size[MAX_MD_NUM] = {CC1MNI_V1_SMEM_SIZE, CC2MNI_V1_SMEM_SIZE};
static unsigned int net_smem_ul_size[MAX_MD_NUM] = {CC1MNI_V2_SMEM_UL_SIZE, CC2MNI_V2_SMEM_UL_SIZE};
static unsigned int net_smem_dl_size[MAX_MD_NUM] = {CC1MNI_V2_SMEM_DL_SIZE, CC2MNI_V2_SMEM_DL_SIZE};
static unsigned int net_v2_smem_size[MAX_MD_NUM] = {CC1MNI_V2_SMEM_SIZE, CC2MNI_V2_SMEM_SIZE};

smem_alloc_t		md_smem_tab[MAX_MD_NUM];
ccci_mem_layout_t	md_mem_layout_tab[MAX_MD_NUM];

extern int		set_ap_smem_remap(int md_id, unsigned int src, unsigned int des);
extern int		set_md_smem_remap(int md_id, unsigned int src, unsigned int des);
extern int		set_md_rom_rw_mem_remap(int md_id, unsigned int src, unsigned int des);




//**************************************************************************//
//*API about modem system settings                                                                        *//
//*                                                                                                                       *//
//*************************************************************************//
unsigned int get_md_sys_max_num(void)
{
	return MAX_MD_NUM;
}
EXPORT_SYMBOL(get_md_sys_max_num);


unsigned int is_md_enable(int md_id)
{
	switch(md_id)
	{
		case MD_SYS1:
			#if defined(MTK_ENABLE_MD1)
			return 1;
			#else
			return 0;
			#endif

		case MD_SYS2:
			#if defined(MTK_ENABLE_MD2)
			return 1;
			#else
			return 0;
			#endif

		default:
			return 0;
	}
}
EXPORT_SYMBOL(is_md_enable);


int get_dev_major_for_md_sys(int md_id)
{
	if(md_id == MD_SYS1)
		return MD1_DEV_MAJOR;
	else if(md_id == MD_SYS2)
		return MD2_DEV_MAJOR;
	
	return -1;
}
EXPORT_SYMBOL(get_dev_major_for_md_sys);


void platform_set_runtime_data(int md_id, modem_runtime_t *runtime)
{
	char str[16];
	snprintf(str,16,"%s", CCCI_PLATFORM);
	runtime->Platform_L = *((int*)str);
    runtime->Platform_H = *((int*)&str[4]);
	runtime->DriverVersion = ccci_drv_ver[md_id];
}
EXPORT_SYMBOL(platform_set_runtime_data);


int ccci_get_sub_module_cfg(int md_id, char name[], char out_buf[], int size)
{
	int actual_size = 0;
	rpc_cfg_inf_t rpc_cfg = {2, 2048};

	if(strcmp(name, "rpc")==0) {
		if(out_buf==NULL)
			return -CCCI_ERR_INVALID_PARAM;

		if(sizeof(rpc_cfg_inf_t)>size)
			actual_size = size;
		else
			actual_size = sizeof(rpc_cfg_inf_t);

		memcpy(out_buf, &rpc_cfg, actual_size);
	} else if (strcmp(name, "tty")==0) {
		*((int *)out_buf) = tty_smem_size[md_id];
		actual_size = sizeof(int);
	} else if (strcmp(name, "net")==0) {
		*((int *)out_buf) = net_ver_cfg[md_id];
		actual_size = sizeof(int);
	} else if (strcmp(name, "net_dl_ctl")==0) {
		*((int *)out_buf) = net_v2_dl_ctl_mem_size[md_id];
		actual_size = sizeof(int);
	} else if (strcmp(name, "net_ul_ctl")==0) {
		*((int *)out_buf) = net_v2_ul_ctl_mem_size[md_id];
		actual_size = sizeof(int);
	}
	return actual_size;
}
EXPORT_SYMBOL(ccci_get_sub_module_cfg);


//**************************************************************************//
//*API about modem memory layout settings                                                             *//
//*                                                                                                                       *//
//*************************************************************************//
static int cal_md_smem_size(int md_id)
{
	int smem_size = 0;

	switch(md_id)
	{
		case MD_SYS1:
			smem_size =
				round_up(MD_EX_LOG_SIZE+CCCI_MD_EX_EXP_INFO_SMEM_SIZE+CCCI_MD_RUNTIME_DATA_SMEM_SIZE+CCCI_MISC_INFO_SMEM_SIZE, 0x1000) +
				round_up(CCCI1_PCM_SMEM_SIZE, 0x1000) +
				round_up(CCCI1_MD_LOG_SIZE, 0x1000) +
				round_up(CCCI1_RPC_SMEM_SIZE, 0x1000) +
				round_up(CCCI_FS_SMEM_SIZE, 0x1000) +
				CCCI1_TTY_SMEM_SIZE*CCCI1_TTY_PORT_NUM +
				CC1MNI_V1_SMEM_SIZE*CCCI1_CCMNI_V1_PORT_NUM +
				CCCI_IPC_SMEM_SIZE +
				CC1MNI_V2_SMEM_UL_SIZE +
				CC1MNI_V2_SMEM_DL_SIZE +
				(CC1MNI_V2_SMEM_SIZE*CCMNI_V2_PORT_NUM);
			smem_size = round_up(smem_size,0x4000);//????can change to 0x1000 aglin, need verify
			break;

		case MD_SYS2:
			smem_size =
				round_up(MD_EX_LOG_SIZE+CCCI_MD_EX_EXP_INFO_SMEM_SIZE+CCCI_MD_RUNTIME_DATA_SMEM_SIZE+CCCI_MISC_INFO_SMEM_SIZE, 0x1000) +
				round_up(CCCI2_PCM_SMEM_SIZE, 0x1000) +
				round_up(CCCI2_MD_LOG_SIZE, 0x1000) +
				round_up(CCCI2_RPC_SMEM_SIZE, 0x1000) +
				round_up(CCCI_FS_SMEM_SIZE, 0x1000) +
				CCCI2_TTY_SMEM_SIZE*CCCI2_TTY_PORT_NUM +
				CC2MNI_V1_SMEM_SIZE*CCCI2_CCMNI_V1_PORT_NUM +
				CCCI_IPC_SMEM_SIZE +
				CC2MNI_V2_SMEM_UL_SIZE +
				CC2MNI_V2_SMEM_DL_SIZE +
				(CC2MNI_V2_SMEM_SIZE*CCMNI_V2_PORT_NUM);
			smem_size = round_up(smem_size,0x4000);//????can change to 0x1000 aglin, need verify
			break;

		default:
			break;
	}
	
	CCCI_DBG_MSG(md_id, "ctl", "md%d share memory size: 0x%08x\n", md_id+1, smem_size);
	return smem_size;
}


static int cfg_md_mem_layout(int md_id)
{
	unsigned int	md_len;
	unsigned int	dsp_len;
	unsigned int	smem_base_before_map;
	unsigned int	md_rom_mem_base;
	int				ret = 0;

	#ifdef MD_IMG_SIZE_ADJUST_BY_VER
	if(get_ap_img_ver() == AP_IMG_2G)
		md_len = DSP_REGION_BASE_2G;
	else if(get_ap_img_ver() == AP_IMG_3G)
		md_len = DSP_REGION_BASE_3G;
	else
		md_len = DSP_REGION_BASE_3G;	
	dsp_len = DSP_REGION_LEN;
	#else
	md_len = MD_IMG_RESRVED_SIZE + MD_RW_MEM_RESERVED_SIZE;
	dsp_len = 0;
	#endif
	
	// MD image
	md_mem_layout_tab[md_id].md_region_phy = get_md_mem_start_addr(md_id);
	md_mem_layout_tab[md_id].md_region_size = md_len;
	md_rom_mem_base = md_mem_layout_tab[md_id].md_region_phy;

	// DSP image
	md_mem_layout_tab[md_id].dsp_region_phy = get_md_mem_start_addr(md_id) + md_len;
	md_mem_layout_tab[md_id].dsp_region_size = dsp_len;

	// Share memory
	smem_base_before_map = get_md_share_mem_start_addr(md_id);
	md_mem_layout_tab[md_id].smem_region_phy = smem_base_before_map - get_smem_base_addr(md_id) + 0x40000000; // 0x40000000 is BANK4 start addr
	md_mem_layout_tab[md_id].smem_region_size = CCCI_SHARED_MEM_SIZE;
			
	// Store address before mapping
	md_mem_layout_tab[md_id].smem_region_phy_before_map = smem_base_before_map;

	// Set share memory remapping
	set_ap_smem_remap(md_id, 0x40000000, smem_base_before_map); 
	set_md_smem_remap(md_id, 0x40000000, smem_base_before_map); 

	// Set md image and rw runtime memory remapping
	set_md_rom_rw_mem_remap(md_id, 0x00000000, md_rom_mem_base); 

	return ret;
}


int ccci_alloc_smem(int md_id)
{
	ccci_mem_layout_t	*mem_layout_ptr = NULL;
	int					ret = 0;
	int					smem_size = 0;
	int					*smem_vir;
	dma_addr_t			smem_phy;
	int					i,j, base_virt, base_phy;
	int					size;

	smem_size = cal_md_smem_size(md_id);
	ret = cfg_md_mem_layout(md_id);
	if(ret < 0) {
		CCCI_MSG_INF(md_id, "ctl", "md mem layout config fail\n");
		return ret;
	}
	mem_layout_ptr = &md_mem_layout_tab[md_id];

#ifdef CCCI_STATIC_SHARED_MEM
	if (CCCI_SHARED_MEM_SIZE < smem_size) {
	    CCCI_MSG_INF(md_id, "ctl", "[error]CCCI shared mem isn't enough: 0x%08X\n", smem_size);
	    return -ENOMEM;
	}
	smem_phy = mem_layout_ptr->smem_region_phy;
	smem_vir = (int*)ioremap_nocache((unsigned long)smem_phy, smem_size);
	if (!smem_vir) {
		CCCI_MSG_INF(md_id, "ctl", "ccci smem ioremap fail\n");
		return -ENOMEM;
	}

#else // dynamic allocation shared memory

	smem_vir = dma_alloc_coherent(NULL, smem_size, &smem_phy, GFP_KERNEL);
	if (smem_vir == NULL) {
		CCCI_MSG_INF(md_id, "ctl", "ccci smem dma_alloc_coherent fail\n");
		return -CCCI_ERR_GET_MEM_FAIL;
	}
	mem_layout_ptr->smem_region_phy = (unsigned int)smem_phy;
	mem_layout_ptr->smem_region_size = smem_size;

#endif

	mem_layout_ptr->smem_region_vir = (unsigned int)smem_vir;
	CCCI_CTL_MSG(md_id, "ccci_smem_phy=%x, ccci_smem_size=%d, ccci_smem_virt=%x\n", 
			(unsigned int)smem_phy, smem_size, (unsigned int)smem_vir);

	WARN_ON(smem_phy&(0x4000-1)||smem_size&(0x4000-1));

	// Memory allocate done, config for each sub module
	base_virt = (int)smem_vir;
	base_phy = (int)smem_phy;

	// Total
	md_smem_tab[md_id].ccci_smem_vir = base_virt;
	md_smem_tab[md_id].ccci_smem_phy = base_phy;
	md_smem_tab[md_id].ccci_smem_size = smem_size;

	//MD runtime data!! Note: This item must be the first!!!
	md_smem_tab[md_id].ccci_md_runtime_data_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_md_runtime_data_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_md_runtime_data_smem_size = CCCI_MD_RUNTIME_DATA_SMEM_SIZE;
	base_virt += CCCI_MD_RUNTIME_DATA_SMEM_SIZE;
	base_phy += CCCI_MD_RUNTIME_DATA_SMEM_SIZE;

	// EXP
	md_smem_tab[md_id].ccci_exp_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_exp_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_exp_smem_size = MD_EX_LOG_SIZE;
	base_virt += MD_EX_LOG_SIZE;
	base_phy += MD_EX_LOG_SIZE;

	//MD Exception expand Info
	md_smem_tab[md_id].ccci_md_ex_exp_info_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_md_ex_exp_info_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_md_ex_exp_info_smem_size = CCCI_MD_EX_EXP_INFO_SMEM_SIZE;
	base_virt += CCCI_MD_EX_EXP_INFO_SMEM_SIZE;
	base_phy += CCCI_MD_EX_EXP_INFO_SMEM_SIZE;

	//Misc info
	md_smem_tab[md_id].ccci_misc_info_base_virt = base_virt;
	md_smem_tab[md_id].ccci_misc_info_base_phy = base_phy;
	md_smem_tab[md_id].ccci_misc_info_size = CCCI_MISC_INFO_SMEM_SIZE;
	base_virt += CCCI_MISC_INFO_SMEM_SIZE;
	base_phy += CCCI_MISC_INFO_SMEM_SIZE;

	base_virt = round_up(base_virt, 0x1000);
	base_phy = round_up(base_phy, 0x1000);

	// PCM
	md_smem_tab[md_id].ccci_pcm_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_pcm_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_pcm_smem_size = pcm_smem_size[md_id];
	size = round_up(pcm_smem_size[md_id], 0x1000);
	base_virt += size;
	base_phy += size;

	// LOG
	md_smem_tab[md_id].ccci_mdlog_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_mdlog_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_mdlog_smem_size = md_log_smem_size[md_id];
	size = round_up(md_log_smem_size[md_id], 0x1000);
	base_virt += size;
	base_phy += size;

	// RPC
	md_smem_tab[md_id].ccci_rpc_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_rpc_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_rpc_smem_size = rpc_smem_size[md_id];
	size = round_up(rpc_smem_size[md_id], 0x1000);
	base_virt += size;
	base_phy += size;

	// FS
	md_smem_tab[md_id].ccci_fs_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_fs_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_fs_smem_size = CCCI_FS_SMEM_SIZE;
	size = round_up(CCCI_FS_SMEM_SIZE, 0x1000);
	base_virt += size;
	base_phy += size;

	// TTY: tty_muxd(uart0), tty_meta(uart1), ccmni1(uart2), ccmni2(uart3), ccmni3(uart4), tty_ipc(uart5)
	j = 0;
	for (i = 0; i < CCCI1_TTY_PORT_NUM-1; i++, j++) {
		md_smem_tab[md_id].ccci_tty_smem_base_virt[i] = base_virt;
		md_smem_tab[md_id].ccci_tty_smem_base_phy[i] = base_phy;
		md_smem_tab[md_id].ccci_tty_smem_size[i] = tty_smem_size[md_id];
		base_virt += tty_smem_size[md_id];
		base_phy += tty_smem_size[md_id];
	}
	for (i = 0; i < CCCI1_CCMNI_V1_PORT_NUM; i++, j++) {
		if(net_v1_smem_size[md_id] == 0) {
			md_smem_tab[md_id].ccci_tty_smem_base_virt[j] = 0;
			md_smem_tab[md_id].ccci_tty_smem_base_phy[j] = 0;
			md_smem_tab[md_id].ccci_tty_smem_size[j] = 0;
		} else {
			md_smem_tab[md_id].ccci_tty_smem_base_virt[j] = base_virt;
			md_smem_tab[md_id].ccci_tty_smem_base_phy[j] = base_phy;
			md_smem_tab[md_id].ccci_tty_smem_size[j] = net_v1_smem_size[md_id];
			base_virt += net_v1_smem_size[md_id];
			base_phy += net_v1_smem_size[md_id];
		}
	}
	md_smem_tab[md_id].ccci_tty_smem_base_virt[j] = base_virt; // TTY for IPC
	md_smem_tab[md_id].ccci_tty_smem_base_phy[j] = base_phy;
	md_smem_tab[md_id].ccci_tty_smem_size[j] = tty_smem_size[md_id];
	base_virt += tty_smem_size[md_id];
	base_phy += tty_smem_size[md_id];
	j++;
	for (; j < UART_MAX_PORT_NUM; j++) {
		md_smem_tab[md_id].ccci_tty_smem_base_virt[j] = 0;
		md_smem_tab[md_id].ccci_tty_smem_base_phy[j] = 0;
		md_smem_tab[md_id].ccci_tty_smem_size[j] = 0;
	}

	// PMIC
	md_smem_tab[md_id].ccci_pmic_smem_base_virt = 0;
	md_smem_tab[md_id].ccci_pmic_smem_base_phy = 0;
	md_smem_tab[md_id].ccci_pmic_smem_size = 0;
	base_virt += 0;
	base_phy += 0;

	// SYS
	md_smem_tab[md_id].ccci_sys_smem_base_virt = 0;
	md_smem_tab[md_id].ccci_sys_smem_base_phy = 0;
	md_smem_tab[md_id].ccci_sys_smem_size = 0;
	base_virt += 0;
	base_phy += 0;
	
	//IPC
	md_smem_tab[md_id].ccci_ipc_smem_base_virt = base_virt;
	md_smem_tab[md_id].ccci_ipc_smem_base_phy = base_phy;
	md_smem_tab[md_id].ccci_ipc_smem_size = CCCI_IPC_SMEM_SIZE;
	base_virt += CCCI_IPC_SMEM_SIZE ;
	base_phy +=  CCCI_IPC_SMEM_SIZE;

	// CCMNI_V2 -- UP-Link
	if(net_smem_ul_size[md_id] != 0) {
		md_smem_tab[md_id].ccci_ccmni_smem_ul_base_virt = base_virt;
		md_smem_tab[md_id].ccci_ccmni_smem_ul_base_phy = base_phy;
		md_smem_tab[md_id].ccci_ccmni_smem_ul_size = net_smem_ul_size[md_id];
		base_virt += net_smem_ul_size[md_id];
		base_phy += net_smem_ul_size[md_id];
	} else {
		md_smem_tab[md_id].ccci_ccmni_smem_ul_base_virt = 0;
		md_smem_tab[md_id].ccci_ccmni_smem_ul_base_phy = 0;
		md_smem_tab[md_id].ccci_ccmni_smem_ul_size = 0;
	}

	// CCMNI_V2 --DOWN-Link
	if(net_smem_dl_size[md_id] != 0) {
		md_smem_tab[md_id].ccci_ccmni_smem_dl_base_virt = base_virt;
		md_smem_tab[md_id].ccci_ccmni_smem_dl_base_phy = base_phy;
		md_smem_tab[md_id].ccci_ccmni_smem_dl_size = net_smem_dl_size[md_id];
		base_virt += net_smem_dl_size[md_id];
		base_phy += net_smem_dl_size[md_id];
	} else {
		md_smem_tab[md_id].ccci_ccmni_smem_dl_base_virt = 0;
		md_smem_tab[md_id].ccci_ccmni_smem_dl_base_phy = 0;
		md_smem_tab[md_id].ccci_ccmni_smem_dl_size = 0;
	}

	// CCMNI_V2 --Ctrl memory
	for (i = 0; i < CCMNI_CHANNEL_CNT; i++) {
		if (net_v2_smem_size[md_id] == 0) {
			md_smem_tab[md_id].ccci_ccmni_ctl_smem_base_virt[i] = 0;
			md_smem_tab[md_id].ccci_ccmni_ctl_smem_base_phy[i] = 0;
			md_smem_tab[md_id].ccci_ccmni_ctl_smem_size[i] = 0;
		} else {
			md_smem_tab[md_id].ccci_ccmni_ctl_smem_base_virt[i] = base_virt;
			md_smem_tab[md_id].ccci_ccmni_ctl_smem_base_phy[i] = base_phy;
			md_smem_tab[md_id].ccci_ccmni_ctl_smem_size[i] = net_v2_smem_size[md_id];
		}
		memset((void*)base_virt, 0, net_v2_smem_size[md_id]);
		base_virt += net_v2_smem_size[md_id];
		base_phy += net_v2_smem_size[md_id];
	}

	for (; i < CCMNI_MAX_CHANNELS; i++){
		md_smem_tab[md_id].ccci_ccmni_ctl_smem_base_virt[i] = 0;
		md_smem_tab[md_id].ccci_ccmni_ctl_smem_base_phy[i] = 0;
		md_smem_tab[md_id].ccci_ccmni_ctl_smem_size[i] = 0;
	}

	return ret;
}
EXPORT_SYMBOL(ccci_alloc_smem);


void ccci_free_smem(int md_id)
{
	ccci_mem_layout_t	*mem_layout_ptr = NULL;
	
	mem_layout_ptr = &md_mem_layout_tab[md_id];
	
#ifdef CCCI_STATIC_SHARED_MEM
	if (mem_layout_ptr->smem_region_vir)
		iounmap((void*)mem_layout_ptr->smem_region_vir);
#else
	if (mem_layout_ptr->smem_region_vir) {
		dma_free_coherent(NULL, 
						  mem_layout_ptr->smem_region_size, 
						  (int*)mem_layout_ptr->smem_region_vir, 
						  (dma_addr_t)mem_layout_ptr->smem_region_phy);
	}
#endif
}
EXPORT_SYMBOL(ccci_free_smem);


smem_alloc_t* get_md_smem_layout(int md_id)
{
	if(md_id >= (sizeof(md_smem_tab)/sizeof(smem_alloc_t)))
		return NULL;
	return &md_smem_tab[md_id];
}
EXPORT_SYMBOL(get_md_smem_layout);


ccci_mem_layout_t* get_md_sys_layout(int md_id)
{
	if(md_id >= (sizeof(md_mem_layout_tab)/sizeof(ccci_mem_layout_t)))
		return NULL;
	return &md_mem_layout_tab[md_id];
}
EXPORT_SYMBOL(get_md_sys_layout);



