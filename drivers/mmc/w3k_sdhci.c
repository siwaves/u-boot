// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Siliconwaves Corporation
 *		      Richard Dai <richard@siliconwaves.com>
 */

#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <asm/global_data.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/libfdt.h>
#include <malloc.h>
#include <sdhci.h>
#include <asm/cache.h>
#define HS400
#define W3K_MMC_MIN_CLK 400000
#define W3K_MMC_MAX_CLK 50000000
#define W3K_SDHCI_SOFTWARE_RESET	0x00 
#define  W3K_SDHCI_RESET_ALL	0x04
#define W3K_SDHCI_CLOCK_CONTROL	0x00
#define W3K_SDHCI_ADMA_SAR_REG	0x10c
#define W3K_SDHCI_STA_REG		0x020
#define W3K_SDHCI_ARGUMENT		0x01C
#define W3K_SDHCI_BLOCK_COUNT	0x034
#define W3K_SDHCI_INT_STATUS	0x00C
#define W3K_SDHCI_DMA_STATUS	0x110
#define W3K_SDHCI_DMA_CTL_REG 	0x110
#define W3K_SDHCI_DMA_CTL		0x108
#define W3K_SRAM_TO_MEM_FLAG 1
#define W3K_SDHCI_MAKE_CMD(c, f) ((c & 0x3f) | ((f & 0x7) << 7))
DECLARE_GLOBAL_DATA_PTR;
void *aligned_buffer;
#define W3C_SDHCI_SDMA_ENABLE 1
struct w3k_sdhci_adma_desc {
	u8 attr;
	u8 reserved;
	u16 len;
	u32 addr;
} __packed;

static void w3k_sdhci_adma_desc(struct w3k_sdhci_adma_desc *desc,
			    u32 addr, u16 len, bool end)
{
	u8 attr;

	attr = ADMA_DESC_ATTR_VALID | ADMA_DESC_TRANSFER_DATA;
	if (end)
		attr |= ADMA_DESC_ATTR_END;

	desc->attr = attr;
	desc->len = len;
	desc->reserved = 0;
	desc->addr = addr;
}

struct w3k_sdhci_adma_desc *w3k_sdhci_adma_init(uint total_len)
{
	uint desc_count = DIV_ROUND_UP(total_len, ADMA_MAX_LEN);
	return  (struct w3k_sdhci_adma_desc *)
				memalign(64, desc_count *
				sizeof(struct w3k_sdhci_adma_desc));
}

void w3k_sdhci_adma_start(struct sdhci_host *host, void *addr, int dir)
{
	if(dir & MMC_DATA_WRITE)
		sdhci_writel(host, 0x000000000, W3K_SDHCI_DMA_CTL);
	else 
		sdhci_writel(host, 0x80000000, W3K_SDHCI_DMA_CTL);
	
	sdhci_writel(host, (unsigned int)addr, W3K_SDHCI_ADMA_SAR_REG);
}

void w3k_sdhci_prepare_adma_table(struct w3k_sdhci_adma_desc *table,
			      struct mmc_data *data)
{
	uint trans_bytes = data->blocksize * data->blocks;
	uint desc_count = DIV_ROUND_UP(trans_bytes, ADMA_MAX_LEN);
	struct w3k_sdhci_adma_desc *desc = table;
	int i = desc_count;
	uint addr;
	
	if (data->flags & MMC_DATA_READ)
		addr = data->dest;
	else
		addr = data->src;

	while (--i ) {
		w3k_sdhci_adma_desc(desc, addr, ADMA_MAX_LEN, false);
		addr += ADMA_MAX_LEN;
		trans_bytes -= ADMA_MAX_LEN;
		desc++;
	}

	w3k_sdhci_adma_desc(desc, addr, trans_bytes, true);
	desc = table;
	for(i = 0; i < desc_count; i++){
		debug("attr:0x%x, len = %d, addr = 0x%x\n", desc->attr, desc->len, desc->addr);
		desc++;
	}
	//flush_cache((dma_addr_t)table,
	//	    ROUND(desc_count * sizeof(struct sdhci_adma_desc),
	//		  ARCH_DMA_MINALIGN));
}


struct w3k_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct w3k_sdhci_priv {
	struct sdhci_host host;
	unsigned int clock;
};
static void w3k_sdhci_dma_flush(struct sdhci_host *host);
static void w3k_sdhci_hw_reset(struct sdhci_host *host);
static void w3k_sdhci_clk_ctrl(struct sdhci_host *host, int enable);
static void dcache_l1_invalid(void)
{
	__asm__ volatile(".word 0xFC200073": : : "memory");
}

void ccache2_flush(u64 flush_addr, int len) {
	int i = 0;
	__asm volatile("fence rw, io" : : : "memory");
	do{
		*(volatile u64*)(0x2010200) = flush_addr + i;
		len -= 64;
		i+=64;
	}while(len > 0);
	__asm volatile("fence io, rw" : : : "memory");
}


static void sdhci_disable_dma(struct sdhci_host *host)
{
	debug("%s()\n", __func__);

	sdhci_writel(host, 0x00020000, W3K_SDHCI_DMA_CTL);
	debug("write W3K_SDHCI_DMA_CTL(0x%x): 0x%x\n", W3K_SDHCI_DMA_CTL, 0x00020000);

	while (sdhci_readl(host, W3K_SDHCI_DMA_CTL) & 0x00020000);
}

static int sdhci_transfer_data(struct sdhci_host *host, struct mmc_data *data,
				unsigned int start_addr)
{
	unsigned int int_stat, dma_stat, timeout, sync_flag = 0;

	debug("%s()\n", __func__);

	timeout = 1000000;

	do {
		int_stat = sdhci_readl(host, W3K_SDHCI_INT_STATUS);
		dma_stat = sdhci_readl(host, W3K_SDHCI_DMA_STATUS);

		if ((dma_stat & 0xF) != 0x0) {
			if ((dma_stat & 0x2) || (dma_stat & 0x8)) {
				sync_flag |= 0x4;
				debug("%s: DMA done\n", __func__);
			}
			if (dma_stat & 0x4) {
				debug("%s: Error detected in status(0x%X)!\n",
				       __func__, dma_stat);
			}

			/* clear dma interrupt */
			sdhci_writel(host, dma_stat, W3K_SDHCI_DMA_STATUS);
			debug("write W3K_SDHCI_DMA_STATUS(0x%x): 0x%x\n", W3K_SDHCI_DMA_STATUS, dma_stat);
		}
#define  W3K_DAT_DONE	0x00001000	/* data transfer done */
		if (int_stat != 0x0) {
			/* handle data requests */
			if (int_stat & W3K_DAT_DONE) {
				sync_flag |= 0x2;
				debug("%s: data done\n", __func__);
			}
#define  W3K_DAT_TIMEOUT	0x00004000	/* data timeout */
#define  W3K_DAT_CRC_ERROR	0x00008000	/* data crc error */
			if ((int_stat & W3K_DAT_TIMEOUT) ||
			    (int_stat & W3K_DAT_CRC_ERROR)) {
				sdhci_disable_dma(host);

				debug("%s: Error detected in status(0x%X)!\n",
				       __func__, int_stat);
				return -1;
			}

			/* clear controller interrupt */
			sdhci_writel(host, int_stat, W3K_SDHCI_INT_STATUS);
			debug("write W3K_SDHCI_INT_STATUS(0x%x): 0x%x\n", W3K_SDHCI_INT_STATUS, int_stat);
		}

		if (sync_flag == 0x6) {
			return 0;
		}

		if (timeout-- > 0) {
			udelay(10);
		} else {
			debug("%s: Transfer data timeout\n", __func__);
			return -1;
		}
	} while (1);

	return 0;
}


static void sdhci_cmd_done(struct sdhci_host *host, struct mmc_cmd *cmd)
{
	int i;

	debug("%s()\n", __func__);
#define W3K_SDHCI_RESPONSE		0x24
	if (cmd->resp_type & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0; i < 4; i++) {
			cmd->response[i] = sdhci_readl(host,
					W3K_SDHCI_RESPONSE + (3 - i) * 4);
		}
	} else {
		cmd->response[0] = sdhci_readl(host, W3K_SDHCI_RESPONSE);
	}
}

static void w3k_sdhci_reset(struct sdhci_host *host, u32 mask)
{
	unsigned long timeout;
	unsigned int val;

	debug("%s()\n", __func__);
	/* Wait max 100 ms */
	timeout = 100;
	val = sdhci_readl(host, W3K_SDHCI_SOFTWARE_RESET);
	val |= mask;
	sdhci_writel(host, val, W3K_SDHCI_SOFTWARE_RESET);
	debug("write W3K_SDHCI_SOFTWARE_RESET(0x%x): 0x%x\n", W3K_SDHCI_SOFTWARE_RESET, val);

	while (sdhci_readl(host, W3K_SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			debug("%s: Reset 0x%x never completed.\n",
			       __func__, (int)mask);
			return;
		}
		timeout--;
		udelay(1000);
	}
}

static int __w3k_sdhci_send_command(struct udevice *dev, struct mmc_cmd *cmd,
		       struct mmc_data *data);

struct w3k_mmc_data {
	union {
		char *dest;
		char *src; /* src buffers don't get written to */
	};
	uint flags;
	uint blocks;
	uint blocksize;
};
#if 1
static int w3k_sdhci_send_command(struct udevice *dev, struct mmc_cmd *cmd,
		       struct mmc_data *data)
{
#define W3K_ONCE_TRAN_BYTES_LEN (1024*32)
	struct w3k_mmc_data data_tmp;
	struct w3k_mmc_data *ptr_data;
	struct mmc_cmd *ptr_cmd;
	uint size;
	int ret = -1;
	uint blocksize, blocks, count_blocks, last_size;
	uint tran_blocks;
	//cmd16 设置块大小的命令
	struct mmc_cmd set_blocksize_cmd = {
		.cmdidx = 0x10, 
		.resp_type = 0x15,
		.cmdarg = 0, //块大小
	};
	//读多个块的命令
	struct mmc_cmd read_multiblock_cmd = {
		.cmdidx = 0x12, 
		.resp_type = 0x15,
		.cmdarg = 0, //块位置
	};
	//写多个块的命令
	struct mmc_cmd write_multiblock_cmd = {
		.cmdidx = 0x19, 
		.resp_type = 0x15,
		.cmdarg = 0, //块位置
	};
	//停止传输命令
	struct mmc_cmd stop_tran_cmd = {
		.cmdidx = 0xc, 
		.resp_type = 0x1d,
		.cmdarg = 0,
	};
	//send状态传输命令
	struct mmc_cmd send_status_cmd = {
		.cmdidx = 0xd, 
		.resp_type = 0x15,
		.cmdarg = 0x10000,
	};

	if(data){
		size = data->blocksize *data->blocks;
		count_blocks = data->blocks;
		tran_blocks = W3K_ONCE_TRAN_BYTES_LEN/data->blocksize;
	}

	debug("\ncmdidx = 0x%x, resp_type = 0x%x, cmdarg= 0x%x\n", cmd->cmdidx, cmd->resp_type, cmd->cmdarg);
	if(data && count_blocks >=  tran_blocks){
		debug("count_blocks = %d, tran_blocks = %d\n", count_blocks, tran_blocks);
		if(count_blocks >= tran_blocks && count_blocks <= (tran_blocks*2)){
			debug("%s %d: data->blocks = %d\n", __func__, __LINE__, data->blocks);
			goto out;
		}
		set_blocksize_cmd.cmdarg = data->blocksize;
		
		memcpy(&data_tmp, data, sizeof(struct mmc_data));
		if (data->flags & MMC_DATA_READ)
			ptr_cmd = &read_multiblock_cmd;
		else
			ptr_cmd = &write_multiblock_cmd;
		ptr_cmd->cmdarg = cmd->cmdarg;
		ptr_data = &data_tmp;
		ptr_data->blocks = tran_blocks;
		
		while(1){
			ret = __w3k_sdhci_send_command(dev, ptr_cmd, (struct mmc_data *)ptr_data);
			if(ret < 0)
				break;
			/*重启一次传输*/
			count_blocks -= tran_blocks;
			size -= (ptr_data->blocks * data->blocksize);
			if(size == 0){
				memcpy(cmd->response, ptr_cmd->response, sizeof(ptr_cmd->response));
				break;
			}
			//停止传输
			ret = __w3k_sdhci_send_command(dev, &stop_tran_cmd, NULL);
			if(ret < 0)
				break;
			//send状态传输命令
			if(data->flags & MMC_DATA_WRITE){
				ret = __w3k_sdhci_send_command(dev, &send_status_cmd, NULL);
				if(ret < 0)
					break;
			}
		
			if(count_blocks >= tran_blocks && count_blocks <= (tran_blocks*2)){
				//一次性传输剩余数据
				ptr_data->blocks = count_blocks;
			}
			ptr_cmd->cmdarg += tran_blocks;

			if (data->flags & MMC_DATA_READ){
				ptr_data->dest += W3K_ONCE_TRAN_BYTES_LEN;
			}
			else{
				ptr_data->src += W3K_ONCE_TRAN_BYTES_LEN;
			}
			//设置块大小
			ret = __w3k_sdhci_send_command(dev, &set_blocksize_cmd, NULL);
			if(ret < 0)
				break;
		}	
	}else {
out:
		ret = __w3k_sdhci_send_command(dev, cmd, data);
	}
	return ret;
}
#endif
static int __w3k_sdhci_send_command(struct udevice *dev, struct mmc_cmd *cmd,
		       struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct sdhci_host *host = mmc->priv;
	unsigned int stat = 0;
	unsigned int command = 0, autocmd;
	int ret = 0;
	int trans_bytes = 0, is_aligned = 1;
	u32 flags = 0;
	unsigned int start_addr = 0;
#ifdef  W3K_SRAM_TO_MEM_FLAG 
	char* sram_addr = (char*)0x40050000;
	char* sram_addr_write = (char*)0x40020000;
#endif
	unsigned int retry = 10000;
#ifndef W3C_SDHCI_SDMA_ENABLE
	struct w3k_sdhci_adma_desc *adma_table = NULL;
#endif
	debug("%s(%d)\n", __func__, cmd->cmdidx);

	while (sdhci_readl(host, W3K_SDHCI_STA_REG) & 0x3);

	sdhci_writel(host, cmd->cmdarg, W3K_SDHCI_ARGUMENT);
	debug("write W3K_SDHCI_ARGUMENT(0x%x): 0x%x\n", W3K_SDHCI_ARGUMENT, cmd->cmdarg);

	/* Set Transfer mode regarding to data flag */
	if (data != 0) {
		w3k_sdhci_dma_flush(host);
		/* set controller data length */
		command = ((data->blocksize & 0xFFF) << 16);

		/* set controller block count */
		if (data->blocks == 1) {
			command |= 0x00000800;
		} else if (data->blocks > 1) {
			command |= 0x00001000;
		}

		if (data->flags & MMC_DATA_WRITE) {
			command |= 0x00002000;
		}

		sdhci_writel(host, data->blocks, W3K_SDHCI_BLOCK_COUNT);
		debug("write W3K_SDHCI_BLOCK_COUNT(0x%x): 0x%x\n", W3K_SDHCI_BLOCK_COUNT, data->blocks);

		stat = sdhci_readl(host, W3K_SDHCI_DMA_STATUS);
		sdhci_writel(host, stat, W3K_SDHCI_DMA_STATUS);
		debug("write W3K_SDHCI_DMA_STATUS(0x%x): 0x%x\n", W3K_SDHCI_DMA_STATUS, stat);
#if W3C_SDHCI_SDMA_ENABLE
		if (data->flags & MMC_DATA_WRITE) {
			sdhci_writel(host, 0x000000000, W3K_SDHCI_DMA_CTL);
			debug("write W3K_SDHCI_DMA_CTL(0x%x): 0x%x\n", W3K_SDHCI_DMA_CTL, 0x00000300);
		} else {
			sdhci_writel(host, 0x80000000, W3K_SDHCI_DMA_CTL);
			debug("write W3K_SDHCI_DMA_CTL(0x%x): 0x%x\n", W3K_SDHCI_DMA_CTL, 0x80000300);
		}
		trans_bytes = data->blocks * data->blocksize;
		if (data->flags & MMC_DATA_READ){
#ifdef W3K_SRAM_TO_MEM_FLAG
			start_addr = sram_addr;
#else
			start_addr = (unsigned int)data->dest;
			//ccache2_flush(start_addr, trans_bytes);
#endif
		}
		else{
#ifdef W3K_SRAM_TO_MEM_FLAG
			debug("write copy 0x%x from 0x%x len:%d\n", (unsigned int )sram_addr_write,  (unsigned int )data->src, trans_bytes);
			memcpy(sram_addr_write, data->src, trans_bytes);
			start_addr = (unsigned int)sram_addr_write;
#else
			start_addr = (unsigned int)data->src;
#endif
		}	
		debug("%s %d blocksize = %d, blocks = %d\n", __func__, __LINE__, data->blocksize, data->blocks);
#define ARCH_DMA_MINALIGN	128
		//flush_cache(start_addr, trans_bytes+ARCH_DMA_MINALIGN);
		//invalidate_dcache_range(start_addr, start_addr+trans_bytes+ARCH_DMA_MINALIGN);
		
#define W3K_SDHCI_SDMA_SADDR	0x0100
#define W3K_SDHCI_SDMA_LEN		0x0104
		sdhci_writel(host, start_addr, W3K_SDHCI_SDMA_SADDR);
		debug("write W3K_SDHCI_SDMA_SADDR(0x%x): 0x%x\n", W3K_SDHCI_SDMA_SADDR, start_addr);
		sdhci_writel(host, trans_bytes & 0XFFFFF, W3K_SDHCI_SDMA_LEN);
		debug("write W3K_SDHCI_SDMA_LEN(0x%x): 0x%x\n", W3K_SDHCI_SDMA_LEN, trans_bytes);
#else
		adma_table = w3k_sdhci_adma_init(data->blocks * data->blocksize);
		if(!adma_table){
			debug("%s %d: w3k_sdhci_adma_init error\n", __func__, __LINE__);
		}
		w3k_sdhci_prepare_adma_table(adma_table, data);
		debug("amda descriptor:0x%x\n", (unsigned int)adma_table);
		w3k_sdhci_adma_start(host, (void*)adma_table, data->flags);
#endif
	}
/* SDMMC_CMD bit fields */
#define  SD_RESP_NO	 0x0	/* no response */
#define  SD_RESP_R1	 0x1	/* R1/R5/R6/R7,R48-count */
#define  SD_RESP_R2	 0x2	/* R2, R136-count */
#define  SD_RESP_R3	 0x3	/* R3, R48 */
#define  SD_RESP_R4	 0x4	/* R4, R48 */
#define  SD_RESP_R1B 0x7	/* R1b */

	/* Translate mmc_resp_type(cmd) to known form of anarion sd/mmc controller */
	switch (cmd->resp_type) {
	case MMC_RSP_NONE:
		flags = SD_RESP_NO;
		break;
	case MMC_RSP_R1:
	case (MMC_RSP_PRESENT|MMC_RSP_OPCODE):
		flags = SD_RESP_R1;
		break;
	case MMC_RSP_R1b:
		flags = SD_RESP_R1B;
		break;
	case MMC_RSP_R2:
		flags = SD_RESP_R2;
		break;
	case MMC_RSP_R3:	/* MMC_RSP_R4 */
		flags = SD_RESP_R3;
		break;
	default :
		debug("Unknown response type!!!\n");
		flags = SD_RESP_NO;
		break;
	}

	if (cmd->resp_type & MMC_RSP_CRC)
		command |= 0x40000000;
	if(0){
	//if (W3K_SDHCI_MAKE_CMD(cmd->cmdidx, flags) == MMC_CMD_SET_BLOCK_COUNT) {
		command |= (2 << 28);
		autocmd = W3K_SDHCI_MAKE_CMD(cmd->cmdidx, flags);
		autocmd |= (SD_RESP_R1 << 7);
		autocmd |= (1 << 10);
		autocmd |= (0 << 11);	/* no data */
		autocmd |= (0 << 14);
		autocmd |= (1 << 30);
#define W3K_SDHCI_ACMD_REG		0x6C
		sdhci_writel(host, autocmd, W3K_SDHCI_ACMD_REG);
	}

	stat = sdhci_readl(host, W3K_SDHCI_INT_STATUS);
	sdhci_writel(host, stat, W3K_SDHCI_INT_STATUS);
	debug("write W3K_SDHCI_INT_STATUS(0x%x): 0x%x\n", W3K_SDHCI_INT_STATUS, stat);

	debug("command : 0x%x\n", W3K_SDHCI_MAKE_CMD(cmd->cmdidx, flags) | command);
#define W3K_SDHCI_COMMAND		0x18
	sdhci_writel(host, W3K_SDHCI_MAKE_CMD(cmd->cmdidx, flags) | command, W3K_SDHCI_COMMAND);
	debug("write W3K_SDHCI_COMMAND(0x%x): 0x%x\n", W3K_SDHCI_COMMAND, W3K_SDHCI_MAKE_CMD(cmd->cmdidx, flags) | command);

	do {
		stat = sdhci_readl(host, W3K_SDHCI_INT_STATUS);
		if (stat & 0x00000100) {
			debug("%s: command done 0x%x\n", __func__, stat);
			break;
		}
		if (stat & 0x00000200) {
			debug("%s(0x%x): command timeout 0x%x\n",
					__func__, sdhci_readl(host, W3K_SDHCI_COMMAND), stat);
			break;
		}
		if (stat & 0x00000400) {
			debug("%s(0x%x): command crc error 0x%x\n",
					__func__, sdhci_readl(host, W3K_SDHCI_COMMAND), stat);
			break;
		}
	} while (--retry != 0);

	if (retry == 0) {
		if (host->quirks & SDHCI_QUIRK_BROKEN_R1B)
			return 0;
		else {
			debug("%s: Timeout for status update!\n", __func__);
			return -ETIMEDOUT;
		}
	}

	if (stat & 0x00000100) {
		sdhci_cmd_done(host, cmd);
	} else
		ret = -1;
	
	if (!ret && data)
		ret = sdhci_transfer_data(host, data, start_addr);
	
#ifndef  W3C_SDHCI_SDMA_ENABLE
	if(adma_table)
		free(adma_table);
#endif
	if (host->quirks & SDHCI_QUIRK_WAIT_SEND_CMD)
		udelay(1000);
	
	stat = sdhci_readl(host, W3K_SDHCI_INT_STATUS);
	sdhci_writel(host, stat, W3K_SDHCI_INT_STATUS);
	debug("write W3K_SDHCI_INT_STATUS(0x%x): 0x%x\n", W3K_SDHCI_INT_STATUS, stat);

#ifdef W3K_SRAM_TO_MEM_FLAG	
	if (!ret && data && data->flags & MMC_DATA_READ){
		debug("read copy 0x%x to 0x%x len:%d\n", (unsigned int )sram_addr,  (unsigned int )data->dest, trans_bytes);
		memcpy(data->dest, (void*)sram_addr, trans_bytes);
	}
	if(!ret && data)
		return 0;
#endif	

#define  W3K_SDHCI_INT_TIMEOUT	0x00000200
#define  W3K_SDHCI_INT_DATA_TIMEOUT	0x00004000
	//w3k_sdhci_reset(host, W3K_SDHCI_RESET_ALL);

	//clear dma interrupt flag
	u32 reg_value = sdhci_readl(host, W3K_SDHCI_DMA_STATUS);
	sdhci_writel(host, reg_value, W3K_SDHCI_DMA_STATUS);

	if (stat & (W3K_SDHCI_INT_TIMEOUT | W3K_SDHCI_INT_DATA_TIMEOUT))
		return -ETIMEDOUT;
	else
		return 0;
}

//#define USE_400K_CLOCK
//#define USE_1M_CLOCK
static int w3k_sdhci_set_clock(struct mmc *mmc, unsigned int clock)
{
	struct sdhci_host *host = mmc->priv;
	unsigned int div, clk;

	debug("%s()\n", __func__);

	if (clock == 0)
		return 0;
#ifdef USE_400K_CLOCK
	if (clock >400000)
		return 0;
#endif
	if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) {
		/* Version 3.00 divisors must be a multiple of 2. */
		if (mmc->cfg->f_max <= clock)
			div = 1;
		else {
			for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
				if ((mmc->cfg->f_max / div) <= clock)
					break;
			}
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((mmc->cfg->f_max / div) <= clock)
				break;
		}
	}
#ifdef USE_1M_CLOCK
	div = 12;
#else
	div >>= 1;
#endif
	clk = sdhci_readl(host, W3K_SDHCI_CLOCK_CONTROL);
	clk &= ~(0x003FFF00);
#define  W3K_SDHCI_DIVIDER_SHIFT	8
#define  W3K_SDHCI_DIV_MASK		0x1FFF
	if (div)
		clk |= ((div - 1) & W3K_SDHCI_DIV_MASK) << W3K_SDHCI_DIVIDER_SHIFT;
	else
		clk |= 0x00200000;

	debug("%s: clock = %d, reg : 0x%x \n", __func__, clock, clk);

	sdhci_writel(host, clk, W3K_SDHCI_CLOCK_CONTROL);
	debug("write W3K_SDHCI_CLOCK_CONTROL(0x%x): 0x%x\n", W3K_SDHCI_CLOCK_CONTROL, clk);

	while (sdhci_readl(host, W3K_SDHCI_CLOCK_CONTROL) & 0x80);

	return 0;
}

static int w3k_sdhci_set_ios(struct udevice *dev)
{
	unsigned int ctrl;
	int count;
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct sdhci_host *host = mmc->priv;

	if (mmc->clock != host->clock){
		w3k_sdhci_set_clock(mmc, mmc->clock);
	}
#define W3K_SDHCI_HOST_CONTROL	0x14
#define  W3K_SDHCI_CTRL_BITBUS_MASK	0x00030000
	/* Set bus width */
	ctrl = sdhci_readl(host, W3K_SDHCI_HOST_CONTROL);
	ctrl &= ~(W3K_SDHCI_CTRL_BITBUS_MASK);
	if (mmc->bus_width == 8) {
		if ((SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) ||
				(host->quirks & SDHCI_QUIRK_USE_WIDE8))
			ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		if (mmc->bus_width == 4)
			ctrl |= SDHCI_CTRL_4BITBUS;
	}
	sdhci_writel(host, ctrl, W3K_SDHCI_HOST_CONTROL);
	debug("write W3K_SDHCI_HOST_CONTROL(0x%x): 0x%x\n", W3K_SDHCI_HOST_CONTROL, ctrl);

	debug("mmc->clock : %d\n", mmc->clock);
	/* HS200/HS400 */
	if (mmc->clock >= 100000000) {
#define W3K_SDHCI_ODL_REG		0x8C
		ctrl = sdhci_readl(host, W3K_SDHCI_ODL_REG);
		ctrl &= ~(0x0000FFFF);
#ifdef HS400
		ctrl |= 0x00001010;
#else	/* HS200 */
		ctrl |= 0x00001C1C;
#endif
		sdhci_writel(host, ctrl, W3K_SDHCI_ODL_REG);
#define W3K_SDHCI_KDL_REG		0x74
		/* enable data out delay, command out delay */
		ctrl = sdhci_readl(host, W3K_SDHCI_KDL_REG);
		ctrl &= ~0x0000000F;
		ctrl |= 0x00000002;
		ctrl |= (3 << 7);
		sdhci_writel(host, ctrl, W3K_SDHCI_KDL_REG);
		while (sdhci_readl(host, W3K_SDHCI_KDL_REG) & 0x00000180);
#define W3K_SDHCI_PCH1_REG		0x5C
		/* Nerc & Necs */
		ctrl = sdhci_readl(host, W3K_SDHCI_PCH1_REG);
		ctrl &= ~0x000000FF;
		ctrl |= 0x55;
		sdhci_writel(host, ctrl, W3K_SDHCI_PCH1_REG);
#define W3K_SDHCI_LAT_REG		0x04
		ctrl = sdhci_readl(host, W3K_SDHCI_LAT_REG);
		ctrl &= ~0x0000007F;
#ifdef HS400
		ctrl |= 0x00000050;
#else	/* HS200 */
		ctrl |= 0x00000000;
#endif
		sdhci_writel(host, ctrl, W3K_SDHCI_LAT_REG);
#define W3K_SDHCI_DLL_REG		0x88
#ifdef HS400
		count = 0;
		ctrl = sdhci_readl(host, W3K_SDHCI_DLL_REG);
		ctrl &= ~0x1;
		sdhci_writel(host, ctrl, W3K_SDHCI_DLL_REG);
		mdelay(1);
		ctrl |= 0x1;
		sdhci_writel(host, ctrl, W3K_SDHCI_DLL_REG);

		while ((sdhci_readl(host, W3K_SDHCI_DLL_REG) & 0x6) == 0x0) {
			count++;
			if (count > 10000) {
				count = 0;
				ctrl = sdhci_readl(host, W3K_SDHCI_DLL_REG);
				ctrl &= ~0x1;
				sdhci_writel(host, ctrl, W3K_SDHCI_DLL_REG);
				mdelay(1);
				ctrl |= 0x1;
				sdhci_writel(host, ctrl, W3K_SDHCI_DLL_REG);
				debug("<error> reset DLL\n");
			}
		}
		debug("W3K_SDHCI_DLL_REG : 0x%x\n", sdhci_readl(host, W3K_SDHCI_DLL_REG));

#else	/* HS200 */
#if 0
#define W3K_SDHCI_ATRG_REG		0x70
		ctrl = sdhci_readl(host, W3K_SDHCI_ATRG_REG);
		ctrl &= ~(0x0000000F);
		ctrl |= 0x00000002;
		sdhci_writel(host, ctrl, W3K_SDHCI_ATRG_REG);

		ctrl = sdhci_readl(host, W3K_SDHCI_LAT_REG);
		ctrl &= ~0x00FFFF7F;
//		ctrl |= ((fdl_n << 16) | (idl_n << 8) |
//			(dlat_sel << 5) |
//			(rlat_sel << 3) | (dlat_neg << 2) |
//			(rlat_neg << 1) | (sel_fck << 0));
		sdhci_writel(host, ctrl, W3K_SDHCI_LAT_REG);

		printk("W3K_SDHCI_LAT_REG : 0x%x\n", sdhci_readl(host, W3K_SDHCI_LAT_REG));

		ctrl = sdhci_readl(host, W3K_SDHCI_KDL_REG);
		ctrl &= ~0x0000000F;
		ctrl |= 0x00000002;
		ctrl |= ((1 << 6) | (1 << 5));
		sdhci_writel(host, ctrl, W3K_SDHCI_KDL_REG);
		while (sdhci_readl(host, W3K_SDHCI_KDL_REG) & 0x0000060);



		ctrl = sdhci_readl(host, W3K_SDHCI_CLOCK_CONTROL);
		debug("read reg - SDHCI_CK_REG 0x%x\n", ctrl);
		ctrl &= ~(0x7 << 22);
#ifdef HS400
		ctrl |= (1 << 22) | (0 << 23) | (1 << 24);
#else	/* HS200 */
		ctrl |= (0 << 22) | (0 << 23) | (0 << 24);
#endif
		sdhci_writel(host, ctrl, W3K_SDHCI_CLOCK_CONTROL);
		debug("write reg - SDHCI_CK_REG 0x%x\n", ctrl);

		while (sdhci_readl(host, W3K_SDHCI_CLOCK_CONTROL) & 0x80);
	}
#endif

#endif
	}
//	if (mmc->clock > 26000000)
//		ctrl |= SDHCI_CTRL_HISPD;
//	else
//		ctrl &= ~SDHCI_CTRL_HISPD;

//	if (host->quirks & SDHCI_QUIRK_NO_HISPD_BIT)
//		ctrl &= ~SDHCI_CTRL_HISPD;

//	sdhci_writeb(host, ctrl, W3K_SDHCI_HOST_CONTROL);
	return 0;
}


static struct dm_mmc_ops w3k_mmc_ops = {
	.send_cmd	= w3k_sdhci_send_command,
	.set_ios	= w3k_sdhci_set_ios,
};

#define W3K_SDHCI_GC_REG 0xffc
static void w3k_sdhci_clk_ctrl(struct sdhci_host *host, int enable)
{
	u32 val = sdhci_readl(host, W3K_SDHCI_GC_REG);
	if(enable == 1)
		val &=~(1<<1);
	else
		val |= (1<<1);
	sdhci_writel(host, val, W3K_SDHCI_GC_REG);
}

static void w3k_sdhci_reset_clk(struct sdhci_host *host)
{
	w3k_sdhci_clk_ctrl(host, 0);
	w3k_sdhci_clk_ctrl(host, 1);
}

static void w3k_sdhci_chip_reset(struct sdhci_host *host)
{
#define 	W3K_SDHCI_VDDP_REG 0x0a0
	u32 count = 10000;
	sdhci_writel(host, 0, W3K_SDHCI_VDDP_REG);
    while (count--){}
	sdhci_writel(host, 1<<1, W3K_SDHCI_VDDP_REG);
}

static void w3k_sdhci_ck_reset(struct sdhci_host *host)
{
#define W3K_SDHCI_CK_REG 0
	sdhci_writel(host, 0, W3K_SDHCI_CK_REG);
	sdhci_writel(host, 1 << 2 | 1 << 3, W3K_SDHCI_CK_REG);
}

static void w3k_sdhci_dma_flush(struct sdhci_host *host)
{

	u32 val = sdhci_readl(host, W3K_SDHCI_DMA_CTL_REG);	
	val &= ~(1<<17);
	val |= (1&1)<<17;
	sdhci_writel(host, val, W3K_SDHCI_DMA_CTL_REG);
	while((sdhci_readl(host, W3K_SDHCI_DMA_CTL_REG) & (1<<17))!=0)
		asm volatile("nop"); 
}

static void w3k_sdhci_hw_reset(struct sdhci_host *host)
{
	w3k_sdhci_reset_clk(host);
	w3k_sdhci_chip_reset(host);
	w3k_sdhci_ck_reset(host);
	w3k_sdhci_dma_flush(host);
}

void w3k_sdhci_host_init(struct sdhci_host *host)
{
	int sck_auto_gate_off = 0, sck_enable = 1, ddr_mode = 0;
	int fck_divider = 30;
	u32 reg_value = 0;
    reg_value |= (!(sck_auto_gate_off & 0x01)) << 1;
    reg_value |= (!(sck_enable & 0x01)) << 4;
    reg_value |= 1 << 3;

    if (fck_divider < 2)
        reg_value |= 1 << 21;
    else if (0 != fck_divider)
        reg_value |= ((fck_divider & 0X1FFF) / 2 - 1) << 8;

    reg_value |= (ddr_mode & 0x01) << 22;
    reg_value |= 1 << 23;  // set dsp bit

	sdhci_writel(host, reg_value, W3K_SDHCI_CLOCK_CONTROL);

	while(sdhci_readl(host, W3K_SDHCI_CLOCK_CONTROL) & (1<<7) )
		 asm volatile("nop");

	reg_value = 3 << 1 | 3 << 28;  // reg init value
	int card_detection_enable = 1, cd_debounce_count = 3;

	reg_value |= card_detection_enable & 0x01;
    reg_value |= (cd_debounce_count & 0xFF) << 8;
#define W3K_SDHCI_PIN_REG 0x008	
	
#define W3K_SDHCI_CFG_REG 0x014
	sdhci_writel(host, reg_value, W3K_SDHCI_PIN_REG);

	//data_timeout_set
	int data_timeout = 1;
	//emmc_read_mod_write(SD_CFG_REG(emmc), 0xFF, timeout, 24);
	reg_value = sdhci_readl(host, W3K_SDHCI_CFG_REG);
	reg_value &= ~(0xff << 24);
	reg_value |= (data_timeout&0xff) << 24;
	sdhci_writel(host, reg_value, W3K_SDHCI_CFG_REG);

	// clear interrupt flag
	reg_value = sdhci_readl(host, W3K_SDHCI_INT_STATUS);
	sdhci_writel(host, reg_value, W3K_SDHCI_INT_STATUS);
	//clear dma interrupt flag
	reg_value = sdhci_readl(host, W3K_SDHCI_DMA_STATUS);
	sdhci_writel(host, reg_value, W3K_SDHCI_DMA_STATUS);

	//gpio mux for emmc1
	writel( 1<<7, (void __iomem *)0x3E208018);
	//sdio enable
	u32 val = sdhci_readl(host, W3K_SDHCI_CFG_REG);
	val &=~(0xf << 19);
	val |= (0xf & 0xf) << 19; 
	sdhci_writel(host, val, W3K_SDHCI_CFG_REG);
}

static int w3k_sdhci_init(struct mmc *mmc)
{
	struct sdhci_host *host = mmc->priv;
	/*hw reset*/
	
	w3k_sdhci_hw_reset(host);
	
	w3k_sdhci_host_init(host);

	if ((host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR) && !aligned_buffer) {
		aligned_buffer = memalign(8, 512*1024);
		if (!aligned_buffer) {
			debug("%s: Aligned buffer alloc failed!!!\n",
			       __func__);
			return -1;
		}
	}

	if (host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR) {
		host->align_buffer = memalign(8, 512 * 1024);
		if (!host->align_buffer) {
			debug("%s: Aligned buffer alloc failed!!!\n",
			       __func__);
			return -ENOMEM;
		}
	}
	return 0;
}



static int __w3k_sdhci_probe(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return w3k_sdhci_init(mmc);
}

static int w3k_sdhci_deferred_probe(struct sdhci_host *host)
{
	struct udevice *dev = host->mmc->dev;
	return __w3k_sdhci_probe(dev);
}

void emmc_fck_divider_set(struct sdhci_host *host, u16 factor)
{
	u32 value = sdhci_readl(host, W3K_SDHCI_CLOCK_CONTROL);

	value &= ~(0X3FFF << 8);

    if (factor < 2) {
        value |= 1 << 21;
    } else {
        value |= ((factor & 0x1FFF) / 2 - 1) << 8;
    }
	sdhci_writel(host, value , W3K_SDHCI_CLOCK_CONTROL);
	while(sdhci_readl(host , W3K_SDHCI_CLOCK_CONTROL) & (1<< 7))
		asm volatile("nop");
	
}

static const struct sdhci_ops w3k_sdhci_ops = {
	.deferred_probe	= w3k_sdhci_deferred_probe,
};


static int w3k_sdhci_probe(struct udevice *dev)
{
	struct w3k_sdhci_plat *plat = dev_get_plat(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct sdhci_host *host = dev_get_priv(dev);
	int ret;
	unsigned int max_clk, min_clk;
	host->name = dev->name;
	host->ioaddr = dev_read_addr_ptr(dev);
	//w3k_mmc_ops = sdhci_ops;

	host->quirks = 0;
	host->version = SDHCI_SPEC_300;
	//debug("%s %d: run\n", __func__, __LINE__);
	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret){
		debug("%s %d:error\n", __func__, __LINE__);	
		return ret;
	}
	host->mmc = &plat->mmc;
	host->mmc->dev = dev;
	min_clk = W3K_MMC_MIN_CLK;
	max_clk = host->max_clk = plat->cfg.f_max;
#if 1
	ret = sdhci_setup_cfg(&plat->cfg, host, host->max_clk, W3K_MMC_MIN_CLK);
	if (ret)
		return ret;
#else 
	host->cfg.name = host->name;

	

//	caps = sdhci_readl(host, SDHCI_CAPABILITIES);
#ifdef CONFIG_MMC_SDMA
//	if (!(caps & SDHCI_CAN_DO_SDMA)) {
//		debug("%s: Your controller doesn't support SDMA!!\n",
//		       __func__);
//		return -1;
//	}
#endif

	if (max_clk)
		host->cfg.f_max = max_clk;
	else {
		if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300)
//			host->cfg.f_max = (caps & SDHCI_CLOCK_V3_BASE_MASK)
//				>> SDHCI_CLOCK_BASE_SHIFT;
			host->cfg.f_max = 200;
		else
//			host->cfg.f_max = (caps & SDHCI_CLOCK_BASE_MASK)
//				>> SDHCI_CLOCK_BASE_SHIFT;
			host->cfg.f_max = 50;
		host->cfg.f_max *= 1000000;
	}
	if (host->cfg.f_max == 0) {
		debug("%s: Hardware doesn't specify base clock frequency\n",
		       __func__);
		return -1;
	}
	if (min_clk)
		host->cfg.f_min = min_clk;
	else {
		if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300)
			host->cfg.f_min = host->cfg.f_max /
				SDHCI_MAX_DIV_SPEC_300;
		else
			host->cfg.f_min = host->cfg.f_max /
				SDHCI_MAX_DIV_SPEC_200;
	}

	host->cfg.voltages = 0;
//	if (caps & SDHCI_CAN_VDD_330)
		host->cfg.voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;
//	if (caps & SDHCI_CAN_VDD_300)
//		host->cfg.voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	//if (caps & SDHCI_CAN_VDD_180)
	//	host->cfg.voltages |= MMC_VDD_165_195;

	if (host->quirks & SDHCI_QUIRK_BROKEN_VOLTAGE)
		host->cfg.voltages |= host->voltages;

	host->cfg.host_caps = MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_4BIT;
	if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) {
//		if (caps & SDHCI_CAN_DO_8BIT)
			host->cfg.host_caps |= MMC_MODE_8BIT;
	}
#endif
	if (host->host_caps)
		host->cfg.host_caps |= host->host_caps;

	host->cfg.b_max = 512;

	//w3k_sdhci_reset(host, W3K_SDHCI_RESET_ALL);


	host->mmc->priv = host;
	upriv->mmc = host->mmc;
	host->cfg.b_max = 512;
	host->ops = &w3k_sdhci_ops;

	ret = __w3k_sdhci_probe(dev);
	if (ret){
		debug("%s %d:error\n", __func__, __LINE__);
		return ret;
	}
	emmc_fck_divider_set(host, 2);
	return ret;
}

static int w3k_sdhci_of_to_plat(struct udevice *dev)
{
	struct sdhci_host *host = dev_get_priv(dev);

	host->name = dev->name;
	host->ioaddr = dev_read_addr_ptr(dev);

	return 0;
}

static int w3k_sdhci_bind(struct udevice *dev)
{
	struct w3k_sdhci_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id w3k_sdhci_ids[] = {
	{ .compatible = "siliconwaves,w3k-sdhci",},
	{ }
};

U_BOOT_DRIVER(w3k_sdhci_drv) = {
	.name		= "w3k_sdhci",
	.id		= UCLASS_MMC,
	.of_match	= w3k_sdhci_ids,
	.of_to_plat = w3k_sdhci_of_to_plat,
	.ops		= &w3k_mmc_ops,
	.bind		= w3k_sdhci_bind,
	.probe		= w3k_sdhci_probe,
	.priv_auto	= sizeof(struct w3k_sdhci_priv),
	.plat_auto	= sizeof(struct w3k_sdhci_plat),
};
