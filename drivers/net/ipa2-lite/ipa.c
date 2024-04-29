// SPDX-License-Identifier: GPL-2.0-only

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_rmnet.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#include "ipa-hw.h"
#include "ipa.h"

#define IPA_FIFO_NUM_DESC		BIT(8)
#define IPA_FIFO_IDX_MASK		(IPA_FIFO_NUM_DESC - 1)
#define IPA_FIFO_SIZE			(IPA_FIFO_NUM_DESC * sizeof(struct fifo_desc))
#define IPA_FIFO_NEXT_IDX(idx)		(((idx) + 1) & IPA_FIFO_IDX_MASK)
#define IPA_NUM_PIPES			(20)
#define IPA_RX_LEN			(2048)
#define IPA_TX_STOP_FREE_THRESH		(0)
#define IPA_PIPE_IRQ_MASK		(P_PRCSD_DESC_EN | P_ERR_EN | P_TRNSFR_END_EN) // same P_DEFAULT_IRQS_EN in bam_dma.c
#define EP_DMA_DIR(ep)			((ep)->is_rx ? DMA_FROM_DEVICE : DMA_TO_DEVICE)

static bool test_mode;
module_param(test_mode, bool, 0644);

static bool dump;
module_param(dump, bool, 0644);

union ipa_cmd {
	struct ipa_hw_imm_cmd_dma_shared_mem dma_smem;
	struct ipa_ip_packet_init ip_pkt_init;
	struct ipa_ip_v4_rule_init rule_v4_init;
	struct ipa_ip_v6_rule_init rule_v6_init;
	struct ipa_hdr_init_local hdr_local_init;
	struct ipa_hdr_init_system hdr_system_init;
};

struct ipa_dma_obj {
	dma_addr_t addr;
	u32 size;
	void *virt;
	struct device *dev;
};

#define PTR_TO_DMA_ADDR(ptr, obj) \
	((obj).addr + (((void *)(ptr)) - (obj).virt))

#define DEF_ACTION(func, arg, ...) \
	static void action_##func(void *ptr) \
	{ \
		arg = ptr; \
		if (ptr) \
			func(__VA_ARGS__); \
	}

struct ipa_ep;

struct ipa_trans {
	struct ipa_ep *ep;
	u32 len; // we need the bam metadata patch to get the rx length
	dma_cookie_t cookie;
	struct sk_buff *skb; // sk_buff used in this trans, NULL for commands (commands do not use this struct anyway)
	dma_addr_t addr;     // dma addr of skb, so we can unmap when finished
};

struct ipa_ep {
	atomic_t free_descs; // named tre_avail in  ipa_dma_trans_info
	u8 allocated_head; // Do we need both head and tail for the allocated transactions?
	u8 allocated_tail;
	u8 pending_head;
	u8 pending_tail;
	struct ipa *ipa;
	struct dma_chan *dma_chan; // bam channel
	struct ipa_trans *trans; // this is a ring buffer with 256 entries // only needed for tx/rx channels
	struct napi_struct *napi;
	u32 has_status	    :1;
	u32 id		    :8;
	u32 is_rx	    :1;
	bool polling;
};

struct ipa {
	atomic_t uc_cmd_busy;
	bool test_mode;
	struct clk *clk;
	struct device *dev;
	struct ipa_ep ep[EP_NUM];
	struct ipa_partition layout[MEM_END + 1];
	struct ipa_qmi *qmi;
	struct net_device *modem, *lan, *loopback;
	struct notifier_block ssr_nb;
	struct wait_queue_head uc_cmd_wq;
	u32 *smem_uc_loaded;
	u32 version, smem_size, smem_restr_bytes;
	void *ssr_cookie;
	void __iomem *ipa_base;
	void __iomem *ipa_sram_base;
};

struct ipa_ndev {
	struct ipa_ep *rx, *tx;
	struct napi_struct napi_rx;
	struct napi_struct napi_tx[];
};

#define FT4_EP0_OFF (2 + 3 * 0)
#define FT4_EP4_OFF (2 + 3 * 1)
#define RT4_EP0_OFF (2 + 3 * 2)
#define RT4_EP4_OFF (2 + 3 * 3)

static const u32 ipa_rules[] = {
	/* Default (zero) rules */
	0, 0,
	/* Rules for loopback */
	/* EP0 filter: dummy range16, routing index 1 */
	[FT4_EP0_OFF] = BIT(4) | (1 << 21),
	[FT4_EP0_OFF + 1] = 0xffff00,
	/* EP4 filter: dummy range16, routing index 2 */
	[FT4_EP4_OFF] = BIT(4) | (2 << 21),
	[FT4_EP4_OFF + 1] = 0xffff00,
	/* EP0 route: dummy range16, dest pipe 5, system hdr */
	[RT4_EP0_OFF] = BIT(21) | BIT(4) | (5 << 16),
	[RT4_EP0_OFF + 1] = 0xffff00,
	/* EP4 route: dummy range16, dest pipe 1, system hdr */
	[RT4_EP4_OFF] = BIT(21) | BIT(4) | (1 << 16),
	[RT4_EP4_OFF + 1] = 0xffff00,
	[RT4_EP4_OFF + 2] = 0,
};

static inline void rmw32(void __iomem *reg, u32 mask, u32 val)
{
	iowrite32((ioread32(reg) & ~mask) | (val & mask), reg);
}

static void ipa_dma_free(struct ipa_dma_obj *obj)
{
	if (obj->size)
		dma_free_coherent(obj->dev, obj->size, obj->virt, obj->addr);
	obj->size = 0;
}

//DEF_ACTION(ipa_dma_free, struct ipa_dma_obj *obj, obj);

static int ipa_dma_alloc(struct ipa *ipa, struct ipa_dma_obj *obj, u32 size)
{
	if (WARN_ON(!size))
		return -EINVAL;

	obj->virt = dma_alloc_coherent(ipa->dev, size, &obj->addr, GFP_KERNEL);
	if (!obj->virt)
		return -ENOMEM;

	obj->size = size;
	obj->dev = ipa->dev;
	return 0;
}

static void ipa_reset_hw(struct ipa *ipa)
{
	iowrite32(1, ipa->ipa_base + REG_IPA_COMP_SW_RESET_OFST);
	iowrite32(0, ipa->ipa_base + REG_IPA_COMP_SW_RESET_OFST);
	iowrite32(1, ipa->ipa_base + REG_IPA_COMP_CFG_OFST);
	if (ipa->version >= 25) // We do not have versions < 25 here, so far.
		iowrite32(0x1fff7f, ipa->ipa_base + REG_IPA_BCR_OFST);
}

static const char *ipa_ep_dma_chan_name[] = {"test_tx",
					     "test_rx",
					     "ap_lan_rx",
					     "cmd_tx",
					     "ap_modem_tx",
					     "ap_modem_rx"
};

static int ipa_setup_ep(struct ipa *ipa, enum ipa_ep_id id)
{
	struct ipa_ep *ep = &ipa->ep[id];
	struct dma_slave_config bam_config;

	// TODO: We only need a trans array if we have a dma_chan
	ep->trans = (struct ipa_trans *) kzalloc(IPA_FIFO_NUM_DESC * sizeof(struct ipa_trans), GFP_KERNEL);
	if (!ep->trans)
		return -ENOMEM;

	atomic_set(&ep->free_descs, IPA_FIFO_NUM_DESC);
	ep->allocated_head = 0;
	ep->allocated_tail = 0;
	ep->pending_head = 0;
	ep->pending_tail = 0;
	ep->id = id;
	ep->ipa = ipa;
	ep->polling = false;
	if(ipa_ep_dma_chan_name[id]) {
		ep->dma_chan = dma_request_chan(ipa->dev, ipa_ep_dma_chan_name[id]); // TODO: define names
		if (IS_ERR(ep->dma_chan)) {
			printk(KERN_INFO "%s: failed to request dma_chan for %s with error %d\n", __func__, ipa_ep_dma_chan_name[id], (int) PTR_ERR(ep->dma_chan));
			kfree(ep->trans);
			return (int) PTR_ERR(ep->dma_chan);
		}
	}
	ep->is_rx = EP_ID_IS_RX(id);

	iowrite32(ep->id != EP_CMD, ipa->ipa_base + REG_IPA_EP_CTRL(id));
	iowrite32(ep->is_rx ? 1 : 0, ipa->ipa_base + REG_IPA_EP_HOL_BLOCK_EN(id));

	bam_config.direction = ep->is_rx ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV;
	if (ep->id == EP_CMD)
		bam_config.dst_maxburst = 20;
	else if (ep->is_rx)
		bam_config.src_maxburst = 8;
	else
		bam_config.dst_maxburst = 8;

	if (ep->dma_chan) {
		dmaengine_slave_config(ep->dma_chan, &bam_config);
	}

	switch (id) {
	case EP_LAN_RX:
		ep->has_status = 1;
		iowrite32(0x00000002, ipa->ipa_base + REG_IPA_EP_HDR(id));
		iowrite32(0x00000803, ipa->ipa_base + REG_IPA_EP_HDR_EXT(id));
		iowrite32(0x00000000, ipa->ipa_base + REG_IPA_EP_HDR_METADATA_MASK(id));
		iowrite32(0x00000001, ipa->ipa_base + REG_IPA_EP_STATUS(id));
		break;
	case EP_TX:
	case EP_TEST_TX:
		iowrite32(ipa->test_mode ? 0xc4 : 0x44, ipa->ipa_base + REG_IPA_EP_HDR(id));
		iowrite32(0x00000001, ipa->ipa_base + REG_IPA_EP_HDR_EXT(id));
		iowrite32(0x00000005, ipa->ipa_base + REG_IPA_EP_STATUS(id));
		iowrite32(0x00000007, ipa->ipa_base + REG_IPA_EP_ROUTE(id));
		iowrite32(0x00000020, ipa->ipa_base + REG_IPA_EP_MODE(id));
		break;
	case EP_RX:
	case EP_TEST_RX:
		iowrite32(0x002800c4, ipa->ipa_base + REG_IPA_EP_HDR(id));
		iowrite32(0x0000000b, ipa->ipa_base + REG_IPA_EP_HDR_EXT(id));
		iowrite32(0xff000000, ipa->ipa_base + REG_IPA_EP_HDR_METADATA_MASK(id));
	default:
		break;
	}

	return 0;
}

static int ipa_uc_send_cmd(struct ipa *ipa, u8 cmd_op, u32 cmd_param, u32 resp_status)
{
	unsigned long timeout = msecs_to_jiffies(1000);
	int val, ret;

	ret = wait_event_timeout(ipa->uc_cmd_wq,
			!(atomic_fetch_or(1, &ipa->uc_cmd_busy) & 1), timeout);
	if (ret <= 0)
		return -ETIMEDOUT;

	timeout = ret;

	iowrite32(cmd_op, ipa->ipa_sram_base + REG_IPA_UC_CMD);
	iowrite32(cmd_param, ipa->ipa_sram_base + REG_IPA_UC_CMD_PARAM);
	iowrite32(0, ipa->ipa_sram_base + REG_IPA_UC_RESP);
	iowrite32(0, ipa->ipa_sram_base + REG_IPA_UC_RESP_PARAM);

	iowrite32(1, ipa->ipa_base + REG_IPA_IRQ_UC_EE0);

	ret = wait_event_timeout(ipa->uc_cmd_wq,
		(val = FIELD_GET(IPA_UC_RESP_OP_MASK, ioread32(ipa->ipa_sram_base + REG_IPA_UC_RESP))) ==
		IPA_UC_RESPONSE_CMD_COMPLETED,
		timeout);

	atomic_set(&ipa->uc_cmd_busy, 0);
	wake_up_all(&ipa->uc_cmd_wq);

	if (val != IPA_UC_RESPONSE_CMD_COMPLETED)
		return -ETIMEDOUT;

	val = FIELD_GET(IPA_UC_RESP_OP_PARAM_STATUS_MASK,
			ioread32(ipa->ipa_sram_base + REG_IPA_UC_RESP_PARAM));
	if (val != resp_status) {
		dev_err(ipa->dev, "cmd %d returned unexpected status: %d\n",
			cmd_op, val);
		return -EINVAL;
	}

	return 0;
}

static void ipa_reset_modem_pipes(struct ipa *ipa)
{
	/* For 2.5+ */
	u8 pipes[] = { 6, 7, 11, 13, 8, 9, 12, 14 };
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(pipes); i++) {
		ret = ipa_uc_send_cmd(ipa, IPA_UC_CMD_RESET_PIPE,
				      IPA_UC_CMD_RESET_PIPE_PARAM(pipes[i], i >= 4), 0);
		if (ret)
			dev_err(ipa->dev, "failed to reset %d pipe: %d\n",
				pipes[i], ret);
	}
}

static void ipa_partition_put(struct ipa *ipa, u32 *offset,
			      enum ipa_part_id id, u32 size_words, u32 align_words)
{
	u32 __iomem *ptr = ipa->ipa_sram_base +
		(ipa->version < 25 ? ipa->smem_restr_bytes : 0) + *offset;
	bool first_canary = true;
	u32 canary = 0xdeadbeaf;

	if (id == MEM_DRV) {
		/* Keep uc_loaded status in SRAM and don't override it */
		ipa->smem_uc_loaded = ptr;
		if (*ptr == 0x10ADEDFF)
			canary = 0x10ADEDFF;
	}

	while ((first_canary || ALIGN(*offset, 4 * align_words) != *offset) &&
	       *offset < ipa->smem_size) {
		*(ptr++) = canary;
		*offset += 4;
		first_canary = false;
	}

	ipa->layout[id].offset = *offset + ipa->smem_restr_bytes;
	ipa->layout[id].size = size_words * 4;

	*offset = *offset + size_words * 4;
}

static int ipa_partition_mem(struct ipa *ipa)
{
	u32 offset, val;

	val = ioread32(ipa->ipa_base + REG_IPA_SHARED_MEM);

	ipa->smem_restr_bytes = FIELD_GET(IPA_SHARED_MEM_BADDR_BMSK, val);
	ipa->smem_size = FIELD_GET(IPA_SHARED_MEM_SIZE_BMSK, val);

	if (WARN_ON(ipa->smem_restr_bytes > ipa->smem_size ||
		    (ipa->smem_restr_bytes & 3) || ipa->smem_size & 3))
		return -EINVAL;

	ipa->smem_size -= ipa->smem_restr_bytes;
	offset = 0x280;

	ipa_partition_put(ipa, &offset, MEM_FT_V4, IPA_NUM_PIPES + 2, 2);
	ipa_partition_put(ipa, &offset, MEM_FT_V6, IPA_NUM_PIPES + 2, 2);
	ipa_partition_put(ipa, &offset, MEM_RT_V4, 7, 2);
	ipa_partition_put(ipa, &offset, MEM_RT_V6, 7, 2);
	ipa_partition_put(ipa, &offset, MEM_MDM_HDR, 80, 2);
	ipa_partition_put(ipa, &offset, MEM_DRV, sizeof(ipa_rules) / 4, 1);

	if (ipa->version == 25)
		ipa_partition_put(ipa, &offset, MEM_MDM_HDR_PCTX, 128, 2);
	else if (ipa->version == 26)
		ipa_partition_put(ipa, &offset, MEM_MDM_COMP, 128, 2);

	ipa_partition_put(ipa, &offset, MEM_MDM,
			  (ipa->smem_size - offset) / 4 - 2, 1);
	ipa_partition_put(ipa, &offset, MEM_END, 0, 2);

	return 0;
}

static void ipa_cmd_callback(void *arg) {
	complete(arg);
}

static int ipa_init_sram_part(struct ipa *ipa, enum ipa_part_id mem_id)
{
	u32 part_offset, payload_addr, *payload, *end, val;
	struct ipa_partition *part = ipa->layout + mem_id;
	struct ipa_ep *ep = ipa->ep + EP_CMD;
	struct ipa_dma_obj pld, cmds;
	int ret;
	DECLARE_COMPLETION_ONSTACK(completion);
	struct dma_async_tx_descriptor *desc;
	union ipa_cmd *cmd;

	if (!part->size)
		return 0;

	ret = ipa_dma_alloc(ipa, &cmds, sizeof(*cmd) * 2 + 4);
	if (ret)
		return ret;

	ret = ipa_dma_alloc(ipa, &pld, part->size);
	if (ret)
		goto free_cmd_args;

	part_offset = part->offset;
	payload_addr = pld.addr;
	payload = pld.virt;

	switch (mem_id) {
	case MEM_DRV:
		memcpy(payload, ipa_rules, sizeof(ipa_rules));
		break;
	case MEM_FT_V4:
	case MEM_FT_V6:
		*(payload++) = 0x1fffff;
		fallthrough;
	case MEM_RT_V4:
	case MEM_RT_V6:
		end = pld.virt + pld.size;
		val = ipa->layout[MEM_DRV].offset - part_offset;

		while (payload <= end)
			*(payload++) = val | 1;
	default:
		break;
	}

	if (ipa->test_mode) {
		payload = pld.virt;
		val = ipa->layout[MEM_DRV].offset - part_offset + 1;
		if (mem_id == MEM_FT_V4) {
			payload[2 + 0] = val + FT4_EP0_OFF * 4;
			payload[2 + 4] = val + FT4_EP4_OFF * 4;
		} else if (mem_id == MEM_RT_V4) {
			payload[1] = val + RT4_EP0_OFF * 4;
			payload[2] = val + RT4_EP4_OFF * 4;
		}
	}

	cmd = cmds.virt;

	switch (mem_id) {
	case MEM_MDM_HDR:
		cmd->hdr_local_init.hdr_table_src_addr = payload_addr;
		cmd->hdr_local_init.size_hdr_table = part->size;
		cmd->hdr_local_init.hdr_table_dst_addr = part_offset;
		desc = dmaengine_prep_slave_single(ep->dma_chan, PTR_TO_DMA_ADDR(cmd, cmds),
				IPA_CMD_HDR_LOCAL_INIT, DMA_MEM_TO_DEV,
				DMA_PREP_IMM_CMD | DMA_PREP_INTERRUPT);
		desc->cookie = dmaengine_submit(desc);
		cmd++;
		fallthrough;
	case MEM_MDM_COMP:
	case MEM_MDM:
	case MEM_DRV:
		desc = dmaengine_prep_slave_single(ep->dma_chan, PTR_TO_DMA_ADDR(cmd, cmds),
				IPA_CMD_DMA_SHARED_MEM, DMA_MEM_TO_DEV,
				DMA_PREP_IMM_CMD | DMA_PREP_INTERRUPT);
		cmd->dma_smem.system_addr = payload_addr;
		cmd->dma_smem.local_addr = part_offset;
		cmd->dma_smem.size = part->size;
		desc->callback = ipa_cmd_callback;
		desc->callback_param = &completion;
		desc->cookie = dmaengine_submit(desc);
		break;
	case MEM_RT_V4:
	case MEM_FT_V4:
		if (mem_id == MEM_RT_V4) {
			desc = dmaengine_prep_slave_single(ep->dma_chan, PTR_TO_DMA_ADDR(cmd, cmds),
					IPA_CMD_RT_V4_INIT, DMA_MEM_TO_DEV,
					DMA_PREP_IMM_CMD | DMA_PREP_INTERRUPT);
		} else {
			desc = dmaengine_prep_slave_single(ep->dma_chan, PTR_TO_DMA_ADDR(cmd, cmds),
					IPA_CMD_FT_V4_INIT, DMA_MEM_TO_DEV,
					DMA_PREP_IMM_CMD | DMA_PREP_INTERRUPT);
		}
		cmd->rule_v4_init.ipv4_addr = part_offset;
		cmd->rule_v4_init.size_ipv4_rules = part->size;
		cmd->rule_v4_init.ipv4_rules_addr = payload_addr;
		desc->callback = ipa_cmd_callback;
		desc->callback_param = &completion;
		desc->cookie = dmaengine_submit(desc);
		break;
	case MEM_RT_V6:
	case MEM_FT_V6:
		if (mem_id == MEM_RT_V6) {
			desc = dmaengine_prep_slave_single(ep->dma_chan, PTR_TO_DMA_ADDR(cmd, cmds),
					IPA_CMD_RT_V6_INIT, DMA_MEM_TO_DEV,
					DMA_PREP_IMM_CMD | DMA_PREP_INTERRUPT);
		} else {
			desc = dmaengine_prep_slave_single(ep->dma_chan, PTR_TO_DMA_ADDR(cmd, cmds),
					IPA_CMD_FT_V6_INIT, DMA_MEM_TO_DEV,
					DMA_PREP_IMM_CMD | DMA_PREP_INTERRUPT);
		}
		cmd->rule_v6_init.ipv6_addr = part_offset;
		cmd->rule_v6_init.size_ipv6_rules = part->size;
		cmd->rule_v6_init.ipv6_rules_addr = payload_addr;
		desc->callback = ipa_cmd_callback;
		desc->callback_param = &completion;
		desc->cookie = dmaengine_submit(desc);
		break;
	default:
		WARN_ON(1);
		ret = -EINVAL;
		goto free_pld;
	}

	dma_async_issue_pending(ep->dma_chan);

	if (!wait_for_completion_timeout(&completion, msecs_to_jiffies(1000)))
		ret = -ETIMEDOUT;
free_pld:
	ipa_dma_free(&pld);

free_cmd_args:
	ipa_dma_free(&cmds);

	return ret;
}

static int ipa_init_sram(struct ipa *ipa)
{
	enum ipa_part_id part;
	int ret;

	for (part = 0; part < MEM_END; part++) {
		ret = ipa_init_sram_part(ipa, part);
		if (ret)
			return ret;
	}

	return 0;
}

static void ipa_unmap_skbs(struct ipa_ep *ep)
{
	struct ipa_trans *trans;
	u8 index;

	if (atomic_read(&ep->free_descs) == IPA_FIFO_NUM_DESC)
		return;

	index = ep->pending_head;

	do {
		trans = &ep->trans[index];

		dma_unmap_single(ep->ipa->dev, trans->addr, ep->is_rx ? IPA_RX_LEN : trans->skb->len, EP_DMA_DIR(ep));

		if (trans->skb)
			dev_kfree_skb_any(trans->skb);

		index++;
	} while (index != ep->pending_tail);
}

static int ipa_ssr_notifier(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	struct ipa *ipa = container_of(nb, struct ipa, ssr_nb);

	if (action == QCOM_SSR_BEFORE_SHUTDOWN) {
		ipa_modem_set_present(ipa->dev, false);
	} else if (action == QCOM_SSR_AFTER_SHUTDOWN) {
		ipa_reset_modem_pipes(ipa);
		ipa_init_sram(ipa);
	} else {
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static irqreturn_t ipa_isr_thread(int irq, void *data)
{
	struct ipa *ipa = data;
	u32 val;

	val = ioread32(ipa->ipa_base + REG_IPA_IRQ_STTS_EE0);
	iowrite32(val, ipa->ipa_base + REG_IPA_IRQ_CLR_EE0);

	if (val & BIT(IPA_IRQ_UC_IRQ_1)) {
		val = ioread32(ipa->ipa_sram_base + REG_IPA_UC_RESP);
		val &= IPA_UC_RESP_OP_MASK;
		if (ipa->qmi && val == IPA_UC_RESPONSE_INIT_COMPLETED) {
			ipa_qmi_uc_loaded(ipa->qmi);
			ipa->smem_uc_loaded[0] = 0x10ADEDFF;
		} else if (ipa->qmi && val == IPA_UC_RESPONSE_CMD_COMPLETED) {
			wake_up_all(&ipa->uc_cmd_wq);
		}
	}

	return IRQ_HANDLED;
}

static void ipa_napi_callback(void *arg)
{
	struct ipa_ep *ep = (struct ipa_ep *) arg;

	if (!ep->polling) {
		// This way further callbacks do no call napi_schedule.
		ep->polling = true;
		napi_schedule(ep->napi); // return value is ignored
	}
}

static struct ipa_trans *ipa_allocate_trans_new(struct ipa_ep *ep)
{
	struct ipa_trans *trans;

	if (atomic_sub_return(1, &ep->free_descs) < 0) {
		atomic_inc(&ep->free_descs);
		return NULL;
	}

	trans = &ep->trans[ep->allocated_tail];
	memset(trans, 0, sizeof(*trans));
	trans->ep = ep;
	ep->allocated_tail++;
	return trans;
}


static void ipa_trans_free_unused(struct ipa_trans *trans)
{
	struct ipa_ep *ep = trans->ep;
	ep->allocated_tail--;
	atomic_inc(&ep->free_descs);
}

static void ipa_trans_free(struct ipa_trans *trans)
{
	struct ipa_ep *ep = trans->ep;
	ep->allocated_head++;
	atomic_inc(&ep->free_descs);
}

static struct ipa_trans *bam_channel_poll_one_new(struct ipa_ep *ep)
{
	struct ipa_trans *trans;
	enum dma_status trans_status;

	if (atomic_read(&ep->free_descs) < IPA_FIFO_NUM_DESC) {
		trans = &ep->trans[ep->pending_head];
		trans_status = dma_async_is_tx_complete(ep->dma_chan, trans->cookie, NULL, NULL);
		if (trans_status == DMA_COMPLETE) {
			ep->pending_head++;
			return trans;
		}
	}

	return NULL;
}

static int ipa_poll_tx(struct napi_struct *napi, int budget)
{
	struct ipa_ep *ep = container_of(napi, struct ipa_ndev, napi_tx[0])->tx;
	struct net_device *ndev = napi->dev;
	struct ipa *ipa = ep->ipa;
	struct device *dev = ipa->dev;
	struct sk_buff *skb;
	u32 packets = 0, bytes = 0;
	int done = 0;

	while (done < budget) {
		struct ipa_trans *trans;
		done++;
		trans = bam_channel_poll_one_new(ep);
		if (!trans)
			break;

		skb = trans->skb;
		BUG_ON(!skb);

		bytes += skb->len;
		packets++;

		dma_unmap_single(dev, trans->addr, skb->len, DMA_TO_DEVICE);
		dev_consume_skb_any(skb);

		ipa_trans_free(trans);
	}

	if (netif_queue_stopped(ndev) &&
	    atomic_read(&ep->free_descs) > IPA_TX_STOP_FREE_THRESH)
		netif_wake_queue(ndev);

	ndev->stats.tx_bytes += bytes;
	ndev->stats.tx_packets += packets;

	if (budget && done < budget && napi_complete_done(napi, done))
		ep->polling = false;

	return done;
}

static int ipa_poll_rx(struct napi_struct *napi, int budget)
{
	struct ipa_ep *ep = container_of(napi, struct ipa_ndev, napi_rx)->rx;
	struct net_device *ndev = napi->dev;
	struct ipa *ipa = ep->ipa;
	struct device *dev = ipa->dev;
	struct sk_buff *skb, *new_skb;
	u32 packets = 0, bytes = 0;
	dma_addr_t addr;
	int done = 0;
	struct dma_async_tx_descriptor *desc;

	while (done < budget) {
		struct ipa_trans *trans;

		trans = bam_channel_poll_one_new(ep);
		if (!trans)
			break;

		skb = trans->skb;

		new_skb = netdev_alloc_skb(ndev, IPA_RX_LEN);
		if (unlikely(!new_skb))
			goto skip_rx;

		addr = dma_map_single(dev, new_skb->data, IPA_RX_LEN,
				      DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(ipa->dev, addr))) {
			dev_kfree_skb_any(new_skb);
			goto skip_rx;
		}

		skb_put(skb, trans->len);
		skb->dev = ndev;
		skb->protocol = htons(ETH_P_MAP);

		dma_unmap_single(dev, trans->addr, IPA_RX_LEN, DMA_FROM_DEVICE);

		if (unlikely(dump)) {
			char prefix[8] = "RX EP  ";

			prefix[5] = '0' + ep->id;
			print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, skb->len, true);
		}

		if (likely(!ep->has_status)) {
			packets++;
			bytes += skb->len;
			netif_receive_skb(skb);
		} else {
			dev_kfree_skb_any(skb);
		}

		ipa_trans_free(trans);
		trans = NULL;

		// We need a new trans here!
		trans = ipa_allocate_trans_new(ep);
		if (!trans) {
			dev_kfree_skb_any(new_skb);
			dev_err(dev, "ipa_allocate_trans_new failed in %s\n", __func__);
			goto skip_rx;
		}

		skb = new_skb;
		desc = dmaengine_prep_slave_single(ep->dma_chan, addr, IPA_RX_LEN, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
		desc->cookie = dmaengine_submit(desc);
		trans->cookie = desc->cookie;
		trans->skb = skb;
		trans->addr = addr;
		dmaengine_desc_attach_metadata(desc, &trans->len, sizeof(trans->len));
		desc->callback = ipa_napi_callback;
		desc->callback_param = ep;
		ep->pending_tail++;
		dma_async_issue_pending(ep->dma_chan);
skip_rx:
		done++;
	}

	ndev->stats.rx_bytes += bytes;
	ndev->stats.rx_packets += packets;
	ndev->stats.rx_dropped += done - packets;

	if (budget && done < budget && napi_complete_done(napi, done))
		ep->polling = false;

	return done;
}

static int ipa_enqueue_skb(struct sk_buff *skb, struct net_device *ndev, struct ipa_ep *ep)
{
	struct device *dev = ep->ipa->dev;
	struct dma_async_tx_descriptor *desc;
	int ret;
	struct ipa_trans *trans = NULL;
	u32 len;
	enum dma_transfer_direction direction;

	len = ep->is_rx ? IPA_RX_LEN : skb->len;

	trans = ipa_allocate_trans_new(ep);
	if (WARN_ON(!trans))
		return -EBUSY;

	if (ep->is_rx) {
		WARN_ON(skb);
		skb = netdev_alloc_skb(ndev, len);
		if (!skb)
			goto release_desc;
	} else if (unlikely(dump)) {
		char prefix[8] = "TX EP  ";

		prefix[5] = '0' + ep->id;
		print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_OFFSET,
			       16, 1, skb->data, len, true);
	}

	dma_addr_t addr = dma_map_single(dev, skb->data, len, EP_DMA_DIR(ep));

	if (dma_mapping_error(dev, addr)) {
		printk(KERN_INFO "%s %d: dma mapping error\n", __func__, __LINE__);
		goto free_skb;
	}

	direction = ep->is_rx ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV;
	desc = dmaengine_prep_slave_single(ep->dma_chan, addr, len, direction, DMA_PREP_INTERRUPT);

	desc->callback = ipa_napi_callback;
	desc->callback_param = ep;

	desc->cookie = dmaengine_submit(desc);
	trans->cookie = desc->cookie;
	trans->skb = skb;
	trans->addr = addr;

	if (ep->is_rx)
		dmaengine_desc_attach_metadata(desc, &trans->len, sizeof(trans->len));

	ep->pending_tail++;

	dma_async_issue_pending(ep->dma_chan);

	return atomic_read(&ep->free_descs); // Is this ever used?

free_skb:
	dev_kfree_skb_any(skb);

release_desc:
	ipa_trans_free_unused(trans);

	return ret;
}

static int ipa_ndev_open(struct net_device *ndev)
{
	struct ipa_ndev *ipa_ndev = netdev_priv(ndev);
	struct ipa *ipa = ipa_ndev->rx->ipa;
	int ret = 0;

	pm_runtime_get_sync(ipa->dev);

	while (atomic_read(&ipa_ndev->rx->free_descs) > 0) { // If we enqueue 256 skbs, do we get into trouble with our u8 index?
		ret = ipa_enqueue_skb(NULL, ndev, ipa_ndev->rx);
		if (WARN_ON(ret < 0))
			goto fail;
	}

	napi_enable(ipa_ndev->rx->napi);
	iowrite32(0, ipa->ipa_base + REG_IPA_EP_HOL_BLOCK_EN(ipa_ndev->rx->id));
	iowrite32(0, ipa->ipa_base + REG_IPA_EP_CTRL(ipa_ndev->rx->id));

	if (ipa_ndev->tx) {
		iowrite32(0, ipa->ipa_base + REG_IPA_EP_CTRL(ipa_ndev->tx->id));
		napi_enable(ipa_ndev->tx->napi);
	}

	netif_start_queue(ndev);

	return 0;

fail:
	ipa_unmap_skbs(ipa_ndev->rx);
	pm_runtime_put(ipa->dev);
	return ret;
}

static int ipa_ndev_stop(struct net_device *ndev)
{
	struct ipa_ndev *ipa_ndev = netdev_priv(ndev);
	struct ipa *ipa = ipa_ndev->rx->ipa;

	netif_stop_queue(ndev);

	napi_disable(ipa_ndev->rx->napi);
	dmaengine_terminate_sync(ipa_ndev->rx->dma_chan);
	ipa_unmap_skbs(ipa_ndev->rx);
	if (ipa_ndev->tx) {
		napi_disable(ipa_ndev->tx->napi);
		dmaengine_terminate_sync(ipa_ndev->tx->dma_chan);
		ipa_unmap_skbs(ipa_ndev->tx);
	}

	pm_runtime_put(ipa->dev);

	return 0;
}

static void ipa_ndev_suspend_resume(struct net_device *ndev, bool resume)
{
	if (!ndev || !netif_running(ndev))
		return;

	if (resume)
		ipa_ndev_open(ndev);
	else
		ipa_ndev_stop(ndev);
}

static netdev_tx_t ipa_ndev_start_xmit(struct sk_buff *skb,
				       struct net_device *ndev)
{
	struct ipa_ndev *ipa_ndev = netdev_priv(ndev);
	int ret;

	if (skb->protocol != htons(ETH_P_MAP) || skb_linearize(skb) || !ipa_ndev->tx)
		goto drop_tx;

	ret = ipa_enqueue_skb(skb, ndev, ipa_ndev->tx);
	if (ret == -EBUSY)
		return NETDEV_TX_BUSY;
	else if (ret < 0)
		goto drop_tx;
	else if (ret <= IPA_TX_STOP_FREE_THRESH)
		netif_stop_queue(ndev);

	return NETDEV_TX_OK;

drop_tx:
	ndev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static const struct net_device_ops ipa_ndev_ops = {
	.ndo_open = ipa_ndev_open,
	.ndo_stop = ipa_ndev_stop,
	.ndo_start_xmit = ipa_ndev_start_xmit,
};

static void ipa_ndev_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &ipa_ndev_ops;
	ndev->addr_len = 0;
	ndev->hard_header_len = 0;
	ndev->min_header_len = ETH_HLEN;
	ndev->needed_headroom = 4; /* QMAP_HDR */
	ndev->mtu = IPA_RX_LEN - 32 - 4; /* STATUS + QMAP_HDR */
	ndev->max_mtu = ndev->mtu;
	ndev->needed_tailroom = 0;
	ndev->priv_flags |= IFF_TX_SKB_SHARING;
	ndev->tx_queue_len = 1000;
	ndev->type = ARPHRD_RAWIP;
	ndev->watchdog_timeo = 1000;
	eth_broadcast_addr(ndev->broadcast);
}

DEF_ACTION(qcom_unregister_ssr_notifier, struct ipa *ipa,
	   ipa->ssr_cookie, &ipa->ssr_nb);
DEF_ACTION(ipa_qmi_teardown, struct ipa *ipa, ipa->qmi);

static void ipa_remove_netdev(void *data)
{
	struct net_device *ndev = data;
	struct ipa_ndev *ipa_ndev = netdev_priv(ndev);

	netif_napi_del(ipa_ndev->rx->napi);
	if (ipa_ndev->tx)
		netif_napi_del(ipa_ndev->tx->napi);
	unregister_netdev(ndev);
	free_netdev(ndev);
}

static struct net_device *
ipa_create_netdev(struct device *dev, const char *name,
			     struct ipa_ep *rx, struct ipa_ep *tx)
{
	struct ipa_ndev *ipa_ndev;
	struct net_device *ndev;
	int ret;

	ndev = alloc_netdev(struct_size(ipa_ndev, napi_tx, !!tx),
			    name, NET_NAME_UNKNOWN, ipa_ndev_setup);
	if (IS_ERR_OR_NULL(ndev))
		return ERR_PTR(-ENOMEM);

	if (rx->id == EP_RX)
		SET_NETDEV_DEV(ndev, dev);
	ipa_ndev = netdev_priv(ndev);
	ipa_ndev->rx = rx;
	ipa_ndev->tx = tx;
	rx->napi = &ipa_ndev->napi_rx;

	netif_napi_add(ndev, rx->napi, ipa_poll_rx);
	if (tx) {
		tx->napi = &ipa_ndev->napi_tx[0];
		netif_napi_add_tx(ndev, tx->napi, ipa_poll_tx);
	}

	ret = register_netdev(ndev);
	if (ret) {
		netif_napi_del(rx->napi);
		if (tx)
			netif_napi_del(tx->napi);
		free_netdev(ndev);
		return ERR_PTR(ret);
	}

	ret = devm_add_action_or_reset(dev, ipa_remove_netdev, ndev);
	if (ret)
		return ERR_PTR(ret);

	return ndev;
}

void ipa_modem_set_present(struct device *dev, bool present)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	(present ? netif_device_attach : netif_device_detach) (ipa->modem);
}

static int ipa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *name = "rmnet_ipa0";
	struct ipa *ipa;
	int ep, ret;

	ipa = devm_kzalloc(dev, sizeof(*ipa), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ipa))
		return -ENOMEM;

	ipa->version = (long)of_device_get_match_data(dev);
	ipa->dev = dev;
	ipa->ssr_nb.notifier_call = ipa_ssr_notifier;
	ipa->test_mode = test_mode;

	atomic_set(&ipa->uc_cmd_busy, 0);
	init_waitqueue_head(&ipa->uc_cmd_wq);
	platform_set_drvdata(pdev, ipa);

	ipa->ipa_base = devm_platform_ioremap_resource_byname(pdev, "ipa-reg");
	if (IS_ERR(ipa->ipa_base))
		return PTR_ERR(ipa->ipa_base);

	ipa->ipa_sram_base = devm_platform_ioremap_resource_byname(pdev, "ipa-shared");
	if (IS_ERR(ipa->ipa_sram_base))
		return PTR_ERR(ipa->ipa_sram_base);

	ipa->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(ipa->clk))
		return dev_err_probe(dev, PTR_ERR(ipa->clk),
				     "failed to get clock\n");

	clk_set_rate(ipa->clk, 40000000);

	ipa_reset_hw(ipa);

	ret = ipa_partition_mem(ipa);
	if (ret)
		return ret;

	for (ep = 0; ep < EP_NUM; ep++) {
		ret = ipa_setup_ep(ipa, ep);
		if (ret)
			return ret;
	}

	iowrite32(0x00040044, ipa->ipa_base + REG_IPA_ROUTE_OFST);

	ret = dma_set_mask_and_coherent(ipa->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(ipa->dev, "error %d setting DMA mask\n", ret);
		return ret;
	}

	ret = ipa_init_sram(ipa);
	if (ret)
		return ret;

	rmw32(ipa->ipa_base + REG_IPA_IRQ_EN_EE0, BIT(IPA_IRQ_UC_IRQ_1), BIT(IPA_IRQ_UC_IRQ_1));

	ret = of_irq_get_byname(dev->of_node, "ipa");
	if (ret < 0)
		return ret;

	ret = devm_request_threaded_irq(dev, ret, NULL, ipa_isr_thread,
					IRQF_ONESHOT, "ipa", ipa);
	if (ret)
		return ret;

	if (ipa->test_mode)
		goto skip_modem;

	ipa->ssr_cookie = qcom_register_ssr_notifier("mpss", &ipa->ssr_nb);
	if (IS_ERR(ipa->ssr_cookie))
		return dev_err_probe(dev, PTR_ERR(ipa->ssr_cookie),
				     "failed to register SSR notifier\n");

	ret = devm_add_action_or_reset(dev, action_qcom_unregister_ssr_notifier, ipa);
	if (ret)
		return ret;

	ipa->qmi = ipa_qmi_setup(dev, ipa->layout);
	if (IS_ERR(ipa->qmi))
		return PTR_ERR(ipa->qmi);

	if (ipa->smem_uc_loaded[0] == 0x10ADEDFF)
		ipa_qmi_uc_loaded(ipa->qmi);

	ret = devm_add_action_or_reset(dev, action_ipa_qmi_teardown, ipa);
	if (ret)
		return ret;

skip_modem:

	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	if (ipa->test_mode) {
		name = "ipa_lo%d";

		ipa->loopback = ipa_create_netdev(dev, name, ipa->ep + EP_TEST_RX,
						  ipa->ep + EP_TEST_TX);
		if (IS_ERR(ipa->loopback))
			return PTR_ERR(ipa->loopback);
	}

	ipa->modem = ipa_create_netdev(dev, name, ipa->ep + EP_RX,
				       ipa->ep + EP_TX);
	if (IS_ERR(ipa->modem))
		return PTR_ERR(ipa->modem);

	ipa->lan = ipa_create_netdev(dev, "ipa_lan%d", ipa->ep + EP_LAN_RX, NULL);
	if (IS_ERR(ipa->lan))
		return PTR_ERR(ipa->lan);

	if (ipa->test_mode)
		return 0;
	else
		ipa_modem_set_present(dev, false);

	ipa->ssr_cookie = qcom_register_ssr_notifier("mpss", &ipa->ssr_nb);
	if (IS_ERR(ipa->ssr_cookie))
		return dev_err_probe(dev, PTR_ERR(ipa->ssr_cookie),
				     "failed to register SSR notifier\n");

	ret = devm_add_action_or_reset(dev, action_qcom_unregister_ssr_notifier, ipa);
	if (ret)
		return ret;

	ipa->qmi = ipa_qmi_setup(dev, ipa->layout);
	if (IS_ERR(ipa->qmi))
		return PTR_ERR(ipa->qmi);

	if (ipa->smem_uc_loaded[0] == 0x10ADEDFF)
		ipa_qmi_uc_loaded(ipa->qmi);

	return devm_add_action_or_reset(dev, action_ipa_qmi_teardown, ipa);
}

static int ipa_remove(struct platform_device *pdev)
{
	struct ipa *ipa = platform_get_drvdata(pdev);
	struct device_node *np;
	struct rproc *rproc;

	if (!ipa->qmi || !ipa_qmi_is_modem_ready(ipa->qmi))
		return 0;

	np = of_parse_phandle(ipa->dev->of_node, "modem-remoteproc", 0);
	if (!np)
		return 0;

	rproc = rproc_get_by_phandle(np->phandle);
	of_node_put(np);
	if (!rproc)
		return 0;

	/* Should we bring it back up? */
	if (rproc->state == RPROC_RUNNING)
		rproc_shutdown(rproc);

	rproc_put(rproc);

	return 0;
}

static int ipa_runtime_resume(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	clk_set_rate(ipa->clk, 40000000);

	return 0;
}

static int ipa_runtime_suspend(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	clk_set_rate(ipa->clk, 9600000);

	return 0;
}

static int ipa_system_resume(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	ipa_ndev_suspend_resume(ipa->modem, true);
	ipa_ndev_suspend_resume(ipa->loopback, true);
	ipa_ndev_suspend_resume(ipa->lan, true);

	return 0;
}

static int ipa_system_suspend(struct device *dev)
{
	struct ipa *ipa = dev_get_drvdata(dev);

	ipa_ndev_suspend_resume(ipa->modem, false);
	ipa_ndev_suspend_resume(ipa->loopback, false);
	ipa_ndev_suspend_resume(ipa->lan, false);

	return 0;
}

static int ipa_modem_rx_id = EP_RX;
static int ipa_modem_tx_id = EP_TX;
static DEVICE_INT_ATTR(rx_endpoint_id, 0444, ipa_modem_rx_id);
static DEVICE_INT_ATTR(tx_endpoint_id, 0444, ipa_modem_tx_id);

static struct attribute *ipa_modem_attrs[] = {
	&dev_attr_rx_endpoint_id.attr.attr,
	&dev_attr_tx_endpoint_id.attr.attr,
	NULL
};

const struct attribute_group ipa_modem_group = {
	.name		= "modem",
	.attrs		= ipa_modem_attrs,
};

const struct attribute_group *ipa_groups[] = {
	&ipa_modem_group,
	NULL
};

static const struct of_device_id ipa_match[] = {
	{ .compatible	= "qcom,ipa-v2.5", (void *)25 },
	{ .compatible	= "qcom,ipa-lite-v2.6", (void *)26 },
	{ },
};

static const struct dev_pm_ops ipa_pm = {
	SET_RUNTIME_PM_OPS(ipa_runtime_suspend, ipa_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(ipa_system_suspend, ipa_system_resume)
};

static struct platform_driver ipa2_lite_driver = {
	.probe		= ipa_probe,
	.remove		= ipa_remove,
	.driver	= {
		.name		= "ipa",
		.dev_groups	= ipa_groups,
		.of_match_table	= ipa_match,
		.pm		= &ipa_pm
	},
};

module_platform_driver(ipa2_lite_driver);

MODULE_DEVICE_TABLE(of, ipa_match);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm IP Accelerator v2.X driver");
