/* SPDX-License-Identifier: GPL-2.0 */

/* Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 * Copyright (C) 2018-2023 Linaro Ltd.
 */
#ifndef _IPA_REG_H_
#define _IPA_REG_H_

#include <linux/bitfield.h>
#include <linux/bug.h>

#include "ipa_version.h"
#include "reg.h"

struct ipa;

/**
 * DOC: IPA Registers
 *
 * IPA registers are located within the "ipa-reg" address space defined by
 * Device Tree.  Each register has a specified offset within that space,
 * which is mapped into virtual memory space in ipa_mem_init().  Each
 * has a unique identifer, taken from the ipa_reg_id enumerated type.
 * All IPA registers are 32 bits wide.
 *
 * Certain "parameterized" register types are duplicated for a number of
 * instances of something.  For example, each IPA endpoint has an set of
 * registers defining its configuration.  The offset to an endpoint's set
 * of registers is computed based on an "base" offset, plus an endpoint's
 * ID multiplied and a "stride" value for the register.  Similarly, some
 * registers have an offset that depends on execution environment.  In
 * this case, the stride is multiplied by a member of the dma_ee_id
 * enumerated type.
 *
 * Each version of IPA implements an array of ipa_reg structures indexed
 * by register ID.  Each entry in the array specifies the base offset and
 * (for parameterized registers) a non-zero stride value.  Not all versions
 * of IPA define all registers.  The offset for a register is returned by
  reg_offset() when the register's ipa_reg structure is supplied;
 * zero is returned for an undefined register (this should never happen).
 *
 * Some registers encode multiple fields within them.  Each field in
 * such a register has a unique identifier (from an enumerated type).
 * The position and width of the fields in a register are defined by
 * an array of field masks, indexed by field ID.  Two functions are
 * used to access register fields; both take an ipa_reg structure as
 * argument.  To encode a value to be represented in a register field,
 * the value and field ID are passed to reg_encode().  To extract
 * a value encoded in a register field, the field ID is passed to
 * reg_decode().  In addition, for single-bit fields, reg_bit()
 * can be used to either encode the bit value, or to generate a mask
 * used to extract the bit value.
 */

/* enum ipa_reg_id - IPA register IDs */
enum ipa_reg_id {
	COMP_CFG,
	COMP_SW_RESET,					/* IPA v2.0 */
	CLKON_CFG,
	ROUTE,
	SHARED_MEM_SIZE,
	IPA_BCR,					/* IPA v2.5+, Not IPA v4.5+ */
	ENABLED_PIPES,					/* IPA v2.6l+ */
	LOCAL_PKT_PROC_CNTXT,				/* IPA v2.5, IPA v3.0+ */
	AGGR_FORCE_CLOSE,
	COUNTER_CFG,					/* IPA v2.5+, Not IPA v4.5+ */
	ENDP_INIT_CTRL,		/* Not IPA v4.2+ for TX, not IPA v4.0+ for RX */
	ENDP_INIT_CFG,
	ENDP_INIT_NAT,			/* TX only */
	ENDP_INIT_HDR,
	ENDP_INIT_HDR_EXT,
	ENDP_INIT_HDR_METADATA_MASK,	/* RX only */
	ENDP_INIT_MODE,			/* TX only */
	ENDP_INIT_AGGR,
	ENDP_INIT_HOL_BLOCK_EN,		/* RX only */
	ENDP_INIT_HOL_BLOCK_TIMER,	/* RX only */
	ENDP_INIT_DEAGGR,		/* TX only */
	ENDP_INIT_RSRC_GRP,
	ENDP_INIT_SEQ,			/* TX only */
	ENDP_STATUS,
	/* The IRQ registers that follow are only used for IPA_EE_AP */
	IPA_IRQ_STTS,
	IPA_IRQ_EN,
	IPA_IRQ_CLR,
	IRQ_SUSPEND_INFO,
	IPA_REG_ID_COUNT,				/* Last; not an ID */
};

/* COMP_CFG register */
enum ipa_reg_comp_cfg_field_id {
	COMP_CFG_ENABLE,				/* Not IPA v4.0+ */
};

/* CLKON_CFG register */
enum ipa_reg_clkon_cfg_field_id {
	CLKON_RX,
	CLKON_PROC,
	TX_WRAPPER,
	CLKON_MISC,
	RAM_ARB,
	FTCH_HPS,
	FTCH_DPS,
	CLKON_HPS,
	CLKON_DPS,
	RX_HPS_CMDQS,
	HPS_DPS_CMDQS,
	DPS_TX_CMDQS,
	RSRC_MNGR,
	CTX_HANDLER,
	ACK_MNGR,
	D_DCPH,
	H_DCPH,
};

/* ROUTE register */
enum ipa_reg_route_field_id {
	ROUTE_DIS,
	ROUTE_DEF_PIPE,
	ROUTE_DEF_HDR_TABLE,
	ROUTE_DEF_HDR_OFST,
	ROUTE_FRAG_DEF_PIPE,
	ROUTE_DEF_RETAIN_HDR,
};

/* SHARED_MEM_SIZE register */
enum ipa_reg_shared_mem_size_field_id {
	MEM_SIZE,
	MEM_BADDR,
};

/* BCR register */
enum ipa_bcr_compat {
	BCR_CMDQ_L_LACK_ONE_ENTRY		= 0x0,	/* Not IPA v4.2+ */
	BCR_TX_NOT_USING_BRESP			= 0x1,	/* Not IPA v4.2+ */
	BCR_TX_SUSPEND_IRQ_ASSERT_ONCE		= 0x2,	/* Not IPA v4.0+ */
	BCR_SUSPEND_L2_IRQ			= 0x3,	/* Not IPA v4.2+ */
	BCR_HOLB_DROP_L2_IRQ			= 0x4,	/* Not IPA v4.2+ */
};

/* LOCAL_PKT_PROC_CNTXT register */
enum ipa_reg_local_pkt_proc_cntxt_field_id {
	IPA_BASE_ADDR,
};

/* COUNTER_CFG register */
enum ipa_reg_counter_cfg_field_id {
	EOT_COAL_GRANULARITY,				/* Not v3.5+ */
	AGGR_GRANULARITY,
};

/* ENDP_INIT_CTRL register */
enum ipa_reg_endp_init_ctrl_field_id {
	ENDP_SUSPEND,					/* Not v4.0+ */
	ENDP_DELAY,					/* Not v4.2+ */
};

/* ENDP_INIT_CFG register */
enum ipa_reg_endp_init_cfg_field_id {
	FRAG_OFFLOAD_EN,
	CS_OFFLOAD_EN,
	CS_METADATA_HDR_OFFSET,
	CS_GEN_QMB_MASTER_SEL,
};

/** enum ipa_cs_offload_en - ENDP_INIT_CFG register CS_OFFLOAD_EN field value */
enum ipa_cs_offload_en {
	IPA_CS_OFFLOAD_NONE			= 0x0,
	IPA_CS_OFFLOAD_UL	/* TX */	= 0x1,	/* Not IPA v4.5+ */
	IPA_CS_OFFLOAD_DL	/* RX */	= 0x2,	/* Not IPA v4.5+ */
	IPA_CS_OFFLOAD_INLINE	/* TX and RX */	= 0x1,	/* IPA v4.5+ */
};

/* ENDP_INIT_NAT register */
enum ipa_reg_endp_init_nat_field_id {
	NAT_EN,
};

/** enum ipa_nat_type - ENDP_INIT_NAT register NAT_EN field value */
enum ipa_nat_type {
	IPA_NAT_TYPE_BYPASS			= 0,
	IPA_NAT_TYPE_SRC			= 1,
	IPA_NAT_TYPE_DST			= 2,
};

/* ENDP_INIT_HDR register */
enum ipa_reg_endp_init_hdr_field_id {
	HDR_LEN,
	HDR_OFST_METADATA_VALID,
	HDR_OFST_METADATA,
	HDR_ADDITIONAL_CONST_LEN,
	HDR_OFST_PKT_SIZE_VALID,
	HDR_OFST_PKT_SIZE,
	HDR_A5_MUX,					/* Not v4.9+ */
	HDR_LEN_INC_DEAGG_HDR,
	HDR_METADATA_REG_VALID,				/* Not v4.5+ */
};

/* ENDP_INIT_HDR_EXT register */
enum ipa_reg_endp_init_hdr_ext_field_id {
	HDR_ENDIANNESS,
	HDR_TOTAL_LEN_OR_PAD_VALID,
	HDR_TOTAL_LEN_OR_PAD,
	HDR_PAYLOAD_LEN_INC_PADDING,
	HDR_TOTAL_LEN_OR_PAD_OFFSET,
	HDR_PAD_TO_ALIGNMENT,
};

/* ENDP_INIT_MODE register */
enum ipa_reg_endp_init_mode_field_id {
	ENDP_MODE,
	DEST_PIPE_INDEX,
	BYTE_THRESHOLD,
	PIPE_REPLICATION_EN,
	PAD_EN,
};

/** enum ipa_mode - ENDP_INIT_MODE register MODE field value */
enum ipa_mode {
	IPA_BASIC				= 0x0,
	IPA_ENABLE_FRAMING_HDLC			= 0x1,
	IPA_ENABLE_DEFRAMING_HDLC		= 0x2,
	IPA_DMA					= 0x3,
};

/* ENDP_INIT_AGGR register */
enum ipa_reg_endp_init_aggr_field_id {
	AGGR_EN,
	AGGR_TYPE,
	BYTE_LIMIT,
	TIME_LIMIT,
	PKT_LIMIT,
	SW_EOF_ACTIVE,
	FORCE_CLOSE,
	HARD_BYTE_LIMIT_EN,
	AGGR_GRAN_SEL,
};

/** enum ipa_aggr_en - ENDP_INIT_AGGR register AGGR_EN field value */
enum ipa_aggr_en {
	IPA_BYPASS_AGGR		/* TX and RX */	= 0x0,
	IPA_ENABLE_AGGR		/* RX */	= 0x1,
	IPA_ENABLE_DEAGGR	/* TX */	= 0x2,
};

/** enum ipa_aggr_type - ENDP_INIT_AGGR register AGGR_TYPE field value */
enum ipa_aggr_type {
	IPA_MBIM_16				= 0x0,
	IPA_HDLC				= 0x1,
	IPA_TLP					= 0x2,
	IPA_RNDIS				= 0x3,
	IPA_GENERIC				= 0x4,
	IPA_COALESCE				= 0x5,
	IPA_QCMAP				= 0x6,
};

/* ENDP_INIT_HOL_BLOCK_EN register */
enum ipa_reg_endp_init_hol_block_en_field_id {
	HOL_BLOCK_EN,
};

/* ENDP_INIT_HOL_BLOCK_TIMER register */
enum ipa_reg_endp_init_hol_block_timer_field_id {
	TIMER_BASE_VALUE,				/* Not v4.5+ */
};

/* ENDP_INIT_DEAGGR register */
enum ipa_reg_endp_deaggr_field_id {
	DEAGGR_HDR_LEN,
	SYSPIPE_ERR_DETECTION,
	PACKET_OFFSET_VALID,
	PACKET_OFFSET_LOCATION,
	IGNORE_MIN_PKT_ERR,
	MAX_PACKET_LEN,
};

/* ENDP_INIT_RSRC_GRP register */
enum ipa_reg_endp_init_rsrc_grp_field_id {
	ENDP_RSRC_GRP,
};

/* ENDP_INIT_SEQ register */
enum ipa_reg_endp_init_seq_field_id {
	SEQ_TYPE,
	SEQ_REP_TYPE,					/* Not v4.5+ */
};

/**
 * enum ipa_seq_type - HPS and DPS sequencer type
 * @IPA_SEQ_DMA:		 Perform DMA only
 * @IPA_SEQ_1_PASS:		 One pass through the pipeline
 * @IPA_SEQ_2_PASS_SKIP_LAST_UC: Two passes, skip the microcprocessor
 * @IPA_SEQ_1_PASS_SKIP_LAST_UC: One pass, skip the microcprocessor
 * @IPA_SEQ_2_PASS:		 Two passes through the pipeline
 * @IPA_SEQ_3_PASS_SKIP_LAST_UC: Three passes, skip the microcprocessor
 * @IPA_SEQ_DECIPHER:		 Optional deciphering step (combined)
 *
 * The low-order byte of the sequencer type register defines the number of
 * passes a packet takes through the IPA pipeline.  The last pass through can
 * optionally skip the microprocessor.  Deciphering is optional for all types;
 * if enabled, an additional mask (two bits) is added to the type value.
 *
 * Note: not all combinations of ipa_seq_type and ipa_seq_rep_type are
 * supported (or meaningful).
 */
enum ipa_seq_type {
	IPA_SEQ_DMA				= 0x00,
	IPA_SEQ_1_PASS				= 0x02,
	IPA_SEQ_2_PASS_SKIP_LAST_UC		= 0x04,
	IPA_SEQ_1_PASS_SKIP_LAST_UC		= 0x06,
	IPA_SEQ_2_PASS				= 0x0a,
	IPA_SEQ_3_PASS_SKIP_LAST_UC		= 0x0c,
	/* The next value can be ORed with the above */
	IPA_SEQ_DECIPHER			= 0x11,
};

/**
 * enum ipa_seq_rep_type - replicated packet sequencer type
 * @IPA_SEQ_REP_DMA_PARSER:	DMA parser for replicated packets
 *
 * This goes in the second byte of the endpoint sequencer type register.
 *
 * Note: not all combinations of ipa_seq_type and ipa_seq_rep_type are
 * supported (or meaningful).
 */
enum ipa_seq_rep_type {
	IPA_SEQ_REP_DMA_PARSER			= 0x08,
};

/* ENDP_STATUS register */
enum ipa_reg_endp_status_field_id {
	STATUS_EN,
	STATUS_ENDP,
	STATUS_LOCATION,				/* Not v4.5+ */
	STATUS_PKT_SUPPRESS,				/* v4.0+ */
};

/* IPA_IRQ_STTS, IPA_IRQ_EN, and IPA_IRQ_CLR registers */
/**
 * enum ipa_irq_id - Bit positions representing type of IPA IRQ
 * @IPA_IRQ_UC_0:	Microcontroller event interrupt
 * @IPA_IRQ_UC_1:	Microcontroller response interrupt
 * @IPA_IRQ_TX_SUSPEND:	Data ready interrupt
 * @IPA_IRQ_COUNT:	Number of IRQ ids (must be last)
 *
 * IRQ types not described above are not currently used.
 *
 * @IPA_IRQ_BAD_SNOC_ACCESS:		(Not currently used)
 * @IPA_IRQ_EOT_COAL:			(Not currently used)
 * @IPA_IRQ_UC_2:			(Not currently used)
 * @IPA_IRQ_UC_3:			(Not currently used)
 * @IPA_IRQ_UC_IN_Q_NOT_EMPTY:		(Not currently used)
 * @IPA_IRQ_UC_RX_CMD_Q_NOT_FULL:	(Not currently used)
 * @IPA_IRQ_PROC_UC_ACK_Q_NOT_EMPTY:	(Not currently used)
 * @IPA_IRQ_RX_ERR:			(Not currently used)
 * @IPA_IRQ_DEAGGR_ERR:			(Not currently used)
 * @IPA_IRQ_TX_ERR:			(Not currently used)
 * @IPA_IRQ_STEP_MODE:			(Not currently used)
 * @IPA_IRQ_PROC_ERR:			(Not currently used)
 * @IPA_IRQ_TX_HOLB_DROP:		(Not currently used)
 * @IPA_IRQ_BAM_GSI_IDLE:		(Not currently used)
 * @IPA_IRQ_PIPE_YELLOW_BELOW:		(Not currently used)
 * @IPA_IRQ_PIPE_RED_BELOW:		(Not currently used)
 * @IPA_IRQ_PIPE_YELLOW_ABOVE:		(Not currently used)
 * @IPA_IRQ_PIPE_RED_ABOVE:		(Not currently used)
 * @IPA_IRQ_UCP:			(Not currently used)
 * @IPA_IRQ_DCMP:			(Not currently used)
 * @IPA_IRQ_GSI_EE:			(Not currently used)
 * @IPA_IRQ_GSI_IPA_IF_TLV_RCVD:	(Not currently used)
 * @IPA_IRQ_GSI_UC:			(Not currently used)
 * @IPA_IRQ_TLV_LEN_MIN_DSM:		(Not currently used)
 * @IPA_IRQ_DRBIP_PKT_EXCEED_MAX_SIZE_EN: (Not currently used)
 * @IPA_IRQ_DRBIP_DATA_SCTR_CFG_ERROR_EN: (Not currently used)
 * @IPA_IRQ_DRBIP_IMM_CMD_NO_FLSH_HZRD_EN: (Not currently used)
 */
enum ipa_irq_id {
	IPA_IRQ_BAD_SNOC_ACCESS			= 0x0,
	/* The next bit is not present for IPA v3.5+ */
	IPA_IRQ_EOT_COAL			= 0x1,
	IPA_IRQ_UC_0				= 0x2,
	IPA_IRQ_UC_1				= 0x3,
	IPA_IRQ_UC_2				= 0x4,
	IPA_IRQ_UC_3				= 0x5,
	IPA_IRQ_UC_IN_Q_NOT_EMPTY		= 0x6,
	IPA_IRQ_UC_RX_CMD_Q_NOT_FULL		= 0x7,
	IPA_IRQ_PROC_UC_ACK_Q_NOT_EMPTY		= 0x8,
	IPA_IRQ_RX_ERR				= 0x9,
	IPA_IRQ_DEAGGR_ERR			= 0xa,
	IPA_IRQ_TX_ERR				= 0xb,
	IPA_IRQ_STEP_MODE			= 0xc,
	IPA_IRQ_PROC_ERR			= 0xd,
	IPA_IRQ_TX_SUSPEND			= 0xe,
	IPA_IRQ_TX_HOLB_DROP			= 0xf,
	IPA_IRQ_BAM_GSI_IDLE			= 0x10,
	IPA_IRQ_PIPE_YELLOW_BELOW		= 0x11,
	IPA_IRQ_PIPE_RED_BELOW			= 0x12,
	IPA_IRQ_PIPE_YELLOW_ABOVE		= 0x13,
	IPA_IRQ_PIPE_RED_ABOVE			= 0x14,
	IPA_IRQ_UCP				= 0x15,
	/* The next bit is not present for IPA v4.5+ */
	IPA_IRQ_DCMP				= 0x16,
	IPA_IRQ_GSI_EE				= 0x17,
	IPA_IRQ_GSI_IPA_IF_TLV_RCVD		= 0x18,
	IPA_IRQ_GSI_UC				= 0x19,
	/* The next bit is present for IPA v4.5+ */
	IPA_IRQ_TLV_LEN_MIN_DSM			= 0x1a,
	/* The next three bits are present for IPA v4.9+ */
	IPA_IRQ_DRBIP_PKT_EXCEED_MAX_SIZE_EN	= 0x1b,
	IPA_IRQ_DRBIP_DATA_SCTR_CFG_ERROR_EN	= 0x1c,
	IPA_IRQ_DRBIP_IMM_CMD_NO_FLSH_HZRD_EN	= 0x1d,
	IPA_IRQ_COUNT,				/* Last; not an id */
};

extern const struct regs ipa_regs_v2_0;
extern const struct regs ipa_regs_v2_5;
extern const struct regs ipa_regs_v2_6l;

const struct reg *ipa_reg(struct ipa *ipa, enum ipa_reg_id reg_id);

int ipa_reg_init(struct ipa *ipa);
void ipa_reg_exit(struct ipa *ipa);

#endif /* _IPA_REG_H_ */
