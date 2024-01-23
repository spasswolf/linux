# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Qualcomm IPA driver.

IPA_REG_VERSIONS	:=	2.0 2.5 2.6l

IPA_DATA_VERSIONS	:=	2.0 2.5 2.6l

obj-$(CONFIG_QCOM_IPA_NEW_LEGACY)	+=	ipa-new-legacy.o

ipa-new-legacy-y	:=	ipa_main.o ipa_power.o ipa_reg.o ipa_mem.o \
				ipa_table.o ipa_interrupt.o bam.o \
				ipa_dma_trans.o ipa_gsi.o \
				ipa_uc.o ipa_endpoint.o ipa_cmd.o ipa_modem.o \
				ipa_resource.o ipa_qmi.o ipa_qmi_msg.o \
				ipa_sysfs.o

ipa-new-legacy-y	+=	$(IPA_REG_VERSIONS:%=reg/ipa_reg-v%.o)

ipa-new-legacy-y	+=	$(IPA_DATA_VERSIONS:%=data/ipa_data-v%.o)
