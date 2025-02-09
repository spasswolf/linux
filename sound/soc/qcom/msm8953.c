// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <uapi/linux/input-event-codes.h>
#include <dt-bindings/sound/qcom,lpass.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include "common.h"
#include "qdsp6/q6afe.h"

#define SLIM_MAX_TX_PORTS 16
#define SLIM_MAX_RX_PORTS 13
#define WCD9335_DEFAULT_MCLK_RATE	9600000

#define MI2S_COUNT  (MI2S_QUINARY + 1)

struct msm8953_snd_data {
	struct snd_soc_card card;
	void __iomem *mic_iomux;
	void __iomem *spkr_iomux;
	void __iomem *quin_iomux;
	// add more iomem here
	struct snd_soc_jack jack;
	bool jack_setup;
	bool slim_port_setup;
	bool use_ibit_clk;
	int mi2s_clk_count[MI2S_COUNT];
};

#define MIC_CTRL_TER_WS_SLAVE_SEL	BIT(21)
#define MIC_CTRL_QUA_WS_SLAVE_SEL_10	BIT(17)
#define MIC_CTRL_TLMM_SCLK_EN		BIT(1)
#define	SPKR_CTL_PRI_WS_SLAVE_SEL_11	(BIT(17) | BIT(16))
#define SPKR_CTL_TLMM_MCLK_EN		BIT(1)
#define SPKR_CTL_TLMM_SCLK_EN		BIT(2)
#define SPKR_CTL_TLMM_DATA1_EN		BIT(3)
#define SPKR_CTL_TLMM_WS_OUT_SEL_MASK	GENMASK(7, 6)
#define SPKR_CTL_TLMM_WS_OUT_SEL_SEC	BIT(6)
#define SPKR_CTL_TLMM_WS_EN_SEL_MASK	GENMASK(19, 18)
#define SPKR_CTL_TLMM_WS_EN_SEL_SEC	BIT(18)
#define DEFAULT_MCLK_RATE		9600000
#define MI2S_BCLK_RATE			1536000

static struct snd_soc_jack_pin msm8953_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

// from apq8096.c
static int msm_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	u32 rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret = 0;

	ret = snd_soc_dai_get_channel_map(codec_dai,
				&tx_ch_cnt, tx_ch, &rx_ch_cnt, rx_ch);
	if (ret != 0 && ret != -ENOTSUPP) {
		pr_err("failed to get codec chan map, err:%d\n", ret);
		goto end;
	} else if (ret == -ENOTSUPP) {
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  rx_ch_cnt, rx_ch);
	else
		ret = snd_soc_dai_set_channel_map(cpu_dai, tx_ch_cnt, tx_ch,
						  0, NULL);
	if (ret != 0 && ret != -ENOTSUPP)
		pr_err("Failed to set cpu chan map, err:%d\n", ret);
	else if (ret == -ENOTSUPP)
		ret = 0;
end:
	return ret;
}

static int msm8953_snd_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	//struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret = 0;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
	case SECONDARY_MI2S_RX:
	case SECONDARY_MI2S_TX:
	case TERTIARY_MI2S_RX:
	case TERTIARY_MI2S_TX:
	case QUATERNARY_MI2S_RX:
	case QUATERNARY_MI2S_TX:
	case QUINARY_MI2S_RX:
	case QUINARY_MI2S_TX:
		break;
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		ret = msm_snd_hw_params(substream, params);
		break;
	default:
		printk(KERN_INFO "%s: invalid dai id 0x%x\n", __func__, cpu_dai->id);
		break;
	}
	return ret;
}

static int msm8953_qdsp6_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_component *component;
	struct snd_soc_card *card = rtd->card;
	struct msm8953_snd_data *pdata = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int i, rval;
	u32 value;

	unsigned int rx_ch[SLIM_MAX_RX_PORTS] = {144, 145, 146, 147, 148, 149,
					150, 151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[SLIM_MAX_TX_PORTS] = {128, 129, 130, 131, 132, 133,
					    134, 135, 136, 137, 138, 139,
					    140, 141, 142, 143};

	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP);

	// sdm845.c has this before the switch/case
	if (!pdata->jack_setup) {
		struct snd_jack *jack;

		rval = snd_soc_card_jack_new_pins(card, "Headset Jack",
						  SND_JACK_HEADSET |
						  SND_JACK_HEADPHONE |
						  SND_JACK_BTN_0 | SND_JACK_BTN_1 |
						  SND_JACK_BTN_2 | SND_JACK_BTN_3 |
						  SND_JACK_BTN_4,
						  &pdata->jack,
						  msm8953_jack_pins,
						  ARRAY_SIZE(msm8953_jack_pins));

		if (rval < 0) {
			dev_err(card->dev, "Unable to add Headphone Jack\n");
			return rval;
		}

		jack = pdata->jack.jack;

		snd_jack_set_key(jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
		snd_jack_set_key(jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
		snd_jack_set_key(jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
		snd_jack_set_key(jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);
		pdata->jack_setup = true;
	}

	switch (cpu_dai->id) {
	#if 0
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
		printk(KERN_INFO "%s: configuring primary audio\n", __func__);
		writel(readl(pdata->spkr_iomux) | SPKR_CTL_PRI_WS_SLAVE_SEL_11,
			pdata->spkr_iomux);
		break;

	case QUATERNARY_MI2S_RX:
	case QUATERNARY_MI2S_TX:
		printk(KERN_INFO "%s: configuring quaternary audio\n", __func__);
		/* Configure the Quat MI2S to TLMM */
		writel(readl(pdata->mic_iomux) | MIC_CTRL_QUA_WS_SLAVE_SEL_10 |
			MIC_CTRL_TLMM_SCLK_EN,
			pdata->mic_iomux);
		break;
	case SECONDARY_MI2S_RX:
	case SECONDARY_MI2S_TX:
		printk(KERN_INFO "%s: configuring secondary audio\n", __func__);
		/* Clear TLMM_WS_OUT_SEL and TLMM_WS_EN_SEL fields */
		value = readl(pdata->spkr_iomux) &
			~(SPKR_CTL_TLMM_WS_OUT_SEL_MASK | SPKR_CTL_TLMM_WS_EN_SEL_MASK);
		/* Configure the Sec MI2S to TLMM */
		writel(value | SPKR_CTL_TLMM_MCLK_EN | SPKR_CTL_TLMM_SCLK_EN |
			SPKR_CTL_TLMM_DATA1_EN | SPKR_CTL_TLMM_WS_OUT_SEL_SEC |
			SPKR_CTL_TLMM_WS_EN_SEL_SEC, pdata->spkr_iomux);
		break;
	case TERTIARY_MI2S_RX:
	case TERTIARY_MI2S_TX:
		printk(KERN_INFO "%s: configuring tertiary audio\n", __func__);
		writel(readl(pdata->mic_iomux) | MIC_CTRL_TER_WS_SLAVE_SEL |
			MIC_CTRL_TLMM_SCLK_EN,
			pdata->mic_iomux);

		break;
	#endif
	case QUINARY_MI2S_RX:
	case QUINARY_MI2S_TX:
		/* Configure Quinary MI2S */
		printk(KERN_INFO "%s: configuring quinary audio\n", __func__);
		if (!pdata->quin_iomux)
			return -ENOENT;
		writel(readl(pdata->quin_iomux) | 0x01, pdata->quin_iomux);
		break;
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		// TODO
		printk(KERN_INFO "%s: configuring slim audio\n", __func__);
		snd_soc_dai_set_channel_map(cpu_dai, ARRAY_SIZE(tx_ch),
						tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

		snd_soc_dai_set_sysclk(cpu_dai, 0, WCD9335_DEFAULT_MCLK_RATE,
					SNDRV_PCM_STREAM_PLAYBACK);
		break;
	default:
		dev_err(card->dev, "unsupported cpu dai configuration\n");
		return -EINVAL;

	}

	for_each_rtd_codec_dais(rtd, i, codec_dai) {

		component = codec_dai->component;
		/* Set default mclk for internal codec */
		rval = snd_soc_component_set_sysclk(component, 0, 0, DEFAULT_MCLK_RATE,
				       SND_SOC_CLOCK_IN);
		if (rval != 0 && rval != -ENOTSUPP) {
			dev_warn(card->dev, "Failed to set mclk: %d\n", rval);
			return rval;
		}
		rval = snd_soc_component_set_jack(component, &pdata->jack, NULL);
		if (rval != 0 && rval != -ENOTSUPP) {
			dev_warn(card->dev, "Failed to set jack: %d\n", rval);
			return rval;
		}
	}

	return 0;
}

static int qdsp6_dai_get_lpass_id(struct snd_soc_dai *cpu_dai)
{
	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
		return MI2S_PRIMARY;
	case SECONDARY_MI2S_RX:
	case SECONDARY_MI2S_TX:
		return MI2S_SECONDARY;
	case TERTIARY_MI2S_RX:
	case TERTIARY_MI2S_TX:
		return MI2S_TERTIARY;
	case QUATERNARY_MI2S_RX:
	case QUATERNARY_MI2S_TX:
		return MI2S_QUATERNARY;
	case QUINARY_MI2S_RX:
	case QUINARY_MI2S_TX:
		return MI2S_QUINARY;
	default:
		return -EINVAL;
	}
}

static int qdsp6_get_bit_clk_id(struct msm8953_snd_data *data, int mi2s_id)
{
	if (data->use_ibit_clk) {
		switch (mi2s_id) {
		case MI2S_PRIMARY:
			return Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT;
		case MI2S_SECONDARY:
			return Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT;
		case MI2S_TERTIARY:
			return Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT;
		case MI2S_QUATERNARY:
			return Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT;
		case MI2S_QUINARY:
			return Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT;
		default:
			break;
		}
	}

	return LPAIF_BIT_CLK;
}

static int msm8953_qdsp6_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct msm8953_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int mi2s, ret;

	mi2s = qdsp6_dai_get_lpass_id(cpu_dai);
	if (mi2s < 0)
		return mi2s;

	if (++data->mi2s_clk_count[mi2s] > 1)
		return 0;


	/* HACK For making external codecs work
	 *
	 * For some codecs in the Quinary DAI link we have to explicitly set the
	 * format to I2S.
	 */
	if (cpu_dai->id == QUINARY_MI2S_RX) {
		snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_BC_FC | SND_SOC_DAIFMT_I2S);
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, qdsp6_get_bit_clk_id(data, mi2s), MI2S_BCLK_RATE, 0);
	if (ret)
		dev_err(card->dev, "Failed to enable LPAIF bit clk: %d\n", ret);
	return ret;
}

static void msm8953_qdsp6_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct msm8953_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int mi2s, ret;

	mi2s = qdsp6_dai_get_lpass_id(cpu_dai);
	if (mi2s < 0)
		return;

	if (--data->mi2s_clk_count[mi2s] > 0)
		return;

	ret = snd_soc_dai_set_sysclk(cpu_dai, qdsp6_get_bit_clk_id(data, mi2s), 0, 0);
	if (ret)
		dev_err(card->dev, "Failed to disable LPAIF bit clk: %d\n", ret);
}

static const struct snd_soc_ops msm8953_qdsp6_be_ops = {
	.hw_params = msm8953_snd_hw_params,
	.startup = msm8953_qdsp6_startup,
	.shutdown = msm8953_qdsp6_shutdown,
};

static int msm8953_qdsp6_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	//struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	//snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static void msm8953_qdsp6_add_ops(struct snd_soc_card *card)
{
	struct msm8953_snd_data *pdata = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai_link *link;
	int i;

	pdata->use_ibit_clk = true;

	/* Make it obvious to userspace that QDSP6 is used */
	card->components = "qdsp6";

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->init = msm8953_qdsp6_dai_init;
			link->ops = &msm8953_qdsp6_be_ops;
			link->be_hw_params_fixup = msm8953_qdsp6_be_hw_params_fixup;
		}
		// FIXME: Do we need something here?
		//link->init = msm8953_dai_init;
	}
}

static const struct snd_kcontrol_new msm8953_snd_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
};

static const struct snd_soc_dapm_widget msm8953_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Secondary Mic", NULL),
	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
};

static int msm8953_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct msm8953_snd_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card = &data->card;
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->dapm_widgets = msm8953_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(msm8953_dapm_widgets);
	card->controls = msm8953_snd_controls;
	card->num_controls = ARRAY_SIZE(msm8953_snd_controls);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	data->mic_iomux = devm_platform_ioremap_resource_byname(pdev, "mic-iomux");
	if (IS_ERR(data->mic_iomux))
		return PTR_ERR(data->mic_iomux);

	data->spkr_iomux = devm_platform_ioremap_resource_byname(pdev, "spkr-iomux");
	if (IS_ERR(data->spkr_iomux))
		return PTR_ERR(data->spkr_iomux);

	data->quin_iomux = devm_platform_ioremap_resource_byname(pdev, "quin-iomux");
	if (IS_ERR(data->quin_iomux))
		return PTR_ERR(data->quin_iomux);

	snd_soc_card_set_drvdata(card, data);

	msm8953_qdsp6_add_ops(card);
	return devm_snd_soc_register_card(&pdev->dev, card);
}

static const struct of_device_id msm8953_device_id[] __maybe_unused = {
	{ .compatible = "qcom,msm8953-qdsp6-sndcard-alt",},
	{},
};
MODULE_DEVICE_TABLE(of, msm8953_device_id);

static struct platform_driver msm8953_platform_driver = {
	.driver = {
		.name = "qcom-msm8953-alt",
		.of_match_table = of_match_ptr(msm8953_device_id),
	},
	.probe = msm8953_platform_probe,
};
module_platform_driver(msm8953_platform_driver);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_AUTHOR("Bert Karwatzki <spasswolf@web.de>");
MODULE_DESCRIPTION("MSM8953 ASoC Machine Driver");
MODULE_LICENSE("GPL");
