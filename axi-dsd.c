/*
 * Copyright (C) 2017 JAB Research LLC
 * Author: Jonathan Borden <jonathan@jabresearch.com>
 *
 * Xilinx zynq based DAC PCM/DSD interface
 * axi-dsd DAC controller
 * axi-dma -> FIFO -> axi stream data interface
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "axi-dsd.h"

#define AXI_STREAM_FIFO_ISR 0x0
#define AXI_STREAM_FIFO_IER 0x4
#define AXI_STREAM_FIFO_TDFR 0x8
#define AXI_STREAM_FIFO_TDFV 0xC
#define AXI_STREAM_FIFO_TDFD 0x10
#define AXI_STREAM_FIFO_TLR 0x14
#define AXI_STREAM_FIFO_RDFR 0x18
#define AXI_STREAM_FIFO_RDFO 0x1C Read
#define AXI_STREAM_FIFO_RDFD 0x20

#define AXI_STREAM_FIFO_RLR 0x24 Read
#define AXI_STREAM_FIFO_SRR 0x28 Write(2)
#define AXI_STREAM_FIFO_TDR 0x2C Write
#define AXI_STREAM_FIFO_RDR 0x30 Read
/* Transmit ID Register(4) C_BASEADDR + x34 Write
Transmit USER Register(4) C_BASEADDR + x38 Write
Receive ID Register(4) C_BASEADDR + x3C Read
Receive USER Register(4) C_BASEADDR + x40 Read
*/
#define AXI_DMA_MM2S_DMACR 0x00 /* DMA control register */
#define AXI_DMA_MM2S_DMASR 0x04 /* DMA status register */
#define AXI_DMA_MM2S_SA 0x18
#define AXI_DMA_MM2S_SA_MSB 0x0C
#define AXI_DMA_MM2S_LENGTH 0X28

#define AXI_DMA_S2MM_DMACR 0x30
#define AXI_DMA_S2MM_DMASR 0x34
#define AXI_DMA_S2MM_DA 0X48
#define AXI_DMA_S2MM_DA_MSB 0x4C
#define AXI_DMA_S2MM_LENGTH 0X58
#define AXI_DMA_REG_MAX 0x60

#define AXI_I2S_REG_RESET	0x00
#define AXI_I2S_REG_CTRL	0x04
#define AXI_I2S_REG_CLK_CTRL	0x08
#define AXI_I2S_REG_STATUS	0x10

#define AXI_I2S_REG_RX_FIFO	0x28
#define AXI_I2S_REG_TX_FIFO	0x2C

#define AXI_I2S_RESET_GLOBAL	BIT(0)
#define AXI_I2S_RESET_TX_FIFO	BIT(1)
#define AXI_I2S_RESET_RX_FIFO	BIT(2)

#define AXI_I2S_CTRL_TX_EN	BIT(0)
#define AXI_I2S_CTRL_RX_EN	BIT(1)

/* The frame size is configurable, but for now we always set it 64 bit */
#define AXI_I2S_BITS_PER_FRAME 64

#define AXI_DSD_BIT_DSD BIT(0)
#define AXI_DSD_BIT_PCM BIT(1)
#define AXI_DSD_BIT_FAM_CLOCK BIT(2)
#define AXI_DSD_BIT_RESET_PARAMS BIT(3)
#define AXI_DSD_REG_FLAGS 0x00
#define AXI_DSD_REG_VOLUME 0x04
#define AXI_DSD_REG_RATE 0x08
#define AXI_DSD_REG_CLKDIV 0x0C
#define AXI_DSD_REG_PCM_MODE 0x10
#define AXI_DSD_REG_CHANNELS 0x14
#define AXI_DSD_REG_MAX 0x18

#define AXI_DSD_PCM_RATES SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_88200|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_176400|SNDRV_PCM_RATE_192000|SNDRV_PCM_RATE_KNOT
#define AXI_DSD_PCM_FMTS SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_U32_LE | SNDRV_PCM_FMTBIT_DSD_U32_LE


#define JBDAC_AXI_CONTROLLER 0x43C00000
#define JBDAC_AXI_DMA        0X40400000

#define CLK_DIV_MIN 1
#define CLK_DIV_MAX 64
struct axi_dsd;
struct axi_dsd_ruledata {
    struct axi_dsd *dsd;
    int serializers;
};
struct axi_dsd {
	struct regmap *regmap_dma;
    struct regmap *regmap_controller;
	struct clk *clkA;
	struct clk *clkB;
    uint bclk_div;
    uint bclk_rate;
    uint nchan;
    bool clk_fam;
    bool is_dsd;
    struct device *dev;
    struct device *dev_dma;
    

	//struct snd_soc_dai_driver dai_driver;

	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_pcm_substream *substream;

	struct snd_ratnum ratnum[2];
	struct axi_dsd_ruledata ruledata[2];
	struct snd_pcm_hw_constraint_ratnums rate_constraints;

};

// regmap_read_poll_timeout(map, addr, val, cond, sleep_us, timeout_us)
static int axi_dsd_wait_dma_done(struct axi_dsd *dsd, uint sleep_us, uint timeout_us)
{
    uint val;
    return regmap_read_poll_timeout(dsd->regmap_dma, AXI_DMA_MM2S_DMASR ,val, val & (DMASR_Halted|DMASR_Idle), sleep_us, timeout_us);
}
static int get_controller_flags(struct axi_dsd *dsd, bool immed)
{
    int flags = 0;
    if (dsd->is_dsd) flags |= AXI_DSD_BIT_DSD;
    if (dsd->clk_fam) flags |= AXI_DSD_BIT_FAM_CLOCK;
    if (immed) flags |= AXI_DSD_BIT_RESET_PARAMS;
    return flags;
}
static uint axi_dsd_channels(struct axi_dsd *dsd){ return dsd->nchan; };
static bool axi_dsd_is_dsd(struct axi_dsd *dsd){ return dsd->is_dsd;};
static bool is_dsd(snd_pcm_format_t format)
{
    switch (format) {
        case SNDRV_PCM_FORMAT_DSD_U8:
        case SNDRV_PCM_FORMAT_DSD_U16_LE:
        case SNDRV_PCM_FORMAT_DSD_U16_BE:
        case SNDRV_PCM_FORMAT_DSD_U32_LE:
        case SNDRV_PCM_FORMAT_DSD_U32_BE:
            return true;
            break;

        default:
            return false;
            break;
    }
}
static int axi_dsd_set_hw_flags(struct axi_dsd *dsd,bool immed)
{
    regmap_write(dsd->regmap_controller, AXI_DSD_REG_CHANNELS, dsd->nchan);
    regmap_write(dsd->regmap_controller, AXI_DSD_REG_CLKDIV, dsd->bclk_div);
    regmap_write(dsd->regmap_controller, AXI_DSD_REG_RATE, dsd->bclk_rate);
    regmap_write(dsd->regmap_controller, AXI_DSD_REG_CHANNELS, dsd->nchan);
    return regmap_write(dsd->regmap_controller, AXI_DSD_REG_FLAGS, get_controller_flags(dsd,immed) );
}
static int axi_dsd_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct axi_dsd *dsd = snd_soc_dai_get_drvdata(dai);
	unsigned int val;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
        axi_dsd_set_hw_flags(dsd, true);
        regmap_update_bits(dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_RS, DMACR_RS);
            val = 0;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
        regmap_update_bits(dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_Reset, DMACR_Reset);
		val = 0;
		break;
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            regmap_update_bits( dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_RS, DMACR_RS);
            val = 0;
            break;
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
            regmap_update_bits( dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_RS, 0);
            val = 0;
            break;
	default:
		return -EINVAL;
	}

	return val;
}


static bool axi_dsd_clk_fam(uint rate)
{
    switch (rate) {
    case 44100 :
    case 88200 :
    case 176400 :
    case 352800 :
    case 705600 :
    case 1411200 :
        return false;
            break;
    case 48000 :
    case 96000 :
    case 192000 :
    case 384000 :
    case 768000 :
    case 1536000 :
        return true;
            break;
    default:
            printk("AXI_DSD:axi_dsd_clk_fam: Unknown rate: %i/n",rate);
            return false;
            
    }

}

static int axi_dsd_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct axi_dsd *dsd = snd_soc_dai_get_drvdata(dai);
	unsigned int bclk_div, rate;
	unsigned int bclk_rate;
/**
 so ... wait for transfers to be done, then reset device and reset hardware ...
**/
    axi_dsd_wait_dma_done(dsd,1000,100000);
    dsd->is_dsd = is_dsd(params_format(params));
    dsd->nchan = params_channels(params);
    dsd->bclk_rate = params_rate(params) * AXI_DSD_FRAME_WIDTH;
    
    dsd->clk_fam = axi_dsd_clk_fam(params_rate(params));
    
    if (dsd->clk_fam)
        rate = dsd->ratnum[0].num;
    else
        rate = dsd->ratnum[1].num;
    
    dsd->bclk_div = rate / dsd->bclk_rate;
    

	//word_size = AXI_DSD_BITS_PER_FRAME / 2 - 1;
	// bclk_div = DIV_ROUND_UP(clk_get_rate(dsd->clk_ref), bclk_rate) / 2 - 1;


	axi_dsd_set_hw_flags(dsd, true);

	return 0;
}
#ifdef DEF_DSD_DAI_SET_CLKDIV
static int axi_dsd_dai_set_clkdiv(struct snd_soc_dai *cpu_dai,
                                      int div_id, int div)
{
    struct axi_dsd *dev = snd_soc_dai_get_drvdata(cpu_dai);
    
    if (div_id != DAVINCI_MCBSP_CLKGDV)
        return -ENODEV;
    
    dev->bclk_div = div;
    return 0;
}
#endif
static const unsigned int axi_dsd_dai_rates[] = {
    44100, 48000,
    88200, 96000,
    176400, 192000,
    352800, 384000,
    705600, 768000,
    1411200, 1536000
};

#define AXI_DSD_MAX_RATE_ERROR_PPM 1000
static int axi_dsd_calc_clk_div(struct axi_dsd *dsd,
                                      unsigned int bclk_freq,
                                      int *error_ppm)
{
    int div; //= mcasp->sysclk_freq / bclk_freq;
    int rem; //= mcasp->sysclk_freq % bclk_freq;
    int iclk = axi_dsd_clk_fam(bclk_freq) ? 1 : 0;

    div = dsd->ratnum[iclk].num / bclk_freq;
    rem = dsd->ratnum[iclk].num % bclk_freq;
                              
    if (rem != 0) {
        if (div == 0 ||
            ((dsd->ratnum[iclk].num / div) - bclk_freq) >
            (bclk_freq - (dsd->ratnum[iclk].num / (div+1)))) {
            div++;
            rem = rem - bclk_freq;
        }
    }
    if (error_ppm)
        *error_ppm =
        (div*1000000 + (int)div64_long(1000000LL*rem,
                                       (int)bclk_freq))
        /div - 1000000;
    
    return div;
}
static int axi_dsd_hw_rule_rate(struct snd_pcm_hw_params *params,
                                      struct snd_pcm_hw_rule *rule)
{
    struct axi_dsd_ruledata *rd = rule->private;
    struct snd_interval *ri = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
    // int sbits = params_width(params);
    // int slots = rd->dsd->tdm_slots;
    struct snd_interval range;
    int i;
    
    /* if (rd->dsd->slot_width)
        sbits = rd->dsd->slot_width;
    */
    snd_interval_any(&range);
    range.empty = 1;
    
    for (i = 0; i < ARRAY_SIZE(axi_dsd_dai_rates); i++) {
        if (snd_interval_test(ri, axi_dsd_dai_rates[i])) {
            uint bclk_freq = AXI_DSD_FRAME_WIDTH * axi_dsd_channels(rd->dsd)  * axi_dsd_dai_rates[i];
            int ppm;
            
            axi_dsd_calc_clk_div(rd->dsd, bclk_freq, &ppm);
            if (abs(ppm) < AXI_DSD_MAX_RATE_ERROR_PPM) {
                if (range.empty) {
                    range.min = axi_dsd_dai_rates[i];
                    range.empty = 0;
                }
                range.max = axi_dsd_dai_rates[i];
            }
        }
    }
    
    dev_dbg(rd->dsd->dev,
            "Frequencies %d-%d -> %d-%d for %d bits and %d channels\n",
            ri->min, ri->max, range.min, range.max, AXI_DSD_FRAME_WIDTH, axi_dsd_channels(rd->dsd));
    
    return snd_interval_refine(hw_param_interval(params, rule->var),
                               &range);
}
#ifdef DEF_HW_RULE_FMT
static int axi_dsd_hw_rule_format(struct snd_pcm_hw_params *params,
                                        struct snd_pcm_hw_rule *rule)
{
    struct axi_dsd_ruledata *rd = rule->private;
    struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
    struct snd_mask nfmt;
    int rate = params_rate(params);
    //int slots = rd->mcasp->tdm_slots;
    int i, count = 0;
    
    snd_mask_none(&nfmt);
    
    for (i = 0; i < SNDRV_PCM_FORMAT_LAST; i++) {
        if (snd_mask_test(fmt, i)) {
            uint sbits = snd_pcm_format_width(i);
            uint bclk_freq = AXI_DSD_FRAME_WIDTH * axi_dsd_channels(rd->dsd)  * axi_dsd_dai_rates[i];
            int ppm;
            
           /*  if (rd->mcasp->slot_width)
                sbits = rd->mcasp->slot_width;
            */
            axi_dsd_calc_clk_div(rd->dsd, bclk_freq,
                                       &ppm);
            if (abs(ppm) < AXI_DSD_MAX_RATE_ERROR_PPM) {
                snd_mask_set(&nfmt, i);
                count++;
            }
        }
    }
    dev_dbg(rd->dsd->dev,
            "%d possible sample format for %d Hz\n",
            count, rate);
    
    return snd_mask_refine(fmt, &nfmt);
}
#endif


static int axi_dsd_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct axi_dsd *dsd = snd_soc_dai_get_drvdata(dai);
    struct axi_dsd_ruledata *ruledata = &dsd->ruledata[substream->stream];
    
	int ret;
    
    dsd->substream = substream;
    
    if (dsd->bclk_div == 0) {
        
        ruledata->dsd = dsd;
        
        ret = snd_pcm_hw_rule_add(substream->runtime, 0,
                                  SNDRV_PCM_HW_PARAM_RATE,
                                  axi_dsd_hw_rule_rate,
                                  ruledata,
                                  SNDRV_PCM_HW_PARAM_FORMAT, -1);
        if (ret)
            return ret;
    }

    
    ret = snd_pcm_hw_constraint_ratnums(substream->runtime, 0,
                                        SNDRV_PCM_HW_PARAM_RATE,
                                        &dsd->rate_constraints);
    if (ret)
        return ret;
    
    ret = clk_prepare_enable(dsd->clkA);
    ret = clk_prepare_enable(dsd->clkB);

    regmap_update_bits(dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_Reset, DMACR_Reset);
    regmap_async_complete(dsd->regmap_dma);
    axi_dsd_set_hw_flags(dsd, true);
    regmap_update_bits(dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_RS, DMACR_RS);

    return ret;
}

static void axi_dsd_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct axi_dsd *dsd = snd_soc_dai_get_drvdata(dai);

	clk_disable_unprepare(dsd->clkA);
    clk_disable_unprepare(dsd->clkB);
}

static int axi_dsd_dai_probe(struct snd_soc_dai *dai)
{
	struct axi_dsd *dsd = snd_soc_dai_get_drvdata(dai);
 //   uint dmaacr_flags,dmaasr_flags;


	snd_soc_dai_init_dma_data(dai, &dsd->playback_dma_data, NULL);

	return 0;
}

static const struct snd_soc_dai_ops axi_dsd_dai_ops = {
	.startup = axi_dsd_startup,
	.shutdown = axi_dsd_shutdown,
	.trigger = axi_dsd_trigger,
	.hw_params = axi_dsd_hw_params,
};

static struct snd_soc_dai_driver axi_dsd_dai = {
	.probe = axi_dsd_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = AXI_DSD_MAX_CHANNELS,
		.rates = AXI_DSD_PCM_RATES,
		.formats = AXI_DSD_PCM_FMTS,
	},
	.ops = &axi_dsd_dai_ops,
	.symmetric_rates = 1,
};

static const struct snd_soc_component_driver axi_dsd_component = {
	.name = "axi-dsd",
};

static const struct regmap_config axi_dsd_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = AXI_DSD_REG_MAX,
};
static const struct regmap_config axi_dma_regmap_config = {
    .reg_bits = 32,
    .reg_stride = 4,
    .val_bits = 32,
    .max_register = AXI_DMA_REG_MAX,
};

static int axi_dsd_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct resource *r_irq;
	struct axi_dsd *dsd;
	struct platform_device *pdev_dma;
	void __iomem *base;
	int ret;


	dsd = devm_kzalloc(&pdev->dev, sizeof(*dsd), GFP_KERNEL);
	if (!dsd)
		return -ENOMEM;

	platform_set_drvdata(pdev, dsd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
    
    dsd->clkA = devm_clk_get(&pdev->dev, "clkA");
    if (IS_ERR(dsd->clkA)) {
        ret = PTR_ERR(dsd->clkA);
        dev_dbg(&pdev->dev, "unable to get clkA clock, err %d\n", ret);
        return ret;
    }

    dsd->ratnum[0].num = clk_get_rate(dsd->clkA);
    dsd->ratnum[0].den_min = CLK_DIV_MIN;
    dsd->ratnum[0].den_max = CLK_DIV_MAX;
    dsd->ratnum[0].den_step = 1;
    dsd->clkB = devm_clk_get(&pdev->dev, "clkB");
    if (IS_ERR(dsd->clkB)) {
        ret = PTR_ERR(dsd->clkB);
        dev_dbg(&pdev->dev, "unable to get clkB clock, err %d\n", ret);
        return ret;
    }

    dsd->ratnum[1].num = clk_get_rate(dsd->clkB);
    dsd->ratnum[1].den_min = 1;
    dsd->ratnum[1].den_max = 128;
    dsd->ratnum[1].den_step = 1;
    
    dsd->rate_constraints.nrats = 2;
    dsd->rate_constraints.rats = &dsd->ratnum[0];
    
    dsd->regmap_controller = devm_regmap_init_mmio(&pdev->dev, base,
		&axi_dsd_regmap_config);
	if (IS_ERR(dsd->regmap_controller))
		return PTR_ERR(dsd->regmap_controller);
    
    pdev_dma = platform_device_register_simple("axi-dma", -1, NULL, 0);

    
    res = platform_get_resource(pdev_dma, IORESOURCE_MEM,0);
    base = devm_ioremap_resource(&pdev_dma->dev,res);
    if (IS_ERR(base))
        return PTR_ERR(base);
    
    dsd->regmap_dma = devm_regmap_init_mmio(&pdev_dma->dev, base,
                                               &axi_dma_regmap_config);
    
    dsd->dev = &pdev->dev;
    dsd->dev_dma = &pdev_dma->dev;

	dsd->playback_dma_data.addr = res->start + AXI_DMA_MM2S_SA;
	dsd->playback_dma_data.addr_width = 4;
	dsd->playback_dma_data.maxburst = 1;

    dev_dbg(&pdev->dev,"Clock A rate %d\n", dsd->ratnum[0].num);
    dev_dbg(&pdev->dev,"Clock B rate %d\n", dsd->ratnum[1].num);


	regmap_write(dsd->regmap_dma, AXI_DMA_MM2S_DMACR, DMACR_Reset);

	ret = devm_snd_soc_register_component(&pdev->dev, &axi_dsd_component,
					 &axi_dsd_dai, 1);
	if (ret)
		goto err_clk_disable;

	ret = devm_snd_dmaengine_pcm_register(&pdev_dma->dev, NULL, 0);
	if (ret)
		goto err_clk_disable;

	/* Get IRQ for the device */
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r_irq) {
		dev_info(dev, "no IRQ found\n");
		dev_info(dev, "axi-dsd at 0x%08x mapped to 0x%08x\n",
			(unsigned int __force)res,
			(unsigned int __force)base);
		return 0;
	}
	dsd->irq = r_irq->start;
	ret = request_irq(dsd->irq, &axi_dsd_irq, 0, DRIVER_NAME, dsd);
	if (ret) {
		dev_err(dev, "axi-dsd: Could not allocate interrupt %d.\n",
			dsd->irq);
		free_irq(r_irq,dsd);
		goto err_clk_disable;
	}

	return 0;

err_clk_disable:
	// clk_disable_unprepare(dsd->clkA);
	// clk_disable_unprepare(dsd->clkB);
    return ret;
}

static int axi_dsd_dev_remove(struct platform_device *pdev)
{
    struct axi_dsd *dsd = platform_get_drvdata(pdev);
    
    clk_disable_unprepare(dsd->clkA);
    clk_disable_unprepare(dsd->clkB);
	struct device *dev = &pdev->dev;
	free_irq(dsd->irq, dsd);
	kfree(dsd);
	dev_set_drvdata(dev, NULL);
	return 0;
    return 0;
}
static const struct of_device_id axi_dsd_of_match[] = {
	{ .compatible = "jabr,axi-dsd-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, axi_dsd_of_match);

static struct platform_driver axi_dsd_driver = {
	.driver = {
		.name = "axi-dsd",
		.of_match_table = axi_dsd_of_match,
	},
	.probe = axi_dsd_probe,
	.remove = axi_dsd_dev_remove,
};
static int __init axi_dsd_init(void)
{
	printk("<1>Hello AXI-DSD\n");
	printk("<1>Module parameters were (0x%08x) and \"%s\"\n", myint,
	       mystr);

	return platform_driver_register(&axi_dsd_driver);
}


static void __exit axi_dsd_exit(void)
{
	platform_driver_unregister(&axi_dsd_driver);
	printk(KERN_ALERT "Goodbye module world.\n");
}

module_init(axi_dsd_init);
module_exit(axi_dsd_exit);
module_platform_driver(axi_dsd_driver);



MODULE_DEVICE_TABLE(of, axi_dsd_of_match);



MODULE_AUTHOR("Jonathan Borden <jonathan@jabresearch.com>");
MODULE_DESCRIPTION("AXI DSD driver");
MODULE_LICENSE("GPL");
