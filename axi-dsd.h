/*
 * ALSA SoC Zynq
 *
 *
 * Author: Jonathan Borden <jonathan@jabresearch.com>
 *
 * Copyright:   (C) 2017 JAB Reserch LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AXI_DSD_H
#define AXI_DSD_H

#define CLKA_RATE 45158000
#define CLKB_RATE 49152000
#define AXI_DSD_FRAME_WIDTH 32

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
Transmit ID Register(4) C_BASEADDR + x34 Write
Transmit USER Register(4) C_BASEADDR + x38 Write
Receive ID Register(4) C_BASEADDR + x3C Read
Receive USER Register(4) C_BASEADDR + x40 Read

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

#define DMACR_RS BIT(0)
#define DMACR_RSVD BIT(1)
#define DMACR_Reset BIT(2)
#define DMACR_Keyhole BIT(3)
#define DMACR_Cyclic_BD_Enable BIT(4)
#define DMACR_IOC_IrqEn BIT(12)
#define DMACR_Dly_IrqEn BIT(13)
#define DMACR_Err_IrqEn BIT(14)
#define DMACR_IRQThreshold BIT(16)
#define DMACR_IRQDelay BIT(24)

#define DMASR_Halted BIT(0)
#define DMASR_Idle BIT(1)
#define DMASR_SGIncld BIT(3)
#define DMASR_DMAIntErr BIT(4)
#define DMASR_DMASlvErr BIT(5)
#define DMASR_DMADecErr BIT(6)
#define DMASR_SGIntErr BIT(8)
#define DMASR_SGSlvErr BIT(9)
#define DMASR_SGDecErr BIT(10)


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

#define AXI_DSD_MAX_CHANNELS 16

#define JBDAC_AXI_CONTROLLER 0x43C00000
#define JBDAC_AXI_DMA        0X40400000


#endif	/* AXI_DSD_H */
