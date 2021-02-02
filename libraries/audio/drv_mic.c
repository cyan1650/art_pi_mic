/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-22     cyan1650       the first version
 */

#include "board.h"
#include "wm8988.h"
#include "drv_sound.h"
#include <rtthread.h>

#define DBG_TAG              "drv.mic"
#define DBG_LVL              DBG_INFO
#include <rtdbg.h>

#define RX_FIFO_SIZE (2048)

struct mic_device
{
    struct rt_audio_device audio;
    struct rt_audio_configure record_config;
    rt_uint8_t *rx_fifo;
};

static struct mic_device mic_dev = {0};

static SAI_HandleTypeDef SAI2B_Handler = {0};
static DMA_HandleTypeDef SAI2_RXDMA_Handler = {0};


extern void SAIA_Frequency_Set(uint32_t frequency);

void SAIB_Init(void)
{
    __HAL_RCC_SAI2_CLK_ENABLE();

    HAL_SAI_DeInit(&SAI2B_Handler);

    SAI2B_Handler.Instance = SAI2_Block_B;
    SAI2B_Handler.Init.AudioMode = SAI_MODESLAVE_RX;
    SAI2B_Handler.Init.Synchro = SAI_SYNCHRONOUS;
    SAI2B_Handler.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    SAI2B_Handler.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    SAI2B_Handler.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    SAI2B_Handler.Init.MonoStereoMode = SAI_STEREOMODE;
    SAI2B_Handler.Init.CompandingMode = SAI_NOCOMPANDING;
    SAI2B_Handler.Init.TriState = SAI_OUTPUT_NOTRELEASED;


    if (HAL_SAI_InitProtocol(&SAI2B_Handler, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
    {

    }

    __HAL_RCC_DMA2_CLK_ENABLE();

    SAI2_RXDMA_Handler.Instance = DMA2_Stream4;
    SAI2_RXDMA_Handler.Init.Request = DMA_REQUEST_SAI2_B;
    SAI2_RXDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    SAI2_RXDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    SAI2_RXDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    SAI2_RXDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    SAI2_RXDMA_Handler.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
    SAI2_RXDMA_Handler.Init.Mode = DMA_CIRCULAR;
    SAI2_RXDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    SAI2_RXDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    __HAL_LINKDMA(&SAI2B_Handler,hdmatx,SAI2_RXDMA_Handler);
    __HAL_LINKDMA(&SAI2B_Handler,hdmarx,SAI2_RXDMA_Handler);

    HAL_DMA_DeInit(&SAI2B_Handler);

    if (HAL_DMA_Init(&SAI2_RXDMA_Handler) != HAL_OK)
    {

    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */

    //__HAL_LINKDMA(&SAI2B_Handler,hdmatx,SAI2_RXDMA_Handler);

    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void SAIB_Channels_Set(uint8_t channels)
{
    if (channels == 1)
    {
        SAI2B_Handler.Init.MonoStereoMode         = SAI_MONOMODE;
    }
    else
    {
        SAI2B_Handler.Init.MonoStereoMode         = SAI_STEREOMODE;
    }

    __HAL_SAI_DISABLE(&SAI2B_Handler);
    HAL_SAI_Init(&SAI2B_Handler);
    __HAL_SAI_ENABLE(&SAI2B_Handler);
}

void DMA2_Stream4_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&SAI2_RXDMA_Handler);
    rt_interrupt_leave();
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    rt_audio_rx_done(&mic_dev.audio, &mic_dev.rx_fifo[0], RX_FIFO_SIZE / 2);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    rt_audio_rx_done(&mic_dev.audio, &mic_dev.rx_fifo[RX_FIFO_SIZE / 2], RX_FIFO_SIZE / 2);
}

static rt_err_t mic_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct mic_device *mic_dev;

    RT_ASSERT(audio != RT_NULL);
    mic_dev = (struct mic_device *)audio->parent.user_data;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_QUERY: /* qurey the types of hw_codec device */
    {
        switch (caps->sub_type)
        {
        case AUDIO_TYPE_QUERY:
            caps->udata.mask = AUDIO_TYPE_INPUT;
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    case AUDIO_TYPE_INPUT: /* Provide capabilities of INPUT unit */
    {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
            caps->udata.config.samplerate   = mic_dev->record_config.samplerate;
            caps->udata.config.channels     = mic_dev->record_config.channels;
            caps->udata.config.samplebits   = mic_dev->record_config.samplebits;
            break;

        case AUDIO_DSP_SAMPLERATE:
            caps->udata.config.samplerate   = mic_dev->record_config.samplerate;
            break;

        case AUDIO_DSP_CHANNELS:
            caps->udata.config.channels     = mic_dev->record_config.channels;
            break;

        case AUDIO_DSP_SAMPLEBITS:
            caps->udata.config.samplebits   = mic_dev->record_config.samplebits;
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }


    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

static rt_err_t mic_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct mic_device *mic_dev;

    RT_ASSERT(audio != RT_NULL);
    mic_dev = (struct mic_device *)audio->parent.user_data;

    switch (caps->main_type)
    {

    case AUDIO_TYPE_INPUT:
    {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
        {
            HAL_SAI_Receive_DMA(&SAI2B_Handler, mic_dev->rx_fifo, RX_FIFO_SIZE / 2);

            /* save configs */
            mic_dev->record_config.samplerate = caps->udata.config.samplerate;
            mic_dev->record_config.channels   = caps->udata.config.channels;
            mic_dev->record_config.samplebits = caps->udata.config.samplebits;
            LOG_D("set samplerate %d", mic_dev->record_config.samplerate);
            LOG_D("set channels %d", mic_dev->record_config.channels);
            break;
        }

        case AUDIO_DSP_SAMPLERATE:
        {
            mic_dev->record_config.samplerate = caps->udata.config.samplerate;
            LOG_D("set channels %d", mic_dev->record_config.channels);
            break;
        }

        case AUDIO_DSP_CHANNELS:
        {
            mic_dev->record_config.channels   = caps->udata.config.channels;
            LOG_D("set channels %d", mic_dev->record_config.channels);
            break;
        }

        default:
            break;
        }

        break;
    }

    default:
        break;
    }

    return result;
}

static rt_err_t mic_init(struct rt_audio_device *audio)
{
    struct mic_device *mic_dev;

    RT_ASSERT(audio != RT_NULL);
    mic_dev = (struct mic_device *)audio->parent.user_data;

    SAIB_Init();

    /* set default params */
    SAIB_Channels_Set(mic_dev->record_config.channels);

    return RT_EOK;
}

static rt_err_t mic_start(struct rt_audio_device *audio, int stream)
{
    struct mic_device *mic_dev;

    RT_ASSERT(audio != RT_NULL);
    mic_dev = (struct mic_device *)audio->parent.user_data;

    if (stream == AUDIO_STREAM_RECORD)
    {
        HAL_SAI_Receive_DMA(&SAI2B_Handler, mic_dev->rx_fifo, RX_FIFO_SIZE / 2);
    }

    return RT_EOK;
}

static rt_err_t mic_stop(struct rt_audio_device *audio, int stream)
{
    struct mic_device *mic_dev;

    RT_ASSERT(audio != RT_NULL);
    mic_dev = (struct mic_device *)audio->parent.user_data;

    if (stream == AUDIO_STREAM_RECORD)
    {
        HAL_SAI_DMAStop(&SAI2B_Handler);
    }

    return RT_EOK;
}

static struct rt_audio_ops mic_ops =
{
    .getcaps     = mic_getcaps,
    .configure   = mic_configure,
    .init        = mic_init,
    .start       = mic_start,
    .stop        = mic_stop,
    .transmit    = RT_NULL,
    .buffer_info = RT_NULL,
};

int rt_hw_mic_init(void)
{
    rt_uint8_t *rx_fifo;

    if (mic_dev.rx_fifo)
        return RT_EOK;

    rx_fifo = rt_malloc(RX_FIFO_SIZE*2);
    if (rx_fifo == RT_NULL)
        return -RT_ENOMEM;
    rt_memset(rx_fifo, 0, RX_FIFO_SIZE);
    mic_dev.rx_fifo = rx_fifo;

    /* init default configuration */
    {
        mic_dev.record_config.samplerate = 8000;
        mic_dev.record_config.channels   = 2;
        mic_dev.record_config.samplebits = 16;
    }

    /* register sound device */
    mic_dev.audio.ops = &mic_ops;
    rt_audio_register(&mic_dev.audio, "mic0", RT_DEVICE_FLAG_RDONLY, &mic_dev);

    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_mic_init);
