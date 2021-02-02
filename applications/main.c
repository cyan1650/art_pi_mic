/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-09-02     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
#include "drv_sound.h"

#define LED_PIN GET_PIN(I, 8)
#define AUDIO_DEVICE_NAME      "sound0"
#define MIC_DEVICE_NAME        "mic0"
#define BUFF_SIZE              512

rt_thread_t record_thread = NULL;
rt_device_t play_audio_device = NULL;
rt_device_t record_device = NULL;

rt_uint8_t read_buffer[BUFF_SIZE];
rt_uint8_t write_buffer[BUFF_SIZE] = { 0 };
static struct rt_audio_caps playback_param = {0, 0, 0};

int record_task(void)
{

    rt_err_t ret;

    /* open Speak */
    play_audio_device = rt_device_find(AUDIO_DEVICE_NAME);
    ret = rt_device_open(play_audio_device, RT_DEVICE_OFLAG_WRONLY);
    RT_ASSERT(ret == RT_EOK);

    /* open MIC */
    record_device = rt_device_find(MIC_DEVICE_NAME);
    ret = rt_device_open(record_device, RT_DEVICE_OFLAG_RDONLY);
    RT_ASSERT(ret == RT_EOK);

    playback_param.main_type = AUDIO_TYPE_OUTPUT;
    playback_param.sub_type = AUDIO_DSP_PARAM;
    playback_param.udata.config.channels = 2;
    playback_param.udata.config.samplebits = 16;

    playback_param.udata.config.samplerate = AUDIO_FREQUENCY_016K; //16000
    ret = rt_device_control(play_audio_device, AUDIO_CTL_CONFIGURE, &playback_param);

    /* dummy byte */
    rt_device_write(play_audio_device, 0, write_buffer, BUFF_SIZE);

    while (1)
    {
        rt_device_read(record_device, 0, read_buffer, BUFF_SIZE);
        rt_memcpy(write_buffer, read_buffer, BUFF_SIZE);
        rt_device_write(play_audio_device, 0, write_buffer, BUFF_SIZE);
    }
}

int main(void)
{
    rt_uint32_t count = 1;

    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    record_thread = rt_thread_create("record_task", &record_task, (void*)RT_NULL, 10240, 20, 0);
    rt_thread_startup(record_thread);

    while(count++)
    {
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
    }
    return RT_EOK;
}

#include "stm32h7xx.h"
static int vtor_config(void)
{
    /* Vector Table Relocation in Internal QSPI_FLASH */
    SCB->VTOR = QSPI_BASE;
    return 0;
}
INIT_BOARD_EXPORT(vtor_config);


