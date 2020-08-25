/****************************************************************************
 * drivers/audio/audio_dma.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Zhong An <zhongan@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

#include <nuttx/audio/audio_dma.h>
#include <nuttx/kmalloc.h>
#include <queue.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct audio_dma_s
{
  struct audio_lowerhalf_s dev;
  struct dma_chan_s *chan;
  uintptr_t src_addr;
  uintptr_t dst_addr;
  uint8_t *alloc_addr;
  uint8_t alloc_index;
  uint8_t fifo_width;
  bool playback;
  bool xrun;
  struct dq_queue_s pendq;
  apb_samp_t buffer_size;
  apb_samp_t buffer_num;
  int xrun_times;
  struct timespec xrun_ts;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_dma_getcaps(struct audio_lowerhalf_s *dev, int type,
                             struct audio_caps_s *caps);
static int audio_dma_shutdown(struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               void *session,
                               const struct audio_caps_s *caps);
static int audio_dma_start(struct audio_lowerhalf_s *dev,
                           void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_dma_stop(struct audio_lowerhalf_s *dev, void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_dma_pause(struct audio_lowerhalf_s *dev,
                           void *session);
static int audio_dma_resume(struct audio_lowerhalf_s *dev,
                            void *session);
#endif
static int audio_dma_reserve(struct audio_lowerhalf_s *dev,
                             void **session);
static int audio_dma_release(struct audio_lowerhalf_s *dev,
                             void *session);
#else
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               const struct audio_caps_s *caps);
static int audio_dma_start(struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_dma_stop(struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_dma_pause(struct audio_lowerhalf_s *dev);
static int audio_dma_resume(struct audio_lowerhalf_s *dev);
#endif
static int audio_dma_reserve(struct audio_lowerhalf_s *dev);
static int audio_dma_release(struct audio_lowerhalf_s *dev);
#endif
static int audio_dma_allocbuffer(struct audio_lowerhalf_s *dev,
                                 struct audio_buf_desc_s *bufdesc);
static int audio_dma_freebuffer(struct audio_lowerhalf_s *dev,
                                struct audio_buf_desc_s *bufdesc);
static int audio_dma_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                   struct ap_buffer_s *apb);
static int audio_dma_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg);
static void audio_dma_callback(struct dma_chan_s *chan, void *arg,
                               ssize_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audio_dma_ops =
{
  .getcaps = audio_dma_getcaps,
  .configure = audio_dma_configure,
  .shutdown = audio_dma_shutdown,
  .start = audio_dma_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop = audio_dma_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause = audio_dma_pause,
  .resume = audio_dma_resume,
#endif
  .allocbuffer = audio_dma_allocbuffer,
  .freebuffer = audio_dma_freebuffer,
  .enqueuebuffer = audio_dma_enqueuebuffer,
  .ioctl = audio_dma_ioctl,
  .reserve = audio_dma_reserve,
  .release = audio_dma_release,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int audio_dma_getcaps(struct audio_lowerhalf_s *dev, int type,
                             struct audio_caps_s *caps)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              if (audio_dma->playback)
                caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;
              else
                caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
              caps->ac_format.hw = 1 << (AUDIO_FMT_PCM - 1);
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

        /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

            /* Report the Sample rates we support */

              caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K  | AUDIO_SAMP_RATE_11K |
                                        AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K |
                                        AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K |
                                        AUDIO_SAMP_RATE_48K | AUDIO_SAMP_RATE_96K |
                                        AUDIO_SAMP_RATE_128K | AUDIO_SAMP_RATE_160K |
                                        AUDIO_SAMP_RATE_172K | AUDIO_SAMP_RATE_192K;
              break;
          }

        break;
   }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  return caps->ac_len;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               void *session,
                               const struct audio_caps_s *caps)
#else
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               const struct audio_caps_s *caps)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct dma_config_s cfg;
  int ret = -EINVAL;

  DEBUGASSERT(audio_dma && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
        if (audio_dma->playback)
          {
            memset(&cfg, 0, sizeof(struct dma_config_s));
            cfg.direction = DMA_MEM_TO_DEV;
            if (audio_dma->fifo_width)
              cfg.dst_width = audio_dma->fifo_width;
            else
              cfg.dst_width = caps->ac_controls.b[2] / 8;
            ret = DMA_CONFIG(audio_dma->chan, &cfg);
          }
        break;
      case AUDIO_TYPE_INPUT:
        if (!audio_dma->playback)
          {
            memset(&cfg, 0, sizeof(struct dma_config_s));
            cfg.direction = DMA_DEV_TO_MEM;
            if (audio_dma->fifo_width)
              cfg.src_width = audio_dma->fifo_width;
            else
              cfg.src_width = caps->ac_controls.b[2] / 8;
            ret = DMA_CONFIG(audio_dma->chan, &cfg);
          }
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int audio_dma_shutdown(struct audio_lowerhalf_s *dev)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  audio_dma_stop(dev);
  kumm_free(audio_dma->alloc_addr);
  audio_dma->alloc_addr = NULL;
  audio_dma->alloc_index = 0;

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_start(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_start(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  if (audio_dma->playback)
    {
      audio_dma->xrun_times = 0;
    }

  return DMA_START_CYCLIC(audio_dma->chan, audio_dma_callback, audio_dma,
                          audio_dma->dst_addr, audio_dma->src_addr,
                          audio_dma->buffer_num * audio_dma->buffer_size,
                          audio_dma->buffer_size);
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_stop(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_stop(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_s *apb;

  DMA_STOP(audio_dma->chan);

  while(!dq_empty(&audio_dma->pendq))
    {
      apb = (struct ap_buffer_s*)dq_remfirst(&audio_dma->pendq);
#ifdef CONFIG_AUDIO_MULTI_SESSION
      audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                           apb, OK, NULL);
#else
      audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                           apb, OK);
#endif
    }
#ifdef CONFIG_AUDIO_MULTI_SESSION
  audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif
  audio_dma->xrun = false;

  if (audio_dma->playback)
    {
      audio_dma->xrun_times = 0;
    }

  return OK;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_pause(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_pause(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  return DMA_PAUSE(audio_dma->chan);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_resume(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_resume(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  if (dq_empty(&audio_dma->pendq))
    return -EINVAL;
  return DMA_RESUME(audio_dma->chan);
}
#endif

static int audio_dma_allocbuffer(struct audio_lowerhalf_s *dev,
                                 struct audio_buf_desc_s *bufdesc)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_s *apb;

  if (bufdesc->numbytes != audio_dma->buffer_size)
    return -EINVAL;

  if (audio_dma->alloc_index == audio_dma->buffer_num)
    return -ENOMEM;

  if (!audio_dma->alloc_addr)
    {
      audio_dma->alloc_addr = kumm_memalign(32,
                                            audio_dma->buffer_num *
                                            audio_dma->buffer_size);
      if (!audio_dma->alloc_addr)
        return -ENOMEM;

      if (audio_dma->playback)
        audio_dma->src_addr = up_addrenv_va_to_pa(audio_dma->alloc_addr);
      else
        audio_dma->dst_addr = up_addrenv_va_to_pa(audio_dma->alloc_addr);
    }
  apb = kumm_zalloc(sizeof(struct ap_buffer_s));
  *bufdesc->u.ppBuffer = apb;
  /* Test if the allocation was successful or not */
  if (*bufdesc->u.ppBuffer == NULL)
    return -ENOMEM;
  /* Populate the buffer contents */
  apb->i.channels = 2;
  apb->crefs      = 1;
  apb->nmaxbytes  = audio_dma->buffer_size;
  apb->samp = audio_dma->alloc_addr +
              audio_dma->alloc_index *
              audio_dma->buffer_size;
  audio_dma->alloc_index++;
  nxsem_init(&apb->sem, 0, 1);
  return sizeof(struct audio_buf_desc_s);
}

static int audio_dma_freebuffer(struct audio_lowerhalf_s *dev,
                                struct audio_buf_desc_s *bufdesc)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_s *apb;

  apb = bufdesc->u.pBuffer;
  audio_dma->alloc_index--;
  nxsem_destroy(&apb->sem);
  kumm_free(apb);

  return sizeof(struct audio_buf_desc_s);
}

static int audio_dma_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                   struct ap_buffer_s *apb)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  irqstate_t flags;

  if (audio_dma->playback)
    up_clean_dcache((uintptr_t)apb->samp,
                    (uintptr_t)apb->samp + apb->nbytes);

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;

  flags = enter_critical_section();
  dq_addlast(&apb->dq_entry, &audio_dma->pendq);
  leave_critical_section(flags);

  if (audio_dma->xrun)
    {
      if (audio_dma->playback)
        {
          struct timespec ts, delta;
          clock_gettime(CLOCK_MONOTONIC, &ts);
          clock_timespec_subtract(&ts, &audio_dma->xrun_ts, &delta);
          printf("!!!!! underflow (%d, %dus) !!!! \n", audio_dma->xrun_times,
                                     (delta.tv_sec * 10000000 + delta.tv_nsec / 1000));
        }
      audio_dma->xrun = false;
      return audio_dma_resume(dev);
    }

  return OK;
}

static int audio_dma_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_info_s *bufinfo;
  uint32_t *times;

  switch (cmd)
    {
      /* Report our preferred buffer size and quantity */

      case AUDIOIOC_GETBUFFERINFO:
        audinfo("AUDIOIOC_GETBUFFERINFO:\n");
        bufinfo              = (struct ap_buffer_info_s *)arg;
        bufinfo->buffer_size = audio_dma->buffer_size;
        bufinfo->nbuffers    = audio_dma->buffer_num;
        return OK;
      case AUDIOIOC_SETBUFFERINFO:
        audinfo("AUDIOIOC_GETBUFFERINFO:\n");
        bufinfo                = (struct ap_buffer_info_s *)arg;
        audio_dma->buffer_size = bufinfo->buffer_size;
        audio_dma->buffer_num  = bufinfo->nbuffers;
        kumm_free(audio_dma->alloc_addr);
        audio_dma->alloc_addr = NULL;
        audio_dma->alloc_index = 0;
        return OK;
      case AUDIOIOC_GETUNDERFLOW:
        times = (uint32_t *)arg;
        if (audio_dma->playback)
          {
            *times = audio_dma->xrun_times;
          }
        return OK;
    }
  return -ENOTTY;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_reserve(struct audio_lowerhalf_s *dev, void **session)
#else
static int audio_dma_reserve(struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_release(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_release(struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

static void audio_dma_callback(struct dma_chan_s *chan, void *arg, ssize_t len)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)arg;
  struct ap_buffer_s *apb;
  bool final = false;

  apb = (struct ap_buffer_s*)dq_remfirst(&audio_dma->pendq);
  if (!apb)
    {
      /* xrun */
      DMA_PAUSE(audio_dma->chan);
      audio_dma->xrun = true;
      return;
    }

  if (!audio_dma->playback)
    up_invalidate_dcache((uintptr_t)apb->samp,
                         (uintptr_t)apb->samp + apb->nbytes);

  if ((apb->flags & AUDIO_APB_FINAL) != 0)
    final = true;

#ifdef CONFIG_AUDIO_MULTI_SESSION
    audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
    audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
  if (final)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      audio_dma_stop(&audio_dma->dev, NULL);
#else
      audio_dma_stop(&audio_dma->dev);
#endif
    }
  else if (dq_empty(&audio_dma->pendq))
    {
      /* xrun */
      DMA_PAUSE(audio_dma->chan);
      audio_dma->xrun = true;
      if (audio_dma->playback)
        {
          audio_dma->xrun_times++;
          clock_gettime(CLOCK_MONOTONIC, &audio_dma->xrun_ts);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *audio_dma_initialize(struct dma_dev_s *dma_dev,
                                               uint8_t chan_num, bool playback,
                                               uint8_t fifo_width, uintptr_t fifo_addr)
{
  struct audio_dma_s *audio_dma;

  if (!dma_dev)
    return NULL;
  audio_dma = kmm_zalloc(sizeof(struct audio_dma_s));
  if (!audio_dma)
    return NULL;
  audio_dma->chan = DMA_GET_CHAN(dma_dev, chan_num);
  if (!audio_dma->chan)
    {
      kmm_free(audio_dma);
      return NULL;
    }
  audio_dma->playback = playback;
  audio_dma->fifo_width = fifo_width;
  if (audio_dma->playback)
    audio_dma->dst_addr = up_addrenv_va_to_pa((void *)fifo_addr);
  else
    audio_dma->src_addr = up_addrenv_va_to_pa((void *)fifo_addr);

  audio_dma->buffer_size = CONFIG_AUDIO_BUFFER_NUMBYTES;
  audio_dma->buffer_num  = CONFIG_AUDIO_NUM_BUFFERS;
  dq_init(&audio_dma->pendq);

  audio_dma->dev.ops = &g_audio_dma_ops;
  return &audio_dma->dev;
}
