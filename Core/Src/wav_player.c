#include "wav_player.h"
#include "sai.h"
#include "stm32h750xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma_ex.h"
#include "stm32h7xx_hal_sai.h"
#include <string.h>
#include <stdio.h>

// 全局播放器实例
static WAV_Player wav_player = {0};

// 音频数据缓冲区
static uint8_t audio_data_buffer[2][WAV_BUFFER_SIZE];
static volatile uint8_t current_buffer = 0;
static volatile uint8_t buffer_filled[2] = {0, 0};
volatile float cpu_usage = 0.00f;

__attribute__((section(".ram_d1"), aligned(32)))
uint32_t sai_tx_buffer[2][SAI_BUFFER_SIZE] = {{0}}; // 双缓冲

/**
  * @brief  音频格式转换（16bit -> 24bit SAI格式）
  */
static uint32_t Audio_ConvertToSAIFormat(uint8_t* src, uint32_t* dst, uint32_t src_samples, uint16_t src_bits, uint16_t channels) {
    uint32_t dst_samples = 0;
    
    if(src_bits == 16) {
        // 16bit -> 24bit 转换
        int16_t* src16 = (int16_t*)src;
        
        // 对于立体声WAV文件，数据是交错的：LRLRLR...
        for(uint32_t i = 0; i < src_samples * channels; i += channels) {
            // 左声道
            int32_t left_sample = (int32_t)src16[i] << 8;  // 左移8位到24bit范围
            // 右声道
            int32_t right_sample = (int32_t)src16[i + 1] << 8;
            
            // SAI需要32位数据，24bit数据放在低24位
            *dst++ = left_sample & 0xFFFFFF;
            *dst++ = right_sample & 0xFFFFFF;
            dst_samples += 2;
        }
        
        // printf("Audio_ConvertToSAIFormat: Converted %lu 16-bit samples to %lu 24-bit samples\n", 
        //        src_samples, dst_samples);
    } 
    else if(src_bits == 24) {
        // 24bit直接处理
        uint8_t* sample_ptr = src;
        for(uint32_t i = 0; i < src_samples; i++) {
            // 处理左声道（小端序）
            uint32_t left_sample = sample_ptr[0] | (sample_ptr[1] << 8) | (sample_ptr[2] << 16);
            sample_ptr += 3;
            
            // 处理右声道
            uint32_t right_sample = sample_ptr[0] | (sample_ptr[1] << 8) | (sample_ptr[2] << 16);
            sample_ptr += 3;
            
            // 符号扩展
            if(left_sample & 0x800000) left_sample |= 0xFF000000;
            if(right_sample & 0x800000) right_sample |= 0xFF000000;
            
            *dst++ = left_sample & 0xFFFFFF;
            *dst++ = right_sample & 0xFFFFFF;
            dst_samples += 2;
        }
    }
    
    return dst_samples;
}

/**
  * @brief  填充音频缓冲区
  */
static uint32_t Fill_Audio_Buffer(uint8_t buffer_idx) {
    if(wav_player.state != AUDIO_PLAYING) {
        printf("Fill_Audio_Buffer: Not in playing state\n");
        return 0;
    }
    
    if(wav_player.bytes_remaining == 0) {
        printf("Fill_Audio_Buffer: No more data to read\n");
        WAV_StopPlayback();
        return 0;
    }
    
    // 计算每个样本的字节数
    uint32_t bytes_per_sample = wav_player.info.channels * (wav_player.info.bit_depth / 8);
    if(bytes_per_sample == 0) {
        printf("Fill_Audio_Buffer: Invalid bytes per sample: channels=%u, bits=%u\n", 
               wav_player.info.channels, wav_player.info.bit_depth);
        return 0;
    }
    
    // 计算可以读取的最大样本数（确保不超过缓冲区大小）
    uint32_t max_samples = WAV_BUFFER_SIZE / bytes_per_sample;
    if(max_samples == 0) {
        printf("Fill_Audio_Buffer: Buffer too small for samples\n");
        return 0;
    }
    
    // 计算要读取的字节数
    uint32_t bytes_to_read = max_samples * bytes_per_sample;
    if(bytes_to_read > wav_player.bytes_remaining) {
        bytes_to_read = wav_player.bytes_remaining;
    }
    
    // 确保字节数是样本大小的整数倍
    bytes_to_read = (bytes_to_read / bytes_per_sample) * bytes_per_sample;
    
    if(bytes_to_read == 0) {
        printf("Fill_Audio_Buffer: No bytes to read after alignment\n");
        WAV_StopPlayback();
        return 0;
    }
    
    // printf("Fill_Audio_Buffer[%d]: Reading %lu bytes (%lu samples) from file, remaining: %lu\n", 
    //        buffer_idx, bytes_to_read, bytes_to_read / bytes_per_sample, wav_player.bytes_remaining);
    
    // 读取音频数据
    UINT bytes_read;
    FRESULT fr = f_read(&wav_player.file, audio_data_buffer[buffer_idx], bytes_to_read, &bytes_read);
    
    if(fr != FR_OK) {
        printf("Fill_Audio_Buffer: File read error: %d\n", fr);
        WAV_StopPlayback();
        return 0;
    }
    
    if(bytes_read == 0) {
        printf("Fill_Audio_Buffer: No bytes read (EOF?)\n");
        WAV_StopPlayback();
        return 0;
    }
    
    // 确保读取的字节数是样本大小的整数倍
    bytes_read = (bytes_read / bytes_per_sample) * bytes_per_sample;
    if(bytes_read == 0) {
        printf("Fill_Audio_Buffer: Read incomplete sample\n");
        return 0;
    }
    
    uint32_t samples_read = bytes_read / bytes_per_sample;
    // printf("Fill_Audio_Buffer[%d]: Read %lu bytes, %lu samples\n", buffer_idx, bytes_read, samples_read);
    
    // 清空SAI缓冲区（重要！）
    memset(sai_tx_buffer[buffer_idx], 0, SAI_BUFFER_SIZE * sizeof(uint32_t));
    
    // 格式转换到SAI缓冲区
    Audio_ConvertToSAIFormat(audio_data_buffer[buffer_idx], 
                                                   sai_tx_buffer[buffer_idx], 
                                                   samples_read, 
                                                   wav_player.info.bit_depth,
                                                   wav_player.info.channels);
    
    // printf("Fill_Audio_Buffer[%d]: Converted %lu samples to %lu SAI samples\n", 
    //        buffer_idx, samples_read, sai_samples);
    
    // 更新播放状态
    wav_player.bytes_remaining -= bytes_read;
    wav_player.current_position += bytes_read;
    
    // 标记缓冲区已填充
    buffer_filled[buffer_idx] = 1;
    
    // printf("Fill_Audio_Buffer[%d]: Success, remaining: %lu bytes\n", buffer_idx, wav_player.bytes_remaining);
    // Verify_SAI_Buffer(buffer_idx);
    return bytes_read;
}

/**
  * @brief  WAV文件头解析
  */
WAV_Status WAV_ParseHeader(FIL* file, WAV_Info* info) {
    uint32_t chunk_id, chunk_size;
    uint32_t riff_size;
    UINT bytes_read;
    FRESULT fr;
    
    // 保存初始文件位置
    uint32_t initial_pos = f_tell(file);
    printf("WAV_ParseHeader: Starting at position %lu\n", (unsigned long)initial_pos);
    
    // 读取RIFF头
    fr = f_read(file, &chunk_id, 4, &bytes_read);
    if(fr != FR_OK || bytes_read != 4) {
        printf("WAV_ParseHeader: RIFF read error: %d, bytes_read: %d\n", fr, bytes_read);
        return WAV_RIFF_ERROR;
    }
    
    if(chunk_id != WAV_RIFF_TAG) {
        printf("WAV_ParseHeader: Not a RIFF file: 0x%08lX, expected: 0x%08lX\n", (unsigned long)chunk_id, (long)WAV_RIFF_TAG);
        return WAV_RIFF_ERROR;
    }
    
    printf("WAV_ParseHeader: Found RIFF chunk\n");
    
    f_read(file, &riff_size, 4, &bytes_read);
    f_read(file, &chunk_id, 4, &bytes_read);
    if(chunk_id != WAV_WAVE_TAG) {
        printf("WAV_ParseHeader: Not a WAVE file: 0x%08lX, expected: 0x%08lX\n", (unsigned long)chunk_id, (long)WAV_WAVE_TAG);
        return WAV_FORMAT_ERROR;
    }
    
    printf("WAV_ParseHeader: Found WAVE chunk, riff_size: %lu\n", (unsigned long)riff_size);
    
    // 查找fmt和data块
    uint32_t offset = 12;
    uint8_t fmt_found = 0, data_found = 0;
    
    while(offset < riff_size + 8) {
        f_lseek(file, offset);
        f_read(file, &chunk_id, 4, &bytes_read);
        f_read(file, &chunk_size, 4, &bytes_read);
        
        printf("WAV_ParseHeader: Chunk 0x%08lX, size: %lu at offset: %lu\n", (unsigned long)chunk_id, (unsigned long)chunk_size, (unsigned long)offset);
        
        if(chunk_id == WAV_FMT_TAG) {
            // 解析fmt块
            uint16_t format_tag, channels, bits_per_sample;
            uint32_t sample_rate, avg_bytes_per_sec;
            uint16_t block_align;
            
            f_read(file, &format_tag, 2, &bytes_read);
            f_read(file, &channels, 2, &bytes_read);
            f_read(file, &sample_rate, 4, &bytes_read);
            f_read(file, &avg_bytes_per_sec, 4, &bytes_read);
            f_read(file, &block_align, 2, &bytes_read);
            f_read(file, &bits_per_sample, 2, &bytes_read);
            
            printf("WAV_ParseHeader: Format - tag=%u, channels=%u, rate=%lu, bits=%u, align=%u\n", 
                   format_tag, channels, (unsigned long)sample_rate, bits_per_sample, block_align);
            
            // 只支持PCM格式
            if(format_tag != 1) {
                printf("WAV_ParseHeader: Unsupported format: %u (only PCM supported)\n", format_tag);
                return WAV_FMT_ERROR;
            }
            
            info->channels = channels;
            info->sample_rate = sample_rate;
            info->bit_depth = bits_per_sample;
            fmt_found = 1;
        }
        else if(chunk_id == WAV_DATA_TAG) {
            // 解析data块
            info->data_offset = offset + 8;
            info->data_size = chunk_size;
            if(info->channels > 0 && info->bit_depth > 0) {
                info->total_samples = chunk_size / (info->channels * (info->bit_depth / 8));
                info->duration_ms = (info->total_samples * 1000) / info->sample_rate;
            }
            data_found = 1;
            printf("WAV_ParseHeader: Data chunk - offset=%lu, size=%lu\n", (unsigned long)info->data_offset, (unsigned long)info->data_size);
        }
        else {
            printf("WAV_ParseHeader: Skipping unknown chunk 0x%08lX\n", (unsigned long)chunk_id);
        }
        
        offset += 8 + chunk_size;
        // 跳过填充字节
        if(chunk_size % 2) offset++;
        
        // 如果两个块都找到了，提前退出
        if(fmt_found && data_found) break;
        
        // 防止无限循环
        if(offset > riff_size + 1000) {
            printf("WAV_ParseHeader: Reached offset limit\n");
            break;
        }
    }
    
    if(!fmt_found) {
        printf("WAV_ParseHeader: fmt chunk not found\n");
        return WAV_FMT_ERROR;
    }
    if(!data_found) {
        printf("WAV_ParseHeader: data chunk not found\n");
        return WAV_DATA_ERROR;
    }
    
    printf("WAV_ParseHeader: Success - %luHz, %u-bit, %u channels, %lums\n",
           (unsigned long)info->sample_rate, info->bit_depth, info->channels, (unsigned long)info->duration_ms);
    
    return WAV_OK;
}

/**
  * @brief  启动WAV文件播放
  */
uint8_t WAV_PlayFile(const TCHAR* filename) {
    printf("\n=== WAV_PlayFile: %s ===\n", filename);
    
    // // 停止当前播放
    // WAV_StopPlayback();
    WAV_ResetPlayer();

    // 重新初始化SAI和DMA（关键修复）
    if(!WAV_ReinitSAI()) {
        printf("WAV_PlayFile: Failed to reinitialize SAI\n");
        return 0;
    }
    
    // 打开文件
    FRESULT fr = f_open(&wav_player.file, filename, FA_READ);
    if(fr != FR_OK) {
        printf("WAV_PlayFile: Failed to open file: %d\n", fr);
        return 0;
    }
    
    printf("WAV_PlayFile: File opened successfully, size: %lu\n", f_size(&wav_player.file));
    
    // 解析头文件
    WAV_Status status = WAV_ParseHeader(&wav_player.file, &wav_player.info);
    if(status != WAV_OK) {
        f_close(&wav_player.file);
        printf("WAV_PlayFile: Header parse failed: %d\n", status);
        return 0;
    }
    
    printf("WAV_PlayFile: Header parsed successfully\n");
    
    // 检查格式支持
    if(wav_player.info.bit_depth != 16 && wav_player.info.bit_depth != 24 && wav_player.info.bit_depth != 8) {
        printf("WAV_PlayFile: Unsupported bit depth: %u\n", wav_player.info.bit_depth);
        f_close(&wav_player.file);
        return 0;
    }
    
    // 检查基本参数有效性
    if(wav_player.info.channels == 0 || wav_player.info.sample_rate == 0 || wav_player.info.data_size == 0) {
        printf("WAV_PlayFile: Invalid audio parameters: channels=%u, rate=%lu, size=%lu\n",
               wav_player.info.channels, (unsigned long)wav_player.info.sample_rate, (unsigned long)wav_player.info.data_size);
        f_close(&wav_player.file);
        return 0;
    }
    
    // 检查采样率是否匹配SAI配置（48kHz）
    if(wav_player.info.sample_rate != 48000) {
        printf("WAV_PlayFile: Warning: Sample rate %luHz, SAI configured for 48kHz\n", (unsigned long)wav_player.info.sample_rate);
        // 这里可以添加采样率转换或重新配置SAI的逻辑
    }
    
    // 初始化播放状态
    wav_player.bytes_remaining = wav_player.info.data_size;
    wav_player.total_bytes = wav_player.info.data_size;
    wav_player.current_position = 0;
    wav_player.state = AUDIO_PLAYING;
    
    printf("WAV_PlayFile: Audio info - %luHz, %u-bit, %u channels, %lums, %lu bytes\n",
           (unsigned long)wav_player.info.sample_rate, wav_player.info.bit_depth,
           wav_player.info.channels, (unsigned long)wav_player.info.duration_ms,
           (unsigned long)wav_player.info.data_size);
    
    // 定位到音频数据开始位置
    if(f_lseek(&wav_player.file, wav_player.info.data_offset) != FR_OK) {
        printf("WAV_PlayFile: Failed to seek to data offset: %lu\n", (unsigned long)wav_player.info.data_offset);
        f_close(&wav_player.file);
        return 0;
    }
    
    printf("WAV_PlayFile: Seeked to data offset: %lu\n", (unsigned long)wav_player.info.data_offset);
    
    // 启动播放
    WAV_StartPlayback();
    return 1;
}

/**
  * @brief  启动SAI音频播放
  */
void WAV_StartPlayback(void) {
    printf("WAV_StartPlayback: Starting audio playback\n");
    
    // 重置缓冲区状态
    buffer_filled[0] = 0;
    buffer_filled[1] = 0;
    current_buffer = 0;
    
    // 先停止任何正在进行的DMA传输
    __disable_irq();
    // 停止DMA传输
    HAL_DMA_Abort(&hdma_sai1_a);
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_Abort(&hsai_BlockA1);
    __enable_irq();
    HAL_Delay(10);
    
    // 填充两个缓冲区
    uint32_t buffer0_result = Fill_Audio_Buffer(0);
    if(buffer0_result > 0) {
        printf("WAV_StartPlayback: Buffer 0 filled successfully with %lu bytes\n", (unsigned long)buffer0_result);
    } else {
        printf("WAV_StartPlayback: Failed to fill buffer 0\n");
        WAV_StopPlayback();
        return;
    }
    
    uint32_t buffer1_result = Fill_Audio_Buffer(1);
    if(buffer1_result > 0) {
        printf("WAV_StartPlayback: Buffer 1 filled successfully with %lu bytes\n", (unsigned long)buffer1_result);
    } else {
        printf("WAV_StartPlayback: Failed to fill buffer 1\n");
        WAV_StopPlayback();
        return;
    }
    
    // 清除Cache以确保数据一致性
    SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[0], SAI_BUFFER_SIZE * sizeof(uint32_t));
    SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[1], SAI_BUFFER_SIZE * sizeof(uint32_t));
    
    printf("WAV_StartPlayback: Cache cleaned, starting DMA...\n");
    
    // 使用与正弦波测试相同的方式启动DMA
    // 注意：这里使用 HAL_DMAEx_MultiBufferStart 而不是 HAL_SAI_Transmit_DMA
    __disable_irq();
    
    // 使用多缓冲区模式启动DMA（与正弦波测试相同）
    HAL_StatusTypeDef dma_status = HAL_DMAEx_MultiBufferStart(&hdma_sai1_a,
                                     (uint32_t)sai_tx_buffer[0], 
                                     (uint32_t)&hsai_BlockA1.Instance->DR, 
                                     (uint32_t)sai_tx_buffer[1], 
                                     SAI_BUFFER_SIZE);
    __enable_irq();
    
    if(dma_status == HAL_OK) {
        printf("WAV_StartPlayback: DMA MultiBuffer started successfully\n");
        
        // 手动使能SAI（与正弦波测试相同）
        // __HAL_SAI_ENABLE(&hsai_BlockA1);
        hsai_BlockA1.Instance->CR1 |= SAI_xCR1_SAIEN | SAI_xCR1_DMAEN;
        
        printf("WAV_StartPlayback: SAI manually enabled\n");
        wav_player.state = AUDIO_PLAYING;
        
        // 检查SAI状态
        uint32_t sr = hsai_BlockA1.Instance->SR;
        uint32_t cr1 = hsai_BlockA1.Instance->CR1;
        uint32_t cr2 = hsai_BlockA1.Instance->CR2;
        
        printf("WAV_StartPlayback: SAI Status - SR: 0x%08lX, CR1: 0x%08lX, CR2: 0x%08lX\n", (unsigned long)sr, (unsigned long)cr1, (unsigned long)cr2);
        
        if(sr & SAI_xSR_FLVL) {
            printf("WAV_StartPlayback: FIFO level: %lu\n", (sr & SAI_xSR_FLVL) >> 16);
        }
        // while(wav_player.state == AUDIO_PLAYING)
        // {
        //     (DMA1_Stream0->CR & DMA_SxCR_CT) ? WAV_SAI_DMA_Complete_Callback() : WAV_SAI_DMA_HalfComplete_Callback();
        //     HAL_Delay(10);
        // }
    } else {
        printf("WAV_StartPlayback: DMA MultiBuffer start failed: %d\n", dma_status);
        wav_player.state = AUDIO_STOPPED;
    }
}

/**
  * @brief  备用启动方法 - 直接配置寄存器
  */
void WAV_StartPlayback_Alternative(void) {
    printf("WAV_StartPlayback_Alternative: Using alternative startup method\n");
    __disable_irq();
    
    // 停止DMA传输
    HAL_DMA_Abort(&hdma_sai1_a);
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_Abort(&hsai_BlockA1);
    __enable_irq();
    HAL_Delay(10);
    __disable_irq();
    // 配置DMA多缓冲区
    HAL_StatusTypeDef dma_status = HAL_DMAEx_MultiBufferStart_IT(&hdma_sai1_a,
                                     (uint32_t)sai_tx_buffer[0], 
                                     (uint32_t)&hsai_BlockA1.Instance->DR, 
                                     (uint32_t)sai_tx_buffer[1], 
                                     SAI_BUFFER_SIZE);
    __enable_irq();
    if(dma_status == HAL_OK) {
        printf("WAV_StartPlayback_Alternative: DMA started successfully\n");
        
        // 直接使能SAI和DMA
        __HAL_SAI_ENABLE(&hsai_BlockA1);
        hsai_BlockA1.Instance->CR1 |= SAI_xCR1_DMAEN;
        
        // 检查SAI状态寄存器
        uint32_t cr1 = hsai_BlockA1.Instance->CR1;
        printf("WAV_StartPlayback_Alternative: SAI CR1 = 0x%08lX\n", (unsigned long)cr1);
        
        if(cr1 & SAI_xCR1_SAIEN) {
            printf("WAV_StartPlayback_Alternative: SAI enabled successfully\n");
            wav_player.state = AUDIO_PLAYING;
        } else {
            printf("WAV_StartPlayback_Alternative: SAI enable failed\n");
            wav_player.state = AUDIO_STOPPED;
        }
    } else {
        printf("WAV_StartPlayback_Alternative: DMA start failed: %d\n", dma_status);
        wav_player.state = AUDIO_STOPPED;
    }
}

/**
  * @brief  验证SAI缓冲区数据
  */
void Verify_SAI_Buffer(uint8_t buffer_idx) {
    printf("Verify_SAI_Buffer[%d]: Checking first 10 samples\n", buffer_idx);
    
    for(int i = 0; i < 10 && i < SAI_BUFFER_SIZE; i++) {
        uint32_t sample = sai_tx_buffer[buffer_idx][i];
        // 将24bit数据转换为有符号整数
        int32_t signed_sample = (int32_t)(sample << 8) >> 8; // 符号扩展
        
        printf("  Sample[%d]: 0x%06lX (%ld)\n", i, (unsigned long)sample & 0xFFFFFF, (long)signed_sample);
    }
    
    // 检查是否有非零数据
    uint32_t zero_count = 0;
    for(int i = 0; i < SAI_BUFFER_SIZE; i++) {
        if(sai_tx_buffer[buffer_idx][i] == 0) {
            zero_count++;
        }
    }
    
    printf("Verify_SAI_Buffer[%d]: Zero samples: %lu/%d (%.1f%%)\n", 
           buffer_idx, (unsigned long)zero_count, SAI_BUFFER_SIZE, 
           (zero_count * 100.0f) / SAI_BUFFER_SIZE);
}

// ... 其他函数保持不变（StopPlayback, PausePlayback, ResumePlayback等）...
/**
  * @brief  停止播放
  */
void WAV_StopPlayback(void) {
    if(wav_player.state != AUDIO_STOPPED) {
        __disable_irq();
        // 停止DMA传输
        HAL_DMA_Abort(&hdma_sai1_a);
        HAL_SAI_DMAStop(&hsai_BlockA1);
        HAL_SAI_Abort(&hsai_BlockA1);
        __enable_irq();
        
        // 关闭文件
        // if(f_is_open(&wav_player.file)) {
        f_close(&wav_player.file);
        // }
        
        wav_player.state = AUDIO_STOPPED;
        printf("WAV playback stopped\n");
    }
}

/**
  * @brief  暂停播放
  */
void WAV_PausePlayback(void) {
    if(wav_player.state == AUDIO_PLAYING) {
        HAL_SAI_DMAPause(&hsai_BlockA1);
        wav_player.state = AUDIO_PAUSED;
        printf("WAV playback paused\n");
    }
}

/**
  * @brief  恢复播放
  */
void WAV_ResumePlayback(void) {
    if(wav_player.state == AUDIO_PAUSED) {
        HAL_SAI_DMAResume(&hsai_BlockA1);
        wav_player.state = AUDIO_PLAYING;
        printf("WAV playback resumed\n");
    }
}

/**
  * @brief  检查是否正在播放
  */
uint8_t WAV_IsPlaying(void) {
    return wav_player.state;
}

/**
  * @brief  获取当前播放时间
  */
uint32_t WAV_GetCurrentTime(void) {
    if(wav_player.total_bytes == 0) return 0;
    
    uint32_t bytes_played = wav_player.total_bytes - wav_player.bytes_remaining;
    uint32_t samples_played = bytes_played / (wav_player.info.channels * (wav_player.info.bit_depth / 8));
    
    return (samples_played * 1000) / wav_player.info.sample_rate;
}

/**
  * @brief  获取总时长
  */
uint32_t WAV_GetDuration(void) {
    return wav_player.info.duration_ms;
}

/**
  * @brief  性能监控
  */
void WAV_Performance_Monitor(void) {
    static uint32_t last_time = 0;
    static uint32_t callback_count = 0;
    // static uint32_t total_callbacks = 0;
    
    uint32_t current_time = HAL_GetTick();
    
    if(current_time - last_time >= 1000) { // 每秒报告一次
        uint32_t callbacks_per_second = callback_count;
        cpu_usage = (callbacks_per_second * 100.0f) / 48000.0f; // 基于48kHz采样率估算
        
        printf("Performance: %lu callbacks/sec, CPU usage: %.1f%%\n", 
               (unsigned long)callbacks_per_second, cpu_usage);
        
        callback_count = 0;
        last_time = current_time;
        // total_callbacks++;
    }
    
    callback_count++;
}

/**
  * @brief  完全重置播放器状态
  */
void WAV_ResetPlayer(void) {
    f_close(&wav_player.file);
    // 停止DMA传输
    __disable_irq();
    HAL_DMA_Abort(&hdma_sai1_a);
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_Abort(&hsai_BlockA1);
    __enable_irq();
    
    // 重置播放器状态
    wav_player.state = AUDIO_STOPPED;
    wav_player.bytes_remaining = 0;
    wav_player.total_bytes = 0;
    wav_player.current_position = 0;
    memset(&wav_player.info, 0, sizeof(WAV_Info));
    
    // 重置缓冲区状态
    buffer_filled[0] = 0;
    buffer_filled[1] = 0;
    current_buffer = 0;
    
    // 清空缓冲区
    memset(audio_data_buffer[0], 0, WAV_BUFFER_SIZE);
    memset(audio_data_buffer[1], 0, WAV_BUFFER_SIZE);
    memset(sai_tx_buffer[0], 0, SAI_BUFFER_SIZE * sizeof(uint32_t));
    memset(sai_tx_buffer[1], 0, SAI_BUFFER_SIZE * sizeof(uint32_t));
    
    // 清除Cache
    SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[0], SAI_BUFFER_SIZE * sizeof(uint32_t));
    SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[1], SAI_BUFFER_SIZE * sizeof(uint32_t));
    
    printf("WAV_ResetPlayer: Player state fully reset\n");
}

/**
  * @brief  重新初始化SAI和DMA
  */
uint8_t WAV_ReinitSAI(void) {
    printf("WAV_ReinitSAI: Reinitializing SAI and DMA\n");
    
    // 完全停止SAI和DMA
    __disable_irq();
    HAL_DMA_Abort(&hdma_sai1_a);
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_Abort(&hsai_BlockA1);
    __enable_irq();
    
    HAL_Delay(50);
    
    // 重新初始化SAI
    HAL_StatusTypeDef sai_status = HAL_SAI_DeInit(&hsai_BlockA1);
    if(sai_status != HAL_OK) {
        printf("WAV_ReinitSAI: SAI deinit failed: %d\n", sai_status);
    }
    
    HAL_Delay(10);
    
    sai_status = HAL_SAI_Init(&hsai_BlockA1);
    if(sai_status != HAL_OK) {
        printf("WAV_ReinitSAI: SAI init failed: %d\n", sai_status);
        return 0;
    }
    
    // 重新配置DMA中断
    __HAL_DMA_ENABLE_IT(&hdma_sai1_a, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    
    printf("WAV_ReinitSAI: SAI and DMA reinitialized successfully\n");
    return 1;
}

uint8_t WAV_PlayFile_WithRetry(const TCHAR* filename, uint8_t max_retries) {
    for (int attempt = 0; attempt < max_retries; attempt++) {
        if (attempt > 0) {
            printf("Retry attempt %d/%d\n", attempt + 1, max_retries);
            HAL_Delay(100); // 重试前延迟
        }
        
        if (WAV_PlayFile(filename)) {
            return 1; // 成功
        }
        
        // 完全重置系统状态
        WAV_ResetPlayer();
        HAL_Delay(50);
    }
    
    return 0; // 所有重试都失败
}

/**
  * @brief  SAI DMA半传输完成回调（填充第一个缓冲区）
  */
void WAV_SAI_DMA_HalfComplete_Callback(DMA_HandleTypeDef* hdma) {
    // printf("DMA Half Complete - filling buffer 0\n");
    
    if(wav_player.state == AUDIO_PLAYING) {
        // 清除Cache
        SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[0], SAI_BUFFER_SIZE * sizeof(uint32_t));
        
        // 填充缓冲区0
        uint32_t bytes_filled = Fill_Audio_Buffer(0);
        if(bytes_filled == 0 && wav_player.bytes_remaining == 0) {
            // 文件播放完成
            printf("Playback completed naturally\n");
            WAV_StopPlayback();
        }
        else {
            // printf("WAV_StartPlayback: Buffer 0 filled with %lu bytes\n", (unsigned long)bytes_filled);
        }
        
        // 再次清除Cache
        SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[0], SAI_BUFFER_SIZE * sizeof(uint32_t));
    }
}

/**
  * @brief  SAI DMA传输完成回调（填充第二个缓冲区）
  */
void WAV_SAI_DMA_Complete_Callback(DMA_HandleTypeDef* hdma) {
    // printf("DMA Complete - filling buffer 1\n");
    
    if(wav_player.state == AUDIO_PLAYING) {
        // 清除Cache
        SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[1], SAI_BUFFER_SIZE * sizeof(uint32_t));
        
        // 填充缓冲区1
        uint32_t bytes_filled = Fill_Audio_Buffer(1);
        if(bytes_filled == 0 && wav_player.bytes_remaining == 0) {
            // 文件播放完成
            printf("Playback completed naturally\n");
            WAV_StopPlayback();
        }
        else {
            // printf("WAV_StartPlayback: Buffer 1 filled with %lu bytes\n", (unsigned long)bytes_filled);
        }
        WAV_Performance_Monitor();
        // 再次清除Cache
        SCB_CleanDCache_by_Addr((uint32_t*)sai_tx_buffer[1], SAI_BUFFER_SIZE * sizeof(uint32_t));
    }
}