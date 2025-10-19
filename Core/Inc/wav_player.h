#ifndef __WAV_PLAYER_H
#define __WAV_PLAYER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fatfs.h"
#include "sai.h"

// WAV文件状态定义
typedef enum {
    WAV_OK = 0,
    WAV_RIFF_ERROR,
    WAV_FORMAT_ERROR,
    WAV_FMT_ERROR,
    WAV_DATA_ERROR
} WAV_Status;

// 音频播放状态
typedef enum {
    AUDIO_STOPPED = 0,
    AUDIO_PLAYING,
    AUDIO_PAUSED
} Audio_State;

// WAV文件信息结构
typedef struct {
    uint32_t sample_rate;       // 采样率
    uint16_t channels;          // 声道数
    uint16_t bit_depth;         // 位深度
    uint32_t data_size;         // 音频数据大小
    uint32_t data_offset;       // 音频数据偏移
    uint32_t total_samples;     // 总采样点数
    uint32_t duration_ms;       // 音频时长(ms)
} WAV_Info;

// 音频播放控制结构
typedef struct {
    Audio_State state;
    FIL file;
    WAV_Info info;
    uint32_t bytes_remaining;
    uint32_t total_bytes;
    uint32_t current_position;
} WAV_Player;

// WAV文件块标识
#define WAV_RIFF_TAG    0x46464952  // "RIFF"
#define WAV_WAVE_TAG    0x45564157  // "WAVE"
#define WAV_FMT_TAG     0x20746D66  // "fmt "
#define WAV_DATA_TAG    0x61746164  // "data"

// 缓冲区大小定义
#define WAV_BUFFER_SIZE 4096
#define SAI_BUFFER_SIZE 2048  // 与main.c中保持一致

// 外部声明（在main.c中定义）
extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;

// 函数声明
WAV_Status WAV_ParseHeader(FIL* file, WAV_Info* info);
uint8_t WAV_PlayFile(const TCHAR* filename);
uint32_t WAV_GetAudioData(FIL* file, uint8_t* buffer, uint32_t size);
WAV_Status WAV_SeekToTime(FIL* file, WAV_Info* info, uint32_t ms);

// SAI接口函数
void WAV_StartPlayback(void);
void WAV_StartPlayback_Alternative(void);
void WAV_StopPlayback(void);
void WAV_PausePlayback(void);
void WAV_ResumePlayback(void);
void Verify_SAI_Buffer(uint8_t buffer_idx);
uint8_t WAV_IsPlaying(void);
uint32_t WAV_GetCurrentTime(void);
uint32_t WAV_GetDuration(void);
void WAV_Performance_Monitor(void);

// 回调函数（需要在main.c中调用）
extern void WAV_SAI_DMA_HalfComplete_Callback(DMA_HandleTypeDef* hdma);
extern void WAV_SAI_DMA_Complete_Callback(DMA_HandleTypeDef* hdma);

#ifdef __cplusplus
}
#endif

#endif // __WAV_PLAYER_H