/*
 * cir_processor.c
 * C library cho CIR Real-time Plotter - 4 Anchors
 *
 * Compile:
 *   Linux : gcc -O2 -shared -fPIC -o cir_processor.so cir_processor.c -lm
 *   Windows: gcc -O2 -shared -o cir_processor.dll cir_processor.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

// ==================== CẤU HÌNH ====================
#define NUM_ANCHORS   4
#define CIR_SAMPLES   100
#define HISTORY_LEN   200
#define FP_SEARCH_WIN 20   // tìm first-path trong N mẫu đầu

// ==================== CẤU TRÚC DỮ LIỆU ====================

typedef struct {
    int    cir[CIR_SAMPLES];       // CIR waveform hiện tại
    int    dist_hist[HISTORY_LEN]; // lịch sử khoảng cách
    int    hist_head;              // con trỏ circular buffer
    int    distance;               // khoảng cách hiện tại (mm)
    int    frame_count;            // số frame đã nhận
    int    fp_index;               // vị trí first-path peak
    int    cir_peak;               // giá trị đỉnh CIR
    double fps;                    // tốc độ frame thực tế
    double last_time;              // timestamp frame cuối (dùng từ Python)
    int    alive;                  // 1 nếu nhận data trong 1.5s gần nhất
} AnchorCIRState;

// ==================== BIẾN TOÀN CỤC ====================
static AnchorCIRState anchor_states[NUM_ANCHORS];
static int            initialized = 0;

// Map anchor ID index: 0=1001, 1=1002, 2=1003, 3=1004
static int anchor_ids[NUM_ANCHORS] = {0x1001, 0x1002, 0x1003, 0x1004};

// ==================== UTILITY ====================

static int get_anchor_index(int anchor_id) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        if (anchor_ids[i] == anchor_id) return i;
    }
    return -1;
}

static int find_first_path(const int* cir, int window) {
    int peak_val = -1;
    int peak_idx = 0;
    int n = window < CIR_SAMPLES ? window : CIR_SAMPLES;
    for (int i = 0; i < n; i++) {
        if (cir[i] > peak_val) {
            peak_val = cir[i];
            peak_idx = i;
        }
    }
    return peak_idx;
}

static int find_global_peak(const int* cir) {
    int peak = 0;
    for (int i = 0; i < CIR_SAMPLES; i++) {
        if (cir[i] > peak) peak = cir[i];
    }
    return peak;
}

// ==================== EXPORTED FUNCTIONS ====================

EXPORT void cir_init(void) {
    memset(anchor_states, 0, sizeof(anchor_states));
    initialized = 1;
}

/*
 * cir_update - cập nhật state cho 1 anchor
 *
 * anchor_id : 0x1001 .. 0x1004
 * distance  : khoảng cách mm
 * cir_data  : mảng CIR_SAMPLES giá trị int
 * timestamp : thời gian hiện tại (giây, từ Python time.time())
 *
 * Trả về: index anchor (0-3), hoặc -1 nếu lỗi
 */
EXPORT int cir_update(int anchor_id, int distance, const int* cir_data, double timestamp) {
    if (!initialized) cir_init();

    int idx = get_anchor_index(anchor_id);
    if (idx < 0) return -1;

    AnchorCIRState* st = &anchor_states[idx];

    // Cập nhật CIR
    memcpy(st->cir, cir_data, CIR_SAMPLES * sizeof(int));

    // Cập nhật distance
    st->distance = distance;

    // Cập nhật history (circular buffer)
    st->dist_hist[st->hist_head] = distance;
    st->hist_head = (st->hist_head + 1) % HISTORY_LEN;

    // Tính first-path index
    st->fp_index = find_first_path(cir_data, FP_SEARCH_WIN);

    // Tính global peak
    st->cir_peak = find_global_peak(cir_data);

    // FPS
    if (st->last_time > 0.0 && timestamp > st->last_time) {
        double dt = timestamp - st->last_time;
        // Exponential moving average
        double inst_fps = 1.0 / dt;
        st->fps = (st->fps < 0.01) ? inst_fps : (0.9 * st->fps + 0.1 * inst_fps);
    }
    st->last_time = timestamp;

    st->frame_count++;
    st->alive = 1;

    return idx;
}

/*
 * cir_get_cir - lấy mảng CIR của anchor
 * out_cir phải là mảng int[CIR_SAMPLES] do caller cấp phát
 */
EXPORT int cir_get_cir(int anchor_id, int* out_cir) {
    int idx = get_anchor_index(anchor_id);
    if (idx < 0) return -1;
    memcpy(out_cir, anchor_states[idx].cir, CIR_SAMPLES * sizeof(int));
    return 0;
}

/*
 * cir_get_dist_hist - lấy lịch sử distance theo thứ tự thời gian
 * out_hist phải là mảng int[HISTORY_LEN] do caller cấp phát
 * Trả về history sắp xếp từ cũ → mới (circular buffer đã được unwrap)
 */
EXPORT int cir_get_dist_hist(int anchor_id, int* out_hist) {
    int idx = get_anchor_index(anchor_id);
    if (idx < 0) return -1;

    AnchorCIRState* st = &anchor_states[idx];
    int head = st->hist_head;

    // Unwrap circular buffer: [head..end] + [0..head-1]
    int part1 = HISTORY_LEN - head;
    memcpy(out_hist,         st->dist_hist + head, part1 * sizeof(int));
    memcpy(out_hist + part1, st->dist_hist,        head  * sizeof(int));

    return 0;
}

/*
 * cir_get_stats - lấy thống kê tổng hợp của 1 anchor
 * Trả về qua các con trỏ out_*
 */
EXPORT int cir_get_stats(int anchor_id,
                          int*    out_distance,
                          int*    out_fp_index,
                          int*    out_cir_peak,
                          int*    out_frame_count,
                          double* out_fps,
                          int*    out_alive) {
    int idx = get_anchor_index(anchor_id);
    if (idx < 0) return -1;

    AnchorCIRState* st = &anchor_states[idx];
    if (out_distance)    *out_distance    = st->distance;
    if (out_fp_index)    *out_fp_index    = st->fp_index;
    if (out_cir_peak)    *out_cir_peak    = st->cir_peak;
    if (out_frame_count) *out_frame_count = st->frame_count;
    if (out_fps)         *out_fps         = st->fps;
    if (out_alive)       *out_alive       = st->alive;

    return 0;
}

/*
 * cir_mark_dead - đánh dấu anchor không còn alive
 * Gọi từ Python khi timeout > 1.5s
 */
EXPORT void cir_mark_dead(int anchor_id) {
    int idx = get_anchor_index(anchor_id);
    if (idx >= 0) anchor_states[idx].alive = 0;
}

/*
 * cir_reset - reset toàn bộ state
 */
EXPORT void cir_reset(void) {
    memset(anchor_states, 0, sizeof(anchor_states));
}

/*
 * cir_get_num_samples - trả về số CIR samples (để Python không hardcode)
 */
EXPORT int cir_get_num_samples(void) { return CIR_SAMPLES; }
EXPORT int cir_get_history_len(void) { return HISTORY_LEN; }
