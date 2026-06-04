#pragma once

#include <cmath>
#include <cstdint>

// --------------------------------------------------------------------------
// BCG / SCG heartbeat detector for accelerometer data
//
// Signal chain:
//   raw accel magnitude
//     -> IIR bandpass (HP + LP)
//     -> sliding window:  mean (DC removal) + std (adaptive threshold)
//     -> Schmitt trigger (dual threshold with hysteresis)
//     -> refractory period
//     -> BPM / HRV
//
// Two threshold levels  (Schmitt trigger):
//   THR_HI = alpha_hi * sigma  -- signal must exceed this to ARM the trigger
//   THR_LO = alpha_lo * sigma  -- signal must drop below this to RESET
//
// This prevents chattering and handles BCG multi-peak morphology.
// --------------------------------------------------------------------------

class SignalProcessor {
public:
    static const int MAX_RECENT_PEAKS = 32;
    static const int MAX_WINDOW = 512;  // max window size in samples

    struct PeakInfo {
        uint32_t timestamp_us;
        float    value;
        float    interval_ms;
    };

    // ---- Configuration ----
    void configure(float sample_rate,
                   float hp_cutoff_hz  = 0.5f,
                   float lp_cutoff_hz  = 10.0f,
                   float window_sec    = 2.0f,
                   float alpha_hi      = 2.0f,
                   float alpha_lo      = 0.5f,
                   float refractory_ms = 300.0f)
    {
        float dt = 1.0f / sample_rate;

        // IIR high-pass
        float rc_hp = 1.0f / (2.0f * M_PI * hp_cutoff_hz);
        hp_alpha_ = rc_hp / (rc_hp + dt);

        // IIR low-pass
        float rc_lp = 1.0f / (2.0f * M_PI * lp_cutoff_hz);
        lp_alpha_ = dt / (rc_lp + dt);

        sample_rate_   = sample_rate;
        alpha_hi_      = alpha_hi;
        alpha_lo_      = alpha_lo;
        refractory_ms_ = refractory_ms;

        // Window size in samples (clamped to MAX_WINDOW)
        win_size_ = (int)(window_sec * sample_rate);
        if (win_size_ > MAX_WINDOW) win_size_ = MAX_WINDOW;
        if (win_size_ < 10)         win_size_ = 10;
    }

    // ---- Process one sample -> returns bandpass-filtered value ----
    float process(float input) {
        // High-pass
        float hp = hp_alpha_ * (hp_prev_out_ + input - hp_prev_in_);
        hp_prev_in_  = input;
        hp_prev_out_ = hp;

        // Low-pass
        float lp = lp_alpha_ * hp + (1.0f - lp_alpha_) * lp_prev_out_;
        lp_prev_out_ = lp;

        last_filtered_ = lp;
        return lp;
    }

    // ---- Peak detection (call after process()) ----
    bool detectPeak(float filtered, uint32_t time_us) {
        // Push into sliding window
        window_[win_idx_] = filtered;
        win_idx_ = (win_idx_ + 1) % win_size_;
        if (win_count_ < win_size_) win_count_++;

        // Need a full window before detecting
        if (win_count_ < win_size_) return false;

        // Compute mean and std over window
        float sum = 0, sum2 = 0;
        for (int i = 0; i < win_size_; i++) {
            sum  += window_[i];
            sum2 += window_[i] * window_[i];
        }
        float mean = sum / win_size_;
        float variance = sum2 / win_size_ - mean * mean;
        if (variance < 0) variance = 0;
        float sigma = sqrtf(variance);

        last_mean_  = mean;
        last_sigma_ = sigma;

        // DC-removed signal
        float ac = filtered - mean;
        last_ac_ = ac;

        // Dual thresholds
        float thr_hi = alpha_hi_ * sigma;
        float thr_lo = alpha_lo_ * sigma;
        last_thr_hi_ = thr_hi;
        last_thr_lo_ = thr_lo;

        uint32_t time_ms = time_us / 1000;

        // Schmitt trigger state machine
        bool peak_detected = false;

        switch (schmitt_state_) {
        case IDLE:
            if (ac > thr_hi) {
                schmitt_state_ = ARMED;
                peak_val_      = ac;
                peak_time_us_  = time_us;
                peak_time_ms_  = time_ms;
            }
            break;

        case ARMED:
            // Track the maximum while armed
            if (ac > peak_val_) {
                peak_val_     = ac;
                peak_time_us_ = time_us;
                peak_time_ms_ = time_ms;
            }
            // Signal dropped below low threshold -> peak confirmed
            if (ac < thr_lo) {
                schmitt_state_ = REFRACTORY;
                refract_end_ms_ = peak_time_ms_ + (uint32_t)refractory_ms_;

                if (last_peak_time_ms_ > 0) {
                    uint32_t elapsed = peak_time_ms_ - last_peak_time_ms_;
                    if (elapsed >= (uint32_t)refractory_ms_) {
                        float interval = (float)elapsed;
                        last_peak_time_ms_ = peak_time_ms_;
                        addInterval(interval);
                        storePeak(peak_time_us_, peak_val_, interval);
                        peak_detected = true;
                    }
                } else {
                    // First peak
                    last_peak_time_ms_ = peak_time_ms_;
                    storePeak(peak_time_us_, peak_val_, 0);
                    peak_detected = true;
                }
            }
            break;

        case REFRACTORY:
            if (time_ms >= refract_end_ms_ && ac < thr_lo) {
                schmitt_state_ = IDLE;
            }
            break;
        }

        return peak_detected;
    }

    // ---- Getters ----
    float getBPM()       const { return bpm_; }
    float getHRV()       const { return hrv_; }
    float getFiltered()  const { return last_filtered_; }
    float getMean()      const { return last_mean_; }
    float getSigma()     const { return last_sigma_; }
    float getAC()        const { return last_ac_; }
    float getThrHi()     const { return last_thr_hi_; }
    float getThrLo()     const { return last_thr_lo_; }

    // Backwards-compatible aliases for main.cpp debug prints
    float getThreshold() const { return last_thr_hi_; }
    float getEnvMax()    const { return last_sigma_; }
    float getEnvMin()    const { return last_mean_; }

    int drainPeaks(PeakInfo* out, int maxOut) {
        int copied = 0;
        while (peak_read_ != peak_write_ && copied < maxOut) {
            out[copied++] = recent_peaks_[peak_read_];
            peak_read_ = (peak_read_ + 1) % MAX_RECENT_PEAKS;
        }
        return copied;
    }

    void reset() {
        hp_prev_in_ = 0; hp_prev_out_ = 0;
        lp_prev_out_ = 0; last_filtered_ = 0;
        last_mean_ = 0; last_sigma_ = 0; last_ac_ = 0;
        last_thr_hi_ = 0; last_thr_lo_ = 0;
        schmitt_state_ = IDLE;
        peak_val_ = 0; peak_time_ms_ = 0; peak_time_us_ = 0;
        last_peak_time_ms_ = 0; refract_end_ms_ = 0;
        win_idx_ = 0; win_count_ = 0;
        for (int i = 0; i < MAX_WINDOW; i++) window_[i] = 0;
        interval_idx_ = 0; interval_cnt_ = 0;
        bpm_ = 0; hrv_ = 0;
        peak_read_ = 0; peak_write_ = 0;
    }

private:
    // --- filter coefficients ---
    float hp_alpha_    = 0.95f;
    float lp_alpha_    = 0.30f;
    float sample_rate_ = 100.0f;

    // --- filter state ---
    float hp_prev_in_  = 0;
    float hp_prev_out_ = 0;
    float lp_prev_out_ = 0;
    float last_filtered_ = 0;

    // --- sliding window ---
    float window_[MAX_WINDOW] = {};
    int   win_size_  = 200;
    int   win_idx_   = 0;
    int   win_count_ = 0;

    float last_mean_   = 0;
    float last_sigma_  = 0;
    float last_ac_     = 0;
    float last_thr_hi_ = 0;
    float last_thr_lo_ = 0;

    // --- Schmitt trigger ---
    float alpha_hi_      = 2.0f;
    float alpha_lo_      = 0.5f;
    float refractory_ms_ = 300.0f;

    enum SchmittState { IDLE, ARMED, REFRACTORY };
    SchmittState schmitt_state_ = IDLE;

    float    peak_val_        = 0;
    uint32_t peak_time_ms_    = 0;
    uint32_t peak_time_us_    = 0;
    uint32_t last_peak_time_ms_ = 0;
    uint32_t refract_end_ms_  = 0;

    // --- recent peaks ring buffer ---
    PeakInfo recent_peaks_[MAX_RECENT_PEAKS] = {};
    int peak_read_  = 0;
    int peak_write_ = 0;

    void storePeak(uint32_t t_us, float val, float interval) {
        recent_peaks_[peak_write_] = {t_us, val, interval};
        peak_write_ = (peak_write_ + 1) % MAX_RECENT_PEAKS;
        if (peak_write_ == peak_read_)
            peak_read_ = (peak_read_ + 1) % MAX_RECENT_PEAKS;
    }

    // --- BPM / HRV ---
    static const int MAX_INTERVALS = 12;
    float intervals_[MAX_INTERVALS] = {};
    int   interval_idx_ = 0;
    int   interval_cnt_ = 0;
    float bpm_ = 0;
    float hrv_ = 0;

    void addInterval(float interval_ms) {
        if (interval_ms < 270 || interval_ms > 2000) return;

        intervals_[interval_idx_] = interval_ms;
        interval_idx_ = (interval_idx_ + 1) % MAX_INTERVALS;
        if (interval_cnt_ < MAX_INTERVALS) interval_cnt_++;

        float sum = 0;
        for (int i = 0; i < interval_cnt_; i++) sum += intervals_[i];
        float mean = sum / interval_cnt_;
        bpm_ = 60000.0f / mean;

        if (interval_cnt_ >= 2) {
            float var = 0;
            for (int i = 0; i < interval_cnt_; i++) {
                float d = intervals_[i] - mean;
                var += d * d;
            }
            hrv_ = sqrtf(var / interval_cnt_);
        }
    }
};
