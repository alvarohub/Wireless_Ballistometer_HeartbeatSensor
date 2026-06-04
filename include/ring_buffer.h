#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <cmath>

// Single accelerometer sample with timestamp
struct AccelSample {
    uint32_t timestamp_us;  // micros() at time of read
    float ax, ay, az;       // raw accelerometer (g's)
    float magnitude;        // sqrt(ax^2 + ay^2 + az^2)
};

// Lock-free Single-Producer Single-Consumer ring buffer
// Producer: sampling task (Core 0)
// Consumer: main loop (Core 1) for WebSocket sending
template<typename T, size_t SIZE>
class RingBuffer {
public:
    // Push an item. Returns false if buffer is full (item dropped).
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) % SIZE;
        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // full — oldest data preserved, newest dropped
        }
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Pop an item. Returns false if buffer is empty.
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // empty
        }
        item = buffer_[tail];
        tail_.store((tail + 1) % SIZE, std::memory_order_release);
        return true;
    }

    // Number of items currently in the buffer
    size_t available() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        if (head >= tail) return head - tail;
        return SIZE - tail + head;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    bool full() const {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) % SIZE;
        return next == tail_.load(std::memory_order_acquire);
    }

    void clear() {
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
    }

    size_t capacity() const { return SIZE - 1; }

private:
    T buffer_[SIZE];
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
};
