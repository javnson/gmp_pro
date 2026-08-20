#ifndef CCTL_DSA_SPSC_RECORD_RING_HPP
#define CCTL_DSA_SPSC_RECORD_RING_HPP

#include <atomic>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace cctl::dsa
{

/**
 * Preallocated single-producer/single-consumer ring for fixed-size records.
 *
 * The producer and consumer never lock or allocate after initialize(). The
 * caller owns object lifetime and must use exactly one producer thread and one
 * consumer thread. try_push() returns false when the configured capacity is
 * full, allowing a real-time producer to choose a non-blocking drop policy.
 */
class spsc_record_ring
{
  public:
    void initialize(std::size_t byte_capacity, std::size_t record_size)
    {
        if (record_size == 0U || byte_capacity < record_size * 2U)
            throw std::invalid_argument("SPSC ring is smaller than two records");
        record_size_ = record_size;
        capacity_ = byte_capacity / record_size;
        storage_.assign(capacity_ * record_size_, std::byte{});
        head_.store(0U, std::memory_order_relaxed);
        tail_.store(0U, std::memory_order_relaxed);
    }

    bool try_push(const void *record) noexcept
    {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= capacity_)
            return false;
        std::memcpy(storage_.data() + (head % capacity_) * record_size_, record,
                    record_size_);
        head_.store(head + 1U, std::memory_order_release);
        return true;
    }

    bool try_pop(void *record) noexcept
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head)
            return false;
        std::memcpy(record, storage_.data() + (tail % capacity_) * record_size_,
                    record_size_);
        tail_.store(tail + 1U, std::memory_order_release);
        return true;
    }

    std::size_t size() const noexcept
    {
        return head_.load(std::memory_order_acquire) -
               tail_.load(std::memory_order_acquire);
    }

    std::size_t capacity() const noexcept
    {
        return capacity_;
    }

    std::size_t record_size() const noexcept
    {
        return record_size_;
    }

    std::size_t storage_bytes() const noexcept
    {
        return storage_.size();
    }

  private:
    std::vector<std::byte> storage_;
    std::size_t record_size_{};
    std::size_t capacity_{};
    alignas(64) std::atomic<std::size_t> head_{0U};
    alignas(64) std::atomic<std::size_t> tail_{0U};
};

} // namespace cctl::dsa

#endif // CCTL_DSA_SPSC_RECORD_RING_HPP
