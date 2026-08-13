#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

// Sliding achieved-rate measurement shared by robot live runs and conveyor PhysX. Forty buckets
// across four seconds refresh ten times per second without making a bursty workload unreadable.
// addSample returns true only when a new aggregate is ready for publication.
class RollingRateWindow {
public:
    static constexpr double kWindowSeconds = 4.0;
    static constexpr size_t kBucketCount = 40;

    void reset() {
        m_buckets = {};
        m_bucket = 0;
        m_bucketWallSeconds = 0.0;
    }

    bool addSample(double wallSeconds, double deliveredSimSeconds, double* outRate) {
        wallSeconds = std::max(0.0, wallSeconds);
        deliveredSimSeconds = std::max(0.0, deliveredSimSeconds);
        Bucket& bucket = m_buckets[m_bucket];
        bucket.wallSeconds += wallSeconds;
        bucket.simSeconds += deliveredSimSeconds;
        m_bucketWallSeconds += wallSeconds;
        if (m_bucketWallSeconds < kWindowSeconds / static_cast<double>(kBucketCount)) {
            return false;
        }

        // Moving into and clearing the oldest bucket makes the window slide. A partly filled ring
        // is valid: sum(sim)/sum(wall) is the achieved rate from the first published update onward.
        m_bucket = (m_bucket + 1) % kBucketCount;
        m_buckets[m_bucket] = Bucket();
        m_bucketWallSeconds = 0.0;
        double totalWallSeconds = 0.0;
        double totalSimSeconds = 0.0;
        for (const Bucket& entry : m_buckets) {
            totalWallSeconds += entry.wallSeconds;
            totalSimSeconds += entry.simSeconds;
        }
        if (!(totalWallSeconds > 0.0)) return false;
        if (outRate) *outRate = totalSimSeconds / totalWallSeconds;
        return true;
    }

private:
    struct Bucket {
        double wallSeconds = 0.0;
        double simSeconds = 0.0;
    };

    std::array<Bucket, kBucketCount> m_buckets{};
    size_t m_bucket = 0;
    double m_bucketWallSeconds = 0.0;
};
