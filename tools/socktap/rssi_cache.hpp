#pragma once
#include <vanetza/net/mac_address.hpp>
#include <map>
#include <chrono>
#include <mutex>
#include <boost/optional.hpp>

struct RSSICache {
    struct Entry {
        int16_t rssi_dbm;
        uint32_t src_l2id;
        vanetza::MacAddress mac;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    static RSSICache& instance() {
        static RSSICache cache;
        return cache;
    }
    
    void store(const vanetza::MacAddress& mac, int16_t rssi_dbm, uint32_t src_l2id) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Store by MAC
        mac_cache_[mac] = Entry{rssi_dbm, src_l2id, mac, std::chrono::steady_clock::now()};
        // Also make the last entry accessible without needing MAC
        last_entry_ = Entry{rssi_dbm, src_l2id, mac, std::chrono::steady_clock::now()};
    }
    
    boost::optional<Entry> retrieve(const vanetza::MacAddress& mac) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mac_cache_.find(mac);
        if (it != mac_cache_.end()) {
            auto age = std::chrono::steady_clock::now() - it->second.timestamp;
            if (age < std::chrono::seconds(2)) {
                return it->second;
            }
        }
        return boost::none;
    }
    
    boost::optional<Entry> get_last() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_entry_) {
            auto age = std::chrono::steady_clock::now() - last_entry_->timestamp;
            if (age < std::chrono::seconds(2)) {
                return last_entry_;
            }
        }
        return boost::none;
    }
    
private:
    std::map<vanetza::MacAddress, Entry> mac_cache_;
    boost::optional<Entry> last_entry_;
    std::mutex mutex_;
};

