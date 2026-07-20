#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

#include <algorithm>
#include <cerrno>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

namespace {

struct LogEntry {
    std::string node_name;
    std::string line;
};

bool directoryExists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool makeDirectories(const std::string& path) {
    if (path.empty() || directoryExists(path)) return true;

    std::string current;
    if (path[0] == '/') current = "/";

    std::stringstream ss(path);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (part.empty()) continue;
        if (!current.empty() && current[current.size() - 1] != '/') current += "/";
        current += part;

        if (directoryExists(current)) continue;
        if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }

    return directoryExists(path);
}

std::string dirnameOf(const std::string& path) {
    const std::string::size_type pos = path.find_last_of('/');
    if (pos == std::string::npos) return ".";
    if (pos == 0) return "/";
    return path.substr(0, pos);
}

std::string homeDir() {
    const char* home = getenv("HOME");
    return home && *home ? std::string(home) : std::string("/tmp");
}

std::string envValue(const char* name) {
    const char* value = getenv(name);
    return value && *value ? std::string(value) : std::string();
}

std::string resolveLogDirectory(const ros::NodeHandle& nh) {
    const std::string log_filename = envValue("ROS_LOG_FILENAME");
    if (!log_filename.empty()) return dirnameOf(log_filename);

    std::string run_id;
    std::string log_dir = envValue("ROS_LOG_DIR");
    if (log_dir.empty()) log_dir = homeDir() + "/.ros/log";
    if (nh.getParam("/run_id", run_id) && !run_id.empty()) return log_dir + "/" + run_id;

    const std::string latest = log_dir + "/latest";
    if (directoryExists(latest)) return latest;
    return log_dir;
}

std::string sanitizeNodeName(const std::string& name) {
    std::string cleaned = name;
    while (!cleaned.empty() && cleaned[0] == '/') cleaned.erase(cleaned.begin());
    if (cleaned.empty()) cleaned = "unknown";

    for (char& ch : cleaned) {
        const bool allowed = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                             (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.';
        if (!allowed) ch = '_';
    }
    return cleaned;
}

std::string levelName(uint8_t level) {
    switch (level) {
        case rosgraph_msgs::Log::DEBUG:
            return "DEBUG";
        case rosgraph_msgs::Log::INFO:
            return "INFO";
        case rosgraph_msgs::Log::WARN:
            return "WARN";
        case rosgraph_msgs::Log::ERROR:
            return "ERROR";
        case rosgraph_msgs::Log::FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}

std::string formatTime(const ros::Time& stamp) {
    const time_t seconds = static_cast<time_t>(stamp.sec);
    struct tm local_time;
    localtime_r(&seconds, &local_time);

    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << "." << std::setw(9)
        << std::setfill('0') << stamp.nsec;
    return oss.str();
}

std::string formatStartTimeDirectoryName() {
    const time_t seconds = time(nullptr);
    struct tm local_time;
    localtime_r(&seconds, &local_time);

    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

std::string compactLocation(const rosgraph_msgs::Log& msg) {
    const std::string file = msg.file.empty() ? "?" : msg.file;
    const std::string function = msg.function.empty() ? "?" : msg.function;

    std::ostringstream oss;
    oss << file << ":" << function << ":" << msg.line;
    return oss.str();
}

class RosoutFileLogger {
   public:
    explicit RosoutFileLogger(ros::NodeHandle nh) : nh_(std::move(nh)) {
        ros::NodeHandle pnh("~");
        pnh.param("flush_period", flush_period_, 0.5);
        pnh.param("max_queue_size", max_queue_size_, 10000);
        pnh.param("max_batch_size", max_batch_size_, 1000);
        pnh.param("flush_streams", flush_streams_, true);
        pnh.param("include_node_name", include_node_name_, true);
        pnh.param("include_location", include_location_, true);

        if (flush_period_ <= 0.0) flush_period_ = 0.5;
        if (max_queue_size_ <= 0) max_queue_size_ = 10000;
        if (max_batch_size_ <= 0) max_batch_size_ = 1000;

        const std::string ros_log_dir = resolveLogDirectory(nh_);
        log_dir_ = ros_log_dir + "/" + formatStartTimeDirectoryName();
        if (!makeDirectories(log_dir_)) {
            ROS_FATAL("Failed to create rosout file logger directory: %s", log_dir_.c_str());
            throw std::runtime_error("failed to create log directory");
        }

        sub_ = nh_.subscribe("/rosout_agg", 1000, &RosoutFileLogger::logCallback, this);
        flush_timer_ = nh_.createTimer(
            ros::Duration(flush_period_), &RosoutFileLogger::flushTimerCallback, this);
        ROS_INFO(
            "rosout_file_logger writing per-node logs to %s, flush_period=%.3fs, "
            "max_queue_size=%d, max_batch_size=%d, include_node_name=%s, include_location=%s",
            log_dir_.c_str(), flush_period_, max_queue_size_, max_batch_size_,
            include_node_name_ ? "true" : "false", include_location_ ? "true" : "false");
    }

    ~RosoutFileLogger() {
        flushAll();
    }

   private:
    void logCallback(const rosgraph_msgs::Log::ConstPtr& msg) {
        LogEntry entry;
        entry.node_name = msg->name;
        entry.line      = formatLine(*msg);

        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (static_cast<int>(queue_.size()) >= max_queue_size_) {
            queue_.pop_front();
            ++dropped_count_;
        }
        queue_.push_back(std::move(entry));
    }

    void flushTimerCallback(const ros::TimerEvent&) {
        flushBatch(max_batch_size_);
    }

    std::string formatLine(const rosgraph_msgs::Log& msg) const {
        std::ostringstream oss;
        oss << "[" << levelName(msg.level) << "] "
            << "[" << formatTime(msg.header.stamp) << "]";
        if (include_node_name_) {
            oss << " [" << (msg.name.empty() ? "/unknown" : msg.name) << "]";
        }
        if (include_location_) {
            oss << " [" << compactLocation(msg) << "]";
        }
        oss << " " << msg.msg << '\n';
        return oss.str();
    }

    void flushBatch(int max_count) {
        std::deque<LogEntry> batch;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            const int count = std::min(max_count, static_cast<int>(queue_.size()));
            for (int i = 0; i < count; ++i) {
                batch.push_back(std::move(queue_.front()));
                queue_.pop_front();
            }
        }

        if (batch.empty()) return;

        for (const LogEntry& entry : batch) {
            std::ofstream* file = streamFor(entry.node_name);
            if (!file || !file->good()) continue;
            *file << entry.line;
        }

        if (flush_streams_) {
            for (auto& pair : streams_) {
                pair.second->flush();
            }
        }

        reportDroppedIfNeeded();
    }

    void flushAll() {
        while (true) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (queue_.empty()) break;
            }
            flushBatch(max_batch_size_);
        }

        for (auto& pair : streams_) {
            pair.second->flush();
        }
    }

    void reportDroppedIfNeeded() {
        size_t dropped = 0;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (dropped_count_ == last_reported_dropped_) return;
            dropped = dropped_count_;
            last_reported_dropped_ = dropped_count_;
        }

        ROS_WARN("rosout_file_logger dropped %zu log messages because the queue was full.", dropped);
    }

    std::ofstream* streamFor(const std::string& node_name) {
        const std::string key = sanitizeNodeName(node_name);
        auto it = streams_.find(key);
        if (it != streams_.end()) return it->second.get();

        const std::string path = log_dir_ + "/" + key + ".log";
        std::unique_ptr<std::ofstream> stream(new std::ofstream(path.c_str(), std::ios::app));
        if (!stream->is_open()) {
            ROS_ERROR("Failed to open per-node rosout log file: %s", path.c_str());
            return nullptr;
        }

        std::ofstream* raw = stream.get();
        streams_[key] = std::move(stream);
        return raw;
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Timer flush_timer_;
    std::string log_dir_;
    std::map<std::string, std::unique_ptr<std::ofstream>> streams_;
    std::deque<LogEntry> queue_;
    std::mutex queue_mutex_;
    double flush_period_ = 0.5;
    int max_queue_size_ = 10000;
    int max_batch_size_ = 1000;
    bool flush_streams_ = true;
    bool include_node_name_ = true;
    bool include_location_ = true;
    size_t dropped_count_ = 0;
    size_t last_reported_dropped_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosout_file_logger");
    ros::NodeHandle nh;

    try {
        RosoutFileLogger logger(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("rosout_file_logger failed: %s", e.what());
        return 1;
    }

    return 0;
}
