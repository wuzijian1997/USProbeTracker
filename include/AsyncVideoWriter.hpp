#pragma once

#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <vector>

namespace UBCRCL {
class AsyncVideoWriter
{
public:
    AsyncVideoWriter()
        : frameQueue_(kMaxQueueSize_)
    {}

    void open(const std::string &filename,
              int fourcc,
              double fps,
              const cv::Size &frameSize,
              const std::vector<int> &params)
    {
        if (!writer_.isOpened()) {
            writer_.open(filename, fourcc, fps, frameSize, params);
            writerThread_ = std::thread(&AsyncVideoWriter::processQueue, this);
        }
    }

    void open(const std::string &filename,
              int apiPreference,
              int fourcc,
              double fps,
              const cv::Size &frameSize,
              const std::vector<int> &params)
    {
        if (!writer_.isOpened()) {
            writer_.open(filename, apiPreference, fourcc, fps, frameSize, params);
            writerThread_ = std::thread(&AsyncVideoWriter::processQueue, this);
        }
    }

    ~AsyncVideoWriter() { release(); }

    void release()
    {
        // stop and join the thread
        stopThread_ = true;
        condVar_.notify_all();
        if (writerThread_.joinable()) {
            writerThread_.join();
        }
        // release the video writer
        if (writer_.isOpened())
            writer_.release();
    }

    void write(const cv::Mat &frame)
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (frameQueue_.size() >= kMaxQueueSize_)
            frameQueue_.pop_front();
        frameQueue_.push_back(frame);
        condVar_.notify_one();
    }

private:
    void processQueue()
    {
        while (!stopThread_) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lock(queueMutex_);
                condVar_.wait(lock, [this]() { return stopThread_ || !frameQueue_.empty(); });
                if (!frameQueue_.empty()) {
                    frame = frameQueue_.front();
                    frameQueue_.pop_front();
                }
            }
            if (!frame.empty() && writer_.isOpened()) {
                writer_.write(frame);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    cv::VideoWriter writer_;
    std::string fileName_;
    std::deque<cv::Mat> frameQueue_;
    std::thread writerThread_;
    std::mutex queueMutex_;
    std::condition_variable condVar_;
    std::atomic<bool> stopThread_{false};
    static constexpr uint16_t kMaxQueueSize_ = 100; // Maximum size of the frame queue
};
} // namespace UBCRCL