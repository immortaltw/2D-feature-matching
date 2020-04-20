#ifndef dataStructures_h
#define dataStructures_h

#include <stdexcept>
#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

template<typename T>
class DataBuffer {
private:
    int size_;
    int read_;
    int write_;
    int count_;
    std::vector<T> buf_;
public:
    DataBuffer(int size): size_(size), read_(0), write_(0), count_(0), buf_(size) {}

    void push(T& elem) {
        buf_[write_] = elem;
        write_ = (write_ + 1) % size_;

        if (count_ != size_) count_++;
    }

    void push_back(T&& elem) {
        push_back(elem);
    }

    T pop() {
        if (count_ == 0) throw "bad index";

        T ret = buf_[read_];
        read_ = (read_ + 1) % size_;
        count_--;
        return ret;
    }

    size_t size() { return count_; }

    T& at(int idx) {
        if ((idx >= 0 && idx >= count_) ||
            (idx < 0 && abs(idx) > count_)) {
            throw std::out_of_range("bad index");
        }

        if (idx >= 0) return buf_[(read_ + idx) % size_];
        else return buf_[((write_ + idx) >= 0)? write_ + idx: write_ + idx + count_];
    }
};

#endif /* dataStructures_h */
