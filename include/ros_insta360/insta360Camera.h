#ifndef INSTA360VIDEOTOFRAMES_INSTA360CAMERA_H
#define INSTA360VIDEOTOFRAMES_INSTA360CAMERA_H

#include "videoDecoder.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros_insta360/camera/camera.h>
#include <ros_insta360/camera/photography_settings.h>
#include <ros_insta360/camera/device_discovery.h>
#include <mutex>

class MyStreamDelegate: public ins_camera::StreamDelegate {
public:
    MyStreamDelegate() {
    }
    ~MyStreamDelegate() {
    }
    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
    }
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
    }
    void OnExposureData(const ins_camera::ExposureData& data) override {
    }

    void OnVideoData(const uint8_t* data, size_t data_size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        if (data_size && stream_index==0) {
            while (data_size > 0) {
                ret = vd.parse(data, data_size);
                data += ret;
                data_size -= ret;
                if (vd.pkt->data){
                    cv::Mat Itmp = vd.decode(data, data_size);
                    if(Itmp.rows!=0) {
                        mu.lock();
                        I=Itmp.clone();
                        mu.unlock();
                    }
                }
            }
        }
    }

    cv::Mat getImage(){
        cv::Mat Itmp;
        mu.lock();
        Itmp = I;
        mu.unlock();
        return Itmp;
    }

private:
    videoDecoder vd;
    int ret;
    cv::Mat I;
    std::mutex mu;
};



class insta360Camera {
private :
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<ins_camera::StreamDelegate> delegate;
    ins_camera::DeviceDiscovery discovery;

public :
    insta360Camera(ins_camera::VideoResolution res=ins_camera::VideoResolution::RES_720_406P30);
    ~insta360Camera();

    void displayInfo();
    cv::Mat getRLImage();

    void extractLeftAndRightImages(cv::Mat I, cv::Mat &L, cv::Mat &R, bool reoriented = true);



};


#endif //INSTA360VIDEOTOFRAMES_INSTA360CAMERA_H
