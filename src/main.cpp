#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>

#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::vector<uint8_t> sps_;
    std::vector<uint8_t> pps_;

public:
    TestStreamDelegate(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        // Publisher for the compressed H.264 video stream
        compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/dual_fisheye/image/compressed", 
            rclcpp::QoS(10)
        );

        // Publisher for IMU data (remains the same)
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        RCLCPP_INFO(node_->get_logger(), "Publisher for compressed images and IMU created.");
    }

    virtual ~TestStreamDelegate() {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // We only care about the main video stream (index 0)
        if (stream_index == 0 && size > 0 && compressed_pub_) {
            bool has_parameter_set = false;
            for (size_t offset = 0; offset + 3 < size;) {
                size_t prefix = 0;
                if (offset + 4 <= size && data[offset] == 0 && data[offset + 1] == 0 &&
                    data[offset + 2] == 0 && data[offset + 3] == 1) {
                    prefix = 4;
                } else if (data[offset] == 0 && data[offset + 1] == 0 && data[offset + 2] == 1) {
                    prefix = 3;
                } else {
                    ++offset;
                    continue;
                }

                const size_t nal_start = offset + prefix;
                size_t nal_end = size;
                for (size_t next = nal_start; next + 3 < size; ++next) {
                    if ((data[next] == 0 && data[next + 1] == 0 && data[next + 2] == 1) ||
                        (next + 4 <= size && data[next] == 0 && data[next + 1] == 0 &&
                         data[next + 2] == 0 && data[next + 3] == 1)) {
                        nal_end = next;
                        break;
                    }
                }

                if (nal_start < nal_end) {
                    const uint8_t nal_type = data[nal_start] & 0x1f;
                    if (nal_type == 7) {
                        sps_.assign(data + offset, data + nal_end);
                        has_parameter_set = true;
                    } else if (nal_type == 8) {
                        pps_.assign(data + offset, data + nal_end);
                        has_parameter_set = true;
                    }
                }
                offset = nal_end;
            }

            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();

            // Set the header
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "camera_frame";

            // Set the format to H.264
            // The subscriber will need to know this to select the correct decoder.
            msg->format = "h264";

            // Late subscribers need the codec headers that were sent before
            // they connected. Repeat cached SPS/PPS until the next keyframe.
            if (!has_parameter_set && !sps_.empty() && !pps_.empty()) {
                msg->data.reserve(sps_.size() + pps_.size() + size);
                msg->data.insert(msg->data.end(), sps_.begin(), sps_.end());
                msg->data.insert(msg->data.end(), pps_.begin(), pps_.end());
                msg->data.insert(msg->data.end(), data, data + size);
            } else {
                msg->data.assign(data, data + size);
            }

            compressed_pub_->publish(std::move(msg));
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "imu_frame";
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;
            
            msg->linear_acceleration.x = gyro.ax * 9.80665;
            msg->linear_acceleration.y = gyro.ay * 9.80665;
            msg->linear_acceleration.z = gyro.az * 9.80665;

            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0; // Neutral orientation
            msg->orientation_covariance[0] = -1.0; // No orientation data available

            for (int i = 0; i < 9; i++)
            {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<rclcpp::Node> node_;

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    ~CameraWrapper() {
        if (cam) {
            cam->Close();
        }
    }

    int run_camera() {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No available camera devices found.");
            return -1;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");
        discovery.FreeDeviceDescriptors(list);

        std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>(node_);
        cam->SetStreamDelegate(delegate);

        auto start = time(NULL);
        cam->SyncLocalTimeToCamera(start);        
        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_720_406P30; //Change this line to edit the resolution
        //Possible resolutions (results may vary per model) are:
        //RES_3840_1920P30
        //RES_2560_1280P30
        //RES_1152_1152P30 (this will give 2304 x 1152 at 30 FPS)
        //RES_1920_960P30
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_720_406P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start live streaming.");
            return -1;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta_publisher");
    
    CameraWrapper camera(node);
    if (camera.run_camera() != 0) {
        rclcpp::shutdown();
        return -1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
