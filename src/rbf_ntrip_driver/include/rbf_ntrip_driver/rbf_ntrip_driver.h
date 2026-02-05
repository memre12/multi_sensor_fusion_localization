#ifndef RBF_NTRIP_DRIVER_H_
#define RBF_NTRIP_DRIVER_H_

#include <rclcpp/rclcpp.hpp>
#include <rbf_ntrip_driver/serial_port.h>
#include <ntrip/ntrip_client.h>
#include <memory>

#include <mavros_msgs/msg/rtcm.hpp>
#include <rtcm_msgs/msg/message.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace rbf_ntrip_driver {

class NtripDriver : public rclcpp::Node {
public:
    NtripDriver(const rclcpp::NodeOptions & options);
    ~NtripDriver() override{
        if(serial_port_ptr_ != nullptr){
            serial_port_ptr_->close();
        }
        if(ntrip_client_ptr_ != nullptr){
            ntrip_client_ptr_->Stop();
        }
    }

    struct Config{
        struct Ntrip{
            std::string host;
            int port;
            std::string mountpoint;
            std::string username;
            std::string password;
            bool use_nav_sat_fix_init;
            std::string nav_sat_fix_topic_name;
            double init_lat_position;
            double init_lon_position;
        };
        struct SerialPort{
            std::string port;
            int baudrate;
            bool publish_port_rtcm;
        };
        struct RTCMPublisher{
            std::string topic_name;
            std::string frame_id;
            bool publish_rtcm;
        };
        Ntrip ntrip;
        SerialPort serial_port;
        RTCMPublisher rtcm_publisher;
    };
    Config config_;
private:
    std::shared_ptr<SerialPort> serial_port_ptr_;
    std::shared_ptr<libntrip::NtripClient> ntrip_client_ptr_;
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    bool nav_sat_fix_received_ = false;


    /*LOAD PARAMETERS*/
    void load_parameters();
    /*INIT NTRIP*/
    bool run_ntrip();
    void try_to_ntrip_connect();

    /*PUBLISHERS*/
    rclcpp::Publisher<mavros_msgs::msg::RTCM>::SharedPtr pub_rtcm_;
    rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_msg_pub;
    rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_msg_pub2;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diagnostic_;

    /*SUBSCRIBERS*/
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_nav_sat_fix_;

    /*CALLBACKS*/
    void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void ntrip_client_callback(char const* _buffer, int _size);
    void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /*TIMES*/
    rclcpp::Time ntrip_time_;
};

} // namespace rbf_ntrip_driver

#endif // RBF_NTRIP_DRIVER_HPP_

