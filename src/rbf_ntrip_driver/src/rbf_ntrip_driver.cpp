#include "rbf_ntrip_driver/rbf_ntrip_driver.h"

namespace rbf_ntrip_driver
{

    NtripDriver::NtripDriver(const rclcpp::NodeOptions &options) : Node("rbf_ntrip_driver", options)
    {
        load_parameters();

        // Configure and start the NTRIP client
        ntrip_client_ptr_ = std::make_shared<libntrip::NtripClient>(config_.ntrip.host, config_.ntrip.port, config_.ntrip.username, config_.ntrip.password, config_.ntrip.mountpoint);
        diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
        diagnostic_updater_->setHardwareID("NTRIP DRIVER");
        diagnostic_updater_->add("NTRIP Client Status", this, &NtripDriver::diagnostic_callback);

        ntrip_client_ptr_->OnReceived(std::bind(&NtripDriver::ntrip_client_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Configure and open the serial port publisher if enabled
        if (config_.serial_port.publish_port_rtcm)
        {
            try
            {
                serial_port_ptr_ = std::make_shared<SerialPort>(config_.serial_port.port.c_str());
                serial_port_ptr_->open();
                serial_port_ptr_->configure(config_.serial_port.baudrate, 8, 'N', 1);
            }
            catch (const SerialPortException &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                rclcpp::shutdown();
            }
        }

        // Create the RTCM publisher if enabled
        if (config_.rtcm_publisher.publish_rtcm)
        {
            // pub_rtcm_ = this->create_publisher<mavros_msgs::msg::RTCM>(config_.rtcm_publisher.topic_name, 10);
            rtcm_msg_pub = this->create_publisher<rtcm_msgs::msg::Message>("/sbg_device1/ntrip_client/rtcm", 10);
            rtcm_msg_pub2 = this->create_publisher<rtcm_msgs::msg::Message>("/sbg_device2/ntrip_client/rtcm2", 10);
        }
        // Subscribe to NAV-SAT-FIX if it's used initially, Otherwise, try to establish the NTRIP connection
        if (config_.ntrip.use_nav_sat_fix_init)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for NavSatFix message to initialize NTRIP client...");
            sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                config_.ntrip.nav_sat_fix_topic_name, 10, std::bind(&NtripDriver::nav_sat_fix_callback, this, std::placeholders::_1));
        }
        else
        {
            try_to_ntrip_connect();
        }
    }

    void NtripDriver::load_parameters()
    {
        config_.ntrip.host = declare_parameter("ntrip.host", "ntrip.example.com");
        config_.ntrip.port = declare_parameter("ntrip.port", 2101);
        config_.ntrip.mountpoint = declare_parameter("ntrip.mount_point", "VRSRTCM31");
        config_.ntrip.username = declare_parameter("ntrip.user_name", "username");
        config_.ntrip.password = declare_parameter("ntrip.password", "password");
        config_.ntrip.use_nav_sat_fix_init = declare_parameter("ntrip.use_nav_sat_fix_init", false);
        config_.ntrip.nav_sat_fix_topic_name = declare_parameter("ntrip.nav_sat_fix_topic_name", "/fix");
        config_.ntrip.init_lat_position = declare_parameter("ntrip.init_ntrip_location_lat", 0.0);
        config_.ntrip.init_lon_position = declare_parameter("ntrip.init_ntrip_location_lon", 0.0);

        config_.serial_port.port = declare_parameter("serial_port.name", "/dev/ttyUSB0");
        config_.serial_port.baudrate = declare_parameter("serial_port.baud_rate", 9600);
        config_.serial_port.publish_port_rtcm = declare_parameter("serial_port.publish_port_rtcm", false);

        config_.rtcm_publisher.topic_name = declare_parameter("rtcm_publisher.rtcm_topic", "rtcm");
        config_.rtcm_publisher.publish_rtcm = declare_parameter("rtcm_publisher.publish_rtcm", true);
        config_.rtcm_publisher.frame_id = declare_parameter("rtcm_publisher.frame_id", "rtcm");

        RCLCPP_INFO(this->get_logger(), "---------NTRIP CONFIGURATION--------");
        RCLCPP_INFO(this->get_logger(), "host: %s", config_.ntrip.host.c_str());
        RCLCPP_INFO(this->get_logger(), "port: %d", config_.ntrip.port);
        RCLCPP_INFO(this->get_logger(), "mountpoint: %s", config_.ntrip.mountpoint.c_str());
        RCLCPP_INFO(this->get_logger(), "username: %s", config_.ntrip.username.c_str());
        RCLCPP_INFO(this->get_logger(), "password: %s", config_.ntrip.password.c_str());
        RCLCPP_INFO(this->get_logger(), "use_nav_sat_fix_init: %d", config_.ntrip.use_nav_sat_fix_init);
        RCLCPP_INFO(this->get_logger(), "nav_sat_fix_topic_name: %s", config_.ntrip.nav_sat_fix_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "init_lat_position: %f", config_.ntrip.init_lat_position);
        RCLCPP_INFO(this->get_logger(), "init_lon_position: %f", config_.ntrip.init_lon_position);
        RCLCPP_INFO(this->get_logger(), "---------SERIAL PORT CONFIGURATION--------");
        RCLCPP_INFO(this->get_logger(), "port: %s", config_.serial_port.port.c_str());
        RCLCPP_INFO(this->get_logger(), "baudrate: %d", config_.serial_port.baudrate);
        RCLCPP_INFO(this->get_logger(), "publish_port_rtcm: %d", config_.serial_port.publish_port_rtcm);
        RCLCPP_INFO(this->get_logger(), "---------RTCM PUBLISHER CONFIGURATION--------");
        RCLCPP_INFO(this->get_logger(), "topic_name: %s", config_.rtcm_publisher.topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "publish_rtcm: %d", config_.rtcm_publisher.publish_rtcm);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", config_.rtcm_publisher.frame_id.c_str());

        RCLCPP_INFO(this->get_logger(), "----------------------------------");
    }

    // Start the NTRIP client
    bool NtripDriver::run_ntrip()
    {
        if (!config_.ntrip.use_nav_sat_fix_init)
        {
            ntrip_client_ptr_->set_location(config_.ntrip.init_lat_position, config_.ntrip.init_lon_position);
        }
        return ntrip_client_ptr_->Run();
    }

    // Try to establish the NTRIP connection
    void NtripDriver::try_to_ntrip_connect()
    {
        constexpr int max_attempts = 120;
        int try_count = 0;
        // Try for a certain number of attempts or until successful
        while (try_count < max_attempts && rclcpp::ok())
        {
            if (run_ntrip())
            {
                ntrip_time_ = this->now();
                return;
            }
            // Retry after a delay if failed
            try_count++;
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        // Shutdown if maximum attempts reached or failed to initialize NTRIP client
        rclcpp::shutdown();
    }

    // Timer callback function
    void NtripDriver::diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!ntrip_client_ptr_->service_is_running())
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Try to connect to NTRIP server failed");
            ntrip_client_ptr_->Stop();
            if (config_.ntrip.use_nav_sat_fix_init)
            {
                if (nav_sat_fix_received_ == false)
                {
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "NavSatFix message not received");
                }
                if (sub_nav_sat_fix_ == nullptr)
                {
                    nav_sat_fix_received_ = false;
                    sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                        config_.ntrip.nav_sat_fix_topic_name, 10, std::bind(&NtripDriver::nav_sat_fix_callback, this, std::placeholders::_1));
                }
            }
            else
            {
                try_to_ntrip_connect();
            }
        }
        else
        {
            if ((this->now() - ntrip_time_).seconds() > 3)
            {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No RTCM data received");
                stat.add("Last RTCM data received(second)", (this->now() - ntrip_time_).seconds());
                ntrip_client_ptr_->Stop();
            }
            else
            {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "NTRIP Client is running");
            }
        }
    }

    // Callback for data received from the NTRIP client
    void NtripDriver::ntrip_client_callback(char const *_buffer, int _size)
    {
        ntrip_time_ = this->now();
        // auto rtcm_msg = mavros_msgs::msg::RTCM();
        // rtcm_msg.header.stamp = this->now();
        // rtcm_msg.header.frame_id = config_.rtcm_publisher.frame_id;
        // rtcm_msg.data = std::vector<uint8_t>(_buffer, _buffer + _size);

        auto rtcm_msg = rtcm_msgs::msg::Message();
        rtcm_msg.header.stamp = this->now();
        rtcm_msg.header.frame_id = config_.rtcm_publisher.frame_id;
        rtcm_msg.message = std::vector<uint8_t>(_buffer, _buffer + _size);

        if (config_.rtcm_publisher.publish_rtcm)
        {
            rtcm_msg_pub->publish(rtcm_msg);
            rtcm_msg_pub2->publish(rtcm_msg);
        }
        if (config_.serial_port.publish_port_rtcm)
        {
            serial_port_ptr_->write(_buffer, _size);
        }
    }

    // Callback for NAV-SAT-FIX data
    void NtripDriver::nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        ntrip_client_ptr_->set_location(msg->latitude, msg->longitude);
        nav_sat_fix_received_ = true;
        try_to_ntrip_connect();
        sub_nav_sat_fix_.reset();
    }

}; // namespace rbf_ntrip_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_ntrip_driver::NtripDriver)
