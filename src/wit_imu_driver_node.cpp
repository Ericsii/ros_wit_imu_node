#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <numeric>
#include <string>
#include <chrono>
#include <linux/serial.h>

// #include "rclcpp/rclcpp.hpp"
// #include <ros/console.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/MagneticField.h>
// #include <sensor_msgs/Temperature.h>
// #include <std_srvs/Trigger.h>
#include "std_srvs/srv/trigger.hpp"

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <wit_imu_driver/wit_imu.h>
#include <wit_imu_driver/wt901c.h>

using namespace std::chrono;

namespace wit_imu_driver
{
namespace ba = boost::asio;

enum TrgState{
    YAWCLR,
    ACCCAL,
    MAGCAL,
    EXICAL
};

class WitImuDriver
{
    using imu_msg = sensor_msgs::msg::Imu;
    using temp_msg = sensor_msgs::msg::Temperature;
    using mag_msg = sensor_msgs::msg::MagneticField;
    using trigger = std_srvs::srv::Trigger;
public:
    WitImuDriver(rclcpp::Node::SharedPtr node)
    : node_(node)
    , port_io_()
    , port_(port_io_)
    , wdg_timeout_(0.5s)
    , rx_buf_(1024)
    {
        node_->declare_parameter("gravity", 9.797673);
        node_->declare_parameter("frame_id", "imu_link");
        node_->declare_parameter("product", (int)WitImu::PRODUCT::WT901C);
        node_->declare_parameter("device", "/dev/ttyUSB0");
        node_->declare_parameter("baud", 9600);
        // pnh_.param<double>("gravity", co_gravity_, 9.797673);
        // pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
        // pnh_.param<int>("product", product_, WitImu::PRODUCT::WT901C);
    }

    bool open()
    {
        std::string dev;
        int baud;
        dev = node_->get_parameter("device").as_string();
        baud = node_->get_parameter("baud").as_int();
        // pnh_.param<std::string>("device", dev, "/dev/ttyUSB0");
        // pnh_.param<int>("baud", baud, 9600);

        boost::system::error_code ec;
        port_.open(dev, ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to open %s. Error code : %d",
                dev.c_str(),
                ec.value()
            );
            return false;
        }
        port_.set_option(ba::serial_port_base::baud_rate(baud), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to set baudrate. Error code: %d",
                ec.value()
            );
            return false;
        }
        port_.set_option(ba::serial_port_base::character_size(8), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to set options. Error code: %d",
                ec.value()
            );
            return false;
        }
        port_.set_option(ba::serial_port_base::flow_control(
                            ba::serial_port_base::flow_control::none), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to set options. Error code: %d",
                ec.value()
            );
            return false;
        }
        port_.set_option(ba::serial_port_base::parity(
                            ba::serial_port_base::parity::none), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to set options. Error code: %d",
                ec.value()
            );
            return false;
        }
        port_.set_option(ba::serial_port_base::stop_bits(
                            ba::serial_port_base::stop_bits::one), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to set options. Error code: %d",
                ec.value()
            );
            return false;
        }
        // FTDI USB-serial device has 16 msec latency in default.
        // It makes large latency and jitter for high rate measurement.
        const int fd = port_.native_handle();
        serial_struct port_info;
        ioctl(fd, TIOCGSERIAL, &port_info);
        port_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd, TIOCSSERIAL, &port_info);

        return true;
    }

    bool spin()
    {
        product_ = node_->get_parameter("product").as_int();
        switch (product_)
        {
            case WitImu::PRODUCT::WT901C:
            {
                co_gravity_ = node_->get_parameter("gravity").as_double();
                
                pub_imu_ = node_->create_publisher<imu_msg>("data_raw", 10);
                pub_temp_ = node_->create_publisher<temp_msg>("temperature", 10);
                pub_mag_ = node_->create_publisher<mag_msg>("mag", 10);
                ptr_imu_ = std::make_shared<Wt901c>(co_gravity_);

                std::function<void(const trigger::Request::SharedPtr, const trigger::Response::SharedPtr)> callback = \
                    std::bind(&WitImuDriver::cbSrvTrgCommand, 
                    this,
                    TrgState::YAWCLR,
                    std::placeholders::_1,
                    std::placeholders::_2);
                srv_trg_yaw_clr_ = node_->create_service<trigger>("trigger_yaw_clear", callback);

                callback = std::bind(&WitImuDriver::cbSrvTrgCommand, 
                                     this,
                                     TrgState::ACCCAL,
                                     std::placeholders::_1,
                                     std::placeholders::_2);
                srv_trg_acc_cal_ = node_->create_service<trigger>("trigger_acc_calibration", callback);

                callback = std::bind(&WitImuDriver::cbSrvTrgCommand, 
                                     this,
                                     TrgState::MAGCAL,
                                     std::placeholders::_1,
                                     std::placeholders::_2);
                srv_trg_mag_cal_ = node_->create_service<trigger>("trigger_mag_calibration", callback);

                callback = std::bind(&WitImuDriver::cbSrvTrgCommand, 
                                     this,
                                     TrgState::EXICAL,
                                     std::placeholders::_1,
                                     std::placeholders::_2);
                srv_trg_exit_cal_ = node_->create_service<trigger>("trigger_exit_calibration", callback);
            }
            break;

            default:
            RCLCPP_ERROR(node_->get_logger(), "Product is not provided");
            return false;
        }
        startRead();
        auto io_run = [this]()
        {
            boost::system::error_code ec;
            port_io_.run(ec);
        };
        io_thread_ = std::thread(io_run);

        wdg_ = node_->create_wall_timer(wdg_timeout_.to_chrono<std::chrono::milliseconds>(), std::bind(&WitImuDriver::cbWdg, this));

        // wdg_ = pnh_.createTimer(wdg_timeout_, &WitImuDriver::cbWdg, this, true, false);

        rclcpp::spin(node_);
        close();
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    ba::io_service port_io_;
    ba::serial_port port_;
    rclcpp::Duration wdg_timeout_;
    rclcpp::TimerBase::SharedPtr wdg_;

    rclcpp::Publisher<imu_msg>::SharedPtr pub_imu_;
    rclcpp::Publisher<temp_msg>::SharedPtr pub_temp_;
    rclcpp::Publisher<mag_msg>::SharedPtr pub_mag_;
    rclcpp::Service<trigger>::SharedPtr srv_trg_yaw_clr_;
    rclcpp::Service<trigger>::SharedPtr srv_trg_height_clr_;
    rclcpp::Service<trigger>::SharedPtr srv_trg_acc_cal_;
    rclcpp::Service<trigger>::SharedPtr srv_trg_mag_cal_;
    rclcpp::Service<trigger>::SharedPtr srv_trg_exit_cal_;

    std::string frame_id_;
    std::thread io_thread_;

    double co_gravity_;
    int product_;
    std::vector<uint8_t> rx_buf_;

    std::shared_ptr<WitImu> ptr_imu_;

    void close()
    {
        if (port_.is_open())
        {
            port_.cancel();
            port_.close();
        }
        if (!port_io_.stopped())
        {
            port_io_.stop();
        }
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
    }

    void startRead()
    {
        ba::async_read(port_,
                    ba::buffer(rx_buf_),
                    ba::transfer_at_least(1),
                    boost::bind(&WitImuDriver::cbPort,
                                this,
                                ba::placeholders::error,
                                ba::placeholders::bytes_transferred));
    }

    void cbPort(const boost::system::error_code& ec,
                std::size_t size)
    {
        if (!ec)
        {
            frame_id_ = node_->get_parameter("frame_id").as_string();
            // ptr_imu_->pushBytes(rx_buf_, size, ros::Time::now());
            ptr_imu_->pushBytes(rx_buf_, size, rclcpp::Clock().now());
            while (ptr_imu_->sizeImuData() != 0)
            {
                imu_msg msg;
                ptr_imu_->popImuData(&msg);
                msg.header.frame_id = frame_id_;
                pub_imu_->publish(msg);
            }
            while (ptr_imu_->sizeTempData() != 0)
            {
                temp_msg msg;
                ptr_imu_->popTempData(&msg);
                msg.header.frame_id = frame_id_;
                pub_temp_->publish(msg);
            }
            while (ptr_imu_->sizeMagData() != 0)
            {
                mag_msg msg;
                ptr_imu_->popMagData(&msg);
                msg.header.frame_id = frame_id_;
                pub_mag_->publish(msg);
            }
            startRead();
            resetWdg();
        }
        else if (ec == boost::system::errc::operation_canceled)
        {
            // Enter to this state when stop a connection
        }
        else
        {
            // Unknown state
            RCLCPP_ERROR(node_->get_logger(), "seiral error : %s", ec.message().c_str());
            rclcpp::shutdown();
        }
    }

    void cbSrvTrgCommand(TrgState state,
                         const std::shared_ptr<trigger::Request> req,
                         const std::shared_ptr<trigger::Response> res)
    {
        (void) req;
        bool ret = false;
        switch(state){
            case YAWCLR:
                ret = sendBytes(ptr_imu_->genYawClr());
                break;
            case ACCCAL:
                ret = sendBytes(ptr_imu_->genAccCal());
                break;
            case MAGCAL:
                ret = sendBytes(ptr_imu_->genMagCal());
                break;
            case EXICAL:
                ret = sendBytes(ptr_imu_->genExitCal());
                break;
            default:
                break;
        }
        if (ret)
        {
            res->message = "Success";
            res->success = true;
        }
        else
        {
            res->message = "Failed";
            res->success = false;
        }
    }

    // void cbSrvTrgWriteCommand(const std::shared_ptr<trigger::Request> req,
    //                           const std::shared_ptr<trigger::Response> res,
    //                           const std::vector<uint8_t>& bytes)
    // {
    //     (void) req;
    //     bool ret = sendBytes(bytes);
    //     if (ret)
    //     {
    //         res->message = "Success";
    //         res->success = true;
    //     }
    //     else
    //     {
    //         res->message = "Failed";
    //         res->success = false;
    //     }
    // }

    void cbWdg()
    {
        if (port_.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(), "Timeouted. No data received from IMU.");
            rclcpp::shutdown();
        }
    }

    void resetWdg()
    {
        wdg_->cancel();
        wdg_->reset();
    }

    bool sendBytes(const std::vector<uint8_t>& bytes)
    {
        boost::system::error_code ec;
        const size_t w_len = ba::write(port_,
                                    ba::buffer(bytes),
                                    ec);
        if (w_len != bytes.size())
        {
            RCLCPP_WARN(node_->get_logger(), "Could not send full length of packet.");
            return false;
        }
        else if (ec.value() != 0)
        {
            RCLCPP_WARN(node_->get_logger(), 
                    "Failed to write. Error code : %d",
                    ec.value());
            return false;
        }
        return true;
    }
};
}   // namespace wit_imu_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wit_imu_driver");
    // ros::init(argc, argv, "wit_imu_driver");
    RCLCPP_INFO(node->get_logger(), "Start wit_imu_driver");

    wit_imu_driver::WitImuDriver imu(node);
    if (!imu.open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("wit_imu_driver"), "Failed to open.");
        return 1;
    }

    if (!imu.spin())
    {
        RCLCPP_ERROR(rclcpp::get_logger("wit_imu_driver"), "Exit by error");
        return 1;
    }

    return 0;
}
