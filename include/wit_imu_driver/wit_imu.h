#ifndef WIT_IMU_DRIVER_WIT_IMU_H
#define WIT_IMU_DRIVER_WIT_IMU_H

#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace wit_imu_driver
{

class WitImu
{
public:
    enum PRODUCT
    {
        WT901C
    };

    explicit WitImu(const double co_gravity = 9.8, const size_t msg_buffer_size = 100)
    : buf_(1024)
    , co_gravity_(co_gravity)
    , msg_buf_max_(msg_buffer_size)
    {
    }

    virtual void pushBytes(const std::vector<uint8_t>& bytes,
                            const size_t size,
                            const rclcpp::Time& stamp) = 0;

    bool popImuData(sensor_msgs::msg::Imu* const p_msg)
    {
        if (imu_buf_.empty())
        {
            return false;
        }
        *p_msg = imu_buf_.front();
        imu_buf_.pop();
        return true;
    };

    size_t sizeImuData() const
    {
        return imu_buf_.size();
    };

    bool popTempData(sensor_msgs::msg::Temperature* const p_msg)
    {
        if (temp_buf_.empty())
        {
            return false;
        }
        *p_msg = temp_buf_.front();
        temp_buf_.pop();
        return true;
    };

    size_t sizeTempData() const
    {
        return temp_buf_.size();
    };

    bool popMagData(sensor_msgs::msg::MagneticField* const p_msg)
    {
        if (mag_buf_.empty())
        {
            return false;
        }
        *p_msg = mag_buf_.front();
        mag_buf_.pop();
        return true;
    };

    size_t sizeMagData() const
    {
        return mag_buf_.size();
    };

    virtual std::vector<uint8_t> genYawClr() const = 0;
    virtual std::vector<uint8_t> genHightClr() const = 0;
    virtual std::vector<uint8_t> genAccCal() const = 0;
    virtual std::vector<uint8_t> genMagCal() const = 0;
    virtual std::vector<uint8_t> genExitCal() const = 0;
protected:
    std::vector<uint8_t> buf_;
    const double co_gravity_;
    const size_t msg_buf_max_;
    std::queue<sensor_msgs::msg::Imu> imu_buf_;
    std::queue<sensor_msgs::msg::Temperature> temp_buf_;
    std::queue<sensor_msgs::msg::MagneticField> mag_buf_;

    static int bytes2int(const uint8_t h, const uint8_t l)
    {
        return static_cast<int16_t>(
                ((static_cast<uint16_t>(h) << 8) & 0xFF00)
                | (static_cast<uint16_t>(l) & 0x00FF));
    }
};
}   // namespace wit_imu_driver

#endif  // WIT_IMU_DRIVER_WIT_IMU_H
