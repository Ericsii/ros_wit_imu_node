#include "wit_imu_driver/wt901c.h"

#include <queue>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"

namespace wit_imu_driver
{
Wt901c::Wt901c(const double co_gravity)
    : WitImu(co_gravity)
    , co_acc_(co_gravity_ * -16.0 / 32768.0)
    , co_avel_(M_PI * 2000.0 / (32768.0 * 180.0))
    , co_temp_(0.01)
    , co_mag_(1.0)
    , co_pose_(M_PI / 32768.0)
{
}

void Wt901c::pushBytes(const std::vector<uint8_t>& bytes,
                        const size_t size,
                        const rclcpp::Time& stamp)
{
    buf_.insert(buf_.cend(), bytes.cbegin(), bytes.cbegin() + size);
    if (buf_.size() < 11)
    {
        // Not enough data length
        return;
    }
    while (buf_.size() >= 11)
    {
        if (buf_[0] != 0x55)
        {
            buf_.erase(buf_.cbegin());
            continue;
        }
        uint8_t cs = std::accumulate(buf_.begin(), buf_.begin() + 10, 0);
        if (cs != buf_[10])
        {
            std::printf("[Wt901c] Checksum is not match. Rx : %02X, Calc : %02X\n", buf_[10], cs);
            buf_.erase(buf_.cbegin());
            continue;
        }
        switch (buf_[1])
        {
            case 0x51:
            work_imu_.header.stamp = stamp;
            work_imu_.linear_acceleration.x = - co_acc_ * bytes2int(buf_[3], buf_[2]);
            work_imu_.linear_acceleration.y = - co_acc_ * bytes2int(buf_[5], buf_[4]);
            work_imu_.linear_acceleration.z = - co_acc_ * bytes2int(buf_[7], buf_[6]);

            work_temp_.header.stamp = stamp;
            work_temp_.temperature = co_temp_ * bytes2int(buf_[9], buf_[8]);
            break;

            case 0x52:
            work_imu_.angular_velocity.x = co_avel_ * bytes2int(buf_[3], buf_[2]);
            work_imu_.angular_velocity.y = co_avel_ * bytes2int(buf_[5], buf_[4]);
            work_imu_.angular_velocity.z = co_avel_ * bytes2int(buf_[7], buf_[6]);
            work_temp_.temperature += co_temp_ * bytes2int(buf_[9], buf_[8]);
            break;

            case 0x53:
            {
                double r, p, y;
                r = co_pose_ * bytes2int(buf_[3], buf_[2]);
                p = co_pose_ * bytes2int(buf_[5], buf_[4]);
                y = co_pose_ * bytes2int(buf_[7], buf_[6]);
                tf2::Quaternion q;
                q.setRPY(r, p, y);
                q.normalize();
                work_imu_.orientation.x = q.x();
                work_imu_.orientation.y = q.y();
                work_imu_.orientation.z = q.z();
                work_imu_.orientation.w = q.w();
                imu_buf_.push(work_imu_);
                if (imu_buf_.size() >= msg_buf_max_)
                {
                    imu_buf_.pop();
                }
            }
            {
                work_temp_.temperature += co_temp_ * bytes2int(buf_[9], buf_[8]);
                work_temp_.temperature /= 3.0;
                temp_buf_.push(work_temp_);
                if (temp_buf_.size() >= msg_buf_max_)
                {
                    temp_buf_.pop();
                }
            }
            break;

            case 0x54:
            work_mag_.header.stamp = stamp;
            work_mag_.magnetic_field.x = co_mag_ * bytes2int(buf_[3], buf_[2]);
            work_mag_.magnetic_field.y = co_mag_ * bytes2int(buf_[5], buf_[4]);
            work_mag_.magnetic_field.z = co_mag_ * bytes2int(buf_[7], buf_[6]);
            mag_buf_.push(work_mag_);
            if (mag_buf_.size() >= msg_buf_max_)
            {
                mag_buf_.pop();
            }
            break;
        }
        buf_.erase(buf_.cbegin(), buf_.cbegin() + 11);
    }
    return;
}

std::vector<uint8_t> Wt901c::genYawClr() const
{
    return std::vector<uint8_t>{0xFF, 0xAA, 0x01, 0x04, 0x00};
}

std::vector<uint8_t> Wt901c::genHightClr() const
{
    return std::vector<uint8_t>{0xFF, 0xAA, 0x01, 0x03, 0x00};
}

std::vector<uint8_t> Wt901c::genAccCal() const
{
    return std::vector<uint8_t>{0xFF, 0xAA, 0x01, 0x01, 0x00};
}

std::vector<uint8_t> Wt901c::genMagCal() const
{
    return std::vector<uint8_t>{0xFF, 0xAA, 0x01, 0x02, 0x00};
}

std::vector<uint8_t> Wt901c::genExitCal() const
{
    return std::vector<uint8_t>{0xFF, 0xAA, 0x01, 0x00, 0x00};
}

}   // namespace wit_imu_driver
