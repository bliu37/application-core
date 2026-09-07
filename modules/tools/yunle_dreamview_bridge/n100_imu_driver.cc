/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/common_msgs/localization_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"

namespace apollo {
namespace yunle {

namespace {

constexpr uint8_t kFrameHead = 0xfc;
constexpr uint8_t kFrameEnd = 0xfd;
constexpr uint8_t kTypeImu = 0x40;
constexpr uint8_t kTypeAhrs = 0x41;
constexpr uint8_t kTypeInsGps = 0x42;
constexpr uint8_t kTypeGeodeticPos = 0x5c;
constexpr uint8_t kTypeGround = 0xf0;
constexpr uint8_t kImuLen = 0x38;
constexpr uint8_t kAhrsLen = 0x30;
constexpr uint8_t kInsGpsLen = 0x48;
constexpr uint8_t kGeodeticPosLen = 0x20;
constexpr double kPi = 3.14159265358979323846;
constexpr char kRawImuTopic[] = "/apollo/sensor/gnss/imu";
constexpr char kCorrectedImuTopic[] = "/apollo/sensor/gnss/corrected_imu";
constexpr char kFrameId[] = "imu";
constexpr char kModuleName[] = "n100_imu";

#pragma pack(push, 1)
struct FrameHeader {
  uint8_t header_start = 0;
  uint8_t data_type = 0;
  uint8_t data_size = 0;
  uint8_t serial_num = 0;
  uint8_t header_crc8 = 0;
  uint8_t header_crc16_h = 0;
  uint8_t header_crc16_l = 0;
};

struct ImuPacket {
  float gyroscope_x = 0.0f;
  float gyroscope_y = 0.0f;
  float gyroscope_z = 0.0f;
  float accelerometer_x = 0.0f;
  float accelerometer_y = 0.0f;
  float accelerometer_z = 0.0f;
  float magnetometer_x = 0.0f;
  float magnetometer_y = 0.0f;
  float magnetometer_z = 0.0f;
  float imu_temperature = 0.0f;
  float pressure = 0.0f;
  float pressure_temperature = 0.0f;
  int64_t timestamp = 0;
};

struct AhrsPacket {
  float roll_speed = 0.0f;
  float pitch_speed = 0.0f;
  float heading_speed = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float heading = 0.0f;
  float qw = 1.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  int64_t timestamp = 0;
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 7, "unexpected N100 frame header size");
static_assert(sizeof(ImuPacket) == kImuLen, "unexpected N100 IMU packet size");
static_assert(sizeof(AhrsPacket) == kAhrsLen, "unexpected N100 AHRS packet size");

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Rpy {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

uint8_t Crc8(const uint8_t* data, size_t size) {
  uint8_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x01) ? static_cast<uint8_t>((crc >> 1) ^ 0x8c)
                         : static_cast<uint8_t>(crc >> 1);
    }
  }
  return crc;
}

uint16_t Crc16(const uint8_t* data, size_t size) {
  uint16_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

bool IsKnownType(uint8_t type) {
  return type == kTypeImu || type == kTypeAhrs || type == kTypeInsGps ||
         type == kTypeGeodeticPos || type == kTypeGround || type == 0x50;
}

bool ExpectedPayloadLength(uint8_t type, uint8_t length) {
  if (type == kTypeImu) {
    return length == kImuLen;
  }
  if (type == kTypeAhrs) {
    return length == kAhrsLen;
  }
  if (type == kTypeInsGps) {
    return length == kInsGpsLen;
  }
  if (type == kTypeGeodeticPos) {
    return length == kGeodeticPosLen;
  }
  return true;
}

Quaternion Multiply(const Quaternion& a, const Quaternion& b) {
  Quaternion out;
  out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return out;
}

Quaternion Normalize(Quaternion q) {
  const double norm =
      std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return Quaternion();
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quaternion AngleAxis(double angle, char axis) {
  Quaternion q;
  q.w = std::cos(angle * 0.5);
  const double s = std::sin(angle * 0.5);
  if (axis == 'x') {
    q.x = s;
  } else if (axis == 'y') {
    q.y = s;
  } else if (axis == 'z') {
    q.z = s;
  }
  return q;
}

Quaternion ConvertAhrsQuaternionToRosStandard(const AhrsPacket& packet) {
  // Match fdilink_ahrs device_type=1:
  // q_out = RotZ(pi) * RotY(pi) * q_ahrs * RotX(pi).
  const Quaternion q_r = Multiply(AngleAxis(kPi, 'z'), AngleAxis(kPi, 'y'));
  const Quaternion q_rr = AngleAxis(kPi, 'x');
  const Quaternion q_ahrs =
      Normalize({packet.qw, packet.qx, packet.qy, packet.qz});
  return Normalize(Multiply(Multiply(q_r, q_ahrs), q_rr));
}

Rpy QuaternionToRpy(const Quaternion& q_in) {
  const Quaternion q = Normalize(q_in);
  Rpy rpy;
  const double sin_roll = 2.0 * (q.w * q.x + q.y * q.z);
  const double cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  rpy.roll = std::atan2(sin_roll, cos_roll);

  const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
  if (std::abs(sin_pitch) >= 1.0) {
    rpy.pitch = std::copysign(kPi / 2.0, sin_pitch);
  } else {
    rpy.pitch = std::asin(sin_pitch);
  }

  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  rpy.yaw = std::atan2(sin_yaw, cos_yaw);
  return rpy;
}

void FillHeader(double timestamp_sec, uint32_t sequence_num,
                apollo::common::Header* header) {
  header->set_timestamp_sec(timestamp_sec);
  header->set_module_name(kModuleName);
  header->set_sequence_num(sequence_num);
  header->set_frame_id(kFrameId);
}

speed_t BaudRate(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B921600;
  }
}

std::string GetEnvString(const char* name) {
  const char* value = std::getenv(name);
  return value == nullptr ? std::string() : std::string(value);
}

int GetEnvInt(const char* name, int default_value) {
  const std::string value = GetEnvString(name);
  if (value.empty()) {
    return default_value;
  }
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    AWARN << "Ignoring invalid integer environment variable " << name << "="
          << value;
    return default_value;
  }
}

}  // namespace

class N100ImuDriver final : public cyber::Component<> {
 public:
  ~N100ImuDriver() override {
    running_.store(false);
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    CloseSerial();
  }

  bool Init() override {
    raw_imu_writer_ =
        node_->CreateWriter<drivers::gnss::Imu>(kRawImuTopic);
    corrected_imu_writer_ =
        node_->CreateWriter<localization::CorrectedImu>(kCorrectedImuTopic);
    if (raw_imu_writer_ == nullptr || corrected_imu_writer_ == nullptr) {
      AERROR << "Failed to create N100 IMU writers.";
      return false;
    }

    baud_rate_ = GetEnvInt("YUNLE_N100_IMU_BAUD", 921600);
    if (!OpenFirstAvailableSerial()) {
      return false;
    }

    running_.store(true);
    reader_thread_ = std::thread([this]() { ReadLoop(); });
    AINFO << "N100 IMU driver started. port=" << serial_port_
          << " baud=" << baud_rate_ << " raw_topic=" << kRawImuTopic
          << " corrected_topic=" << kCorrectedImuTopic;
    return true;
  }

 private:
  bool OpenFirstAvailableSerial() {
    std::vector<std::string> ports;
    const std::string env_port = GetEnvString("YUNLE_N100_IMU_PORT");
    if (!env_port.empty()) {
      ports.push_back(env_port);
    }
    ports.push_back("/dev/wheeltec_FDI_IMU_GNSS");
    ports.push_back(
        "/dev/serial/by-id/"
        "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-"
        "port0");
    ports.push_back("/dev/ttyUSB1");

    for (const auto& port : ports) {
      if (OpenSerial(port)) {
        serial_port_ = port;
        return true;
      }
    }

    AERROR << "Failed to open N100 IMU serial port. Tried "
           << "YUNLE_N100_IMU_PORT, /dev/wheeltec_FDI_IMU_GNSS, "
           << "the CP2102 by-id path, and /dev/ttyUSB1.";
    return false;
  }

  bool OpenSerial(const std::string& port) {
    const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      return false;
    }

    termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      close(fd);
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    const speed_t speed = BaudRate(baud_rate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      close(fd);
      return false;
    }

    serial_fd_ = fd;
    return true;
  }

  void CloseSerial() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  bool ReadExact(uint8_t* data, size_t size, int timeout_ms) {
    size_t offset = 0;
    while (running_.load() && offset < size) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(serial_fd_, &read_fds);
      timeval timeout;
      timeout.tv_sec = timeout_ms / 1000;
      timeout.tv_usec = (timeout_ms % 1000) * 1000;
      const int ready = select(serial_fd_ + 1, &read_fds, nullptr, nullptr,
                               &timeout);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        AWARN_EVERY(100) << "N100 serial select failed.";
        return false;
      }
      if (ready == 0) {
        return false;
      }

      const ssize_t read_size = read(serial_fd_, data + offset, size - offset);
      if (read_size > 0) {
        offset += static_cast<size_t>(read_size);
        continue;
      }
      if (read_size < 0 && (errno == EINTR || errno == EAGAIN)) {
        continue;
      }
      return false;
    }
    return offset == size;
  }

  bool ReadFrame(FrameHeader* header, std::vector<uint8_t>* payload) {
    uint8_t byte = 0;
    while (running_.load()) {
      if (!ReadExact(&byte, 1, 100)) {
        return false;
      }
      if (byte == kFrameHead) {
        break;
      }
    }
    if (!running_.load()) {
      return false;
    }

    uint8_t header_bytes[sizeof(FrameHeader)] = {};
    header_bytes[0] = byte;
    if (!ReadExact(header_bytes + 1, sizeof(FrameHeader) - 1, 20)) {
      return false;
    }
    std::memcpy(header, header_bytes, sizeof(FrameHeader));

    if (!IsKnownType(header->data_type)) {
      AWARN_EVERY(100) << "Unknown N100 frame type: "
                       << static_cast<int>(header->data_type);
      return false;
    }
    if (!ExpectedPayloadLength(header->data_type, header->data_size)) {
      AWARN_EVERY(100) << "Unexpected N100 frame length. type="
                       << static_cast<int>(header->data_type)
                       << " len=" << static_cast<int>(header->data_size);
      return false;
    }
    if (Crc8(header_bytes, 4) != header->header_crc8) {
      AWARN_EVERY(100) << "N100 header CRC8 mismatch.";
      return false;
    }

    payload->assign(header->data_size + 1, 0);
    if (!ReadExact(payload->data(), payload->size(), 20)) {
      return false;
    }

    const uint16_t expected_crc16 =
        static_cast<uint16_t>(header->header_crc16_l) |
        (static_cast<uint16_t>(header->header_crc16_h) << 8);
    const uint16_t actual_crc16 = Crc16(payload->data(), header->data_size);
    if (expected_crc16 != actual_crc16) {
      AWARN_EVERY(100) << "N100 payload CRC16 mismatch.";
      return false;
    }
    if ((*payload)[header->data_size] != kFrameEnd) {
      AWARN_EVERY(100) << "N100 frame end mismatch.";
      return false;
    }
    payload->resize(header->data_size);
    return true;
  }

  void ReadLoop() {
    FrameHeader header;
    std::vector<uint8_t> payload;
    while (running_.load() && cyber::OK()) {
      if (!ReadFrame(&header, &payload)) {
        continue;
      }

      if (header.data_type == kTypeAhrs &&
          payload.size() == sizeof(AhrsPacket)) {
        AhrsPacket packet;
        std::memcpy(&packet, payload.data(), sizeof(packet));
        std::lock_guard<std::mutex> lock(ahrs_mutex_);
        latest_ahrs_ = packet;
        has_ahrs_ = true;
        continue;
      }

      if (header.data_type == kTypeImu &&
          payload.size() == sizeof(ImuPacket)) {
        ImuPacket packet;
        std::memcpy(&packet, payload.data(), sizeof(packet));
        PublishImu(packet);
      }
    }
  }

  void PublishImu(const ImuPacket& packet) {
    const double timestamp_sec = cyber::Clock::NowInSeconds();
    ++sequence_num_;

    const double gyro_x = packet.gyroscope_x;
    const double gyro_y = -packet.gyroscope_y;
    const double gyro_z = -packet.gyroscope_z;
    const double acc_x = packet.accelerometer_x;
    const double acc_y = -packet.accelerometer_y;
    const double acc_z = -packet.accelerometer_z;

    auto raw = std::make_shared<drivers::gnss::Imu>();
    FillHeader(timestamp_sec, sequence_num_, raw->mutable_header());
    raw->set_measurement_time(timestamp_sec);
    raw->set_measurement_span(0.0);
    raw->mutable_angular_velocity()->set_x(gyro_x);
    raw->mutable_angular_velocity()->set_y(gyro_y);
    raw->mutable_angular_velocity()->set_z(gyro_z);
    raw->mutable_linear_acceleration()->set_x(acc_x);
    raw->mutable_linear_acceleration()->set_y(acc_y);
    raw->mutable_linear_acceleration()->set_z(acc_z);
    raw_imu_writer_->Write(raw);

    AhrsPacket ahrs;
    bool has_ahrs = false;
    {
      std::lock_guard<std::mutex> lock(ahrs_mutex_);
      ahrs = latest_ahrs_;
      has_ahrs = has_ahrs_;
    }
    if (!has_ahrs) {
      return;
    }

    const Quaternion orientation = ConvertAhrsQuaternionToRosStandard(ahrs);
    const Rpy rpy = QuaternionToRpy(orientation);
    auto corrected = std::make_shared<localization::CorrectedImu>();
    FillHeader(timestamp_sec, sequence_num_, corrected->mutable_header());
    auto* imu = corrected->mutable_imu();
    imu->mutable_orientation()->set_qw(orientation.w);
    imu->mutable_orientation()->set_qx(orientation.x);
    imu->mutable_orientation()->set_qy(orientation.y);
    imu->mutable_orientation()->set_qz(orientation.z);
    imu->mutable_euler_angles()->set_x(rpy.roll);
    imu->mutable_euler_angles()->set_y(rpy.pitch);
    imu->mutable_euler_angles()->set_z(rpy.yaw);
    imu->mutable_angular_velocity()->set_x(gyro_x);
    imu->mutable_angular_velocity()->set_y(gyro_y);
    imu->mutable_angular_velocity()->set_z(gyro_z);
    imu->mutable_linear_acceleration()->set_x(acc_x);
    imu->mutable_linear_acceleration()->set_y(acc_y);
    imu->mutable_linear_acceleration()->set_z(acc_z);
    corrected_imu_writer_->Write(corrected);
  }

  int serial_fd_ = -1;
  int baud_rate_ = 921600;
  std::string serial_port_;
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  uint32_t sequence_num_ = 0;

  std::mutex ahrs_mutex_;
  AhrsPacket latest_ahrs_;
  bool has_ahrs_ = false;

  std::shared_ptr<cyber::Writer<drivers::gnss::Imu>> raw_imu_writer_;
  std::shared_ptr<cyber::Writer<localization::CorrectedImu>>
      corrected_imu_writer_;
};

CYBER_REGISTER_COMPONENT(N100ImuDriver)

}  // namespace yunle
}  // namespace apollo
