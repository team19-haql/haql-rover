// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bodenbot/bodenbot.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}

////////////////////////////////////////////////

namespace bodenbot {
class SerialInterface {
public:
  SerialInterface(std::string device, int address) {
    address_ = address;

    if ((fd_ = open(device.c_str(), O_RDWR)) < 0) {
      throw std::runtime_error("Failed to open the i2c bus");
    }
    RCLCPP_INFO(rclcpp::get_logger("SerialController"),
                "Opening I2C bus %s [%x]", device.c_str(), address);
  }

  void write_vel(int id, double velocity) {
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
      throw std::runtime_error(
          "Failed to acquire bus access and/or talk to slave.");
    }

    float vel = velocity;
    unsigned char send[1] = {(unsigned char)id};
    unsigned char *buffer = (unsigned char *)&vel;

    struct i2c_msg msgs[2] = {
        {
            .addr = (unsigned char)address_,
            .flags = 0,
            .len = 1,
            .buf = send,
        },
        {
            .addr = (unsigned char)address_,
            .flags = 0,
            .len = 4,
            .buf = buffer,
        },
    };

    struct i2c_rdwr_ioctl_data data {
      .msgs = msgs, .nmsgs = 2,
    };
    int res = ioctl(fd_, I2C_RDWR, &data);

    if (res < 0) {
      throw std::runtime_error("Failed to write to the i2c bus");
    }
  }

  double read_vel(int id) {
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
      throw std::runtime_error(
          "Failed to acquire bus access and/or talk to slave.");
    }

    unsigned char send[1] = {(unsigned char)id};
    unsigned char buffer[4];

    struct i2c_msg msgs[2] = {
        {
            .addr = (unsigned char)address_,
            .flags = 0,
            .len = 1,
            .buf = send,
        },
        {
            .addr = (unsigned char)address_,
            .flags = I2C_M_RD,
            .len = 4,
            .buf = buffer,
        },
    };

    struct i2c_rdwr_ioctl_data data {
      .msgs = msgs, .nmsgs = 2,
    };
    int res = ioctl(fd_, I2C_RDWR, &data);
    float vel = *((float *)buffer);

    if (res < 0) {
      throw std::runtime_error("Failed to read from the i2c bus");
    }
    return vel;
  }

  void close_connection() {
    close(fd_);
    fd_ = -1;
  }

private:
  int address_;
  int fd_;
};

hardware_interface::CallbackReturn
SerialController::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // configuration parameters for hardware
#define DEBUG_PARAM(message, name)                                             \
  RCLCPP_INFO(rclcpp::get_logger("SerialController"), message,                 \
              info_.hardware_parameters[name].c_str());

  DEBUG_PARAM("debug: %s", "debug");
  DEBUG_PARAM("mock hardware: %s", "mock_hardware");
  DEBUG_PARAM("i2c_file: %s", "i2c_file");
  DEBUG_PARAM("i2c_address: %s", "i2c_address");
  // parameter for velocity_max_accel

#undef DEBUG_PARAM

  debug_ = info_.hardware_parameters["debug"] == "True";
  mock_hardware_ = info_.hardware_parameters["mock_hardware"] == "True";
  i2c_file_ = info_.hardware_parameters["i2c_file"];
  i2c_address_ = std::atoi(info_.hardware_parameters["i2c_address"].c_str());

  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());

  for (hardware_interface::ComponentInfo &joint : info_.joints) {
    // we only have velocity command and feedback
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("SerialController"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("SerialController"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("SerialController"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("SerialController"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("SerialController"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("SerialController"),
                "Found joint '%s' with id '%s' and reversed '%s'",
                joint.name.c_str(), joint.parameters["id"].c_str(),
                joint.parameters["reversed"].c_str());

    motor_id_.emplace_back(std::atoi(joint.parameters["id"].c_str()));
    reversed_.emplace_back(joint.parameters["reversed"] == "true");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SerialController::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialController"),
              "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SerialController::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialController"),
              "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn SerialController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SerialController"),
              "Activating ...please wait...");

  for (auto i = 0u; i < hw_velocities_.size(); i++) {
    hw_velocities_[i] = 0;
  }

  if (!mock_hardware_) {
    serial_interface_ = new SerialInterface(i2c_file_, i2c_address_);
  } else {
    serial_interface_ = NULL;
    RCLCPP_INFO(rclcpp::get_logger("SerialController"),
                "Not writing to device");
  }

  for (auto i = 0u; i < info_.joints.size(); i++) {
    hw_commands_[i] = 0;
    hw_velocities_[i] = 0;
    hw_positions_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialController"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_INFO(rclcpp::get_logger("SerialController"),
              "Deactivating ...please wait...");

  if (serial_interface_) {
    serial_interface_->close_connection();
    delete serial_interface_;
    serial_interface_ = NULL;
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialController"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SerialController::read(const rclcpp::Time & /*time*/,
                       const rclcpp::Duration &period) {
  // update state interface for steer joint
  for (auto i = 0u; i < info_.joints.size(); i++) {
    if (serial_interface_) {
      // nothing
      int id = motor_id_[i];
      try {
        double velocity = serial_interface_->read_vel(id);
        if (reversed_[i]) {
          velocity = -velocity;
        }
        hw_velocities_[i] = velocity;
      } catch (std::exception const &e) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialController"), e.what());
      }
    }

    hw_positions_[i] += hw_velocities_[i] * period.seconds();
    hw_positions_[i] = std::fmod(hw_positions_[i], 2 * M_PI);

    if (debug_) {
      RCLCPP_INFO(rclcpp::get_logger("SerialController"),
                  "Read [%ld] (%f) (%f)", motor_id_[i], hw_velocities_[i],
                  hw_positions_[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
bodenbot::SerialController::write(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {
  for (auto i = 0u; i < info_.joints.size(); i++) {
    int id = motor_id_[i];
    double velocity = hw_commands_[i];
    if (reversed_[i]) {
      velocity = -velocity;
    }

    if (debug_) {
      RCLCPP_INFO(rclcpp::get_logger("SerialController"), "Write [%d] (%f)", id,
                  velocity);
    }

    if (!serial_interface_) {
      hw_velocities_[i] = velocity;
      continue;
    } else {
      try {
        serial_interface_->write_vel(id, velocity);
      } catch (std::exception const &e) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialController"), e.what());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

} // namespace bodenbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bodenbot::SerialController,
                       hardware_interface::SystemInterface)
