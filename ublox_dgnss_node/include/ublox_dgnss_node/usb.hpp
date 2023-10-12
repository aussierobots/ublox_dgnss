// Copyright 2021 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__USB_HPP_
#define UBLOX_DGNSS_NODE__USB_HPP_

#include <libusb-1.0/libusb.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <exception>
#include <string>
#include <functional>
#include <deque>
#include <vector>
#include <memory>

#define F9_VENDOR_ID      0x1546   // U-Blox AG
#define F9_PRODUCT_ID     0x01a9   // u-blox GNSS receiver

#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02

#define IN_BUFFER_SIZE 64 * 40

namespace usb
{
using UCharVector = std::vector<u_char>;
enum TransferType {USB_IN, USB_OUT};
struct transfer_t
{
  struct libusb_transfer * transfer;
  std::shared_ptr<UCharVector> buffer;
  bool completed;
  TransferType type;
  transfer_t()
  {
    buffer = std::make_shared<UCharVector>();
  }
};

class TimeoutException : public std::exception
{
public:
  const char * what() const throw()
  {
    return "Timeout";
  }
};

class UsbException : public std::runtime_error
{
public:
  explicit UsbException(std::string msg)
  : std::runtime_error(msg)
  {
  }
};

// external function callback definitions for the connection class
typedef std::function<void (struct libusb_transfer * transfer)> connection_out_cb_fn;
typedef std::function<void (struct libusb_transfer * transfer)> connection_in_cb_fn;
typedef std::function<void (UsbException e, void * user_data)> connection_exception_cb_fn;
typedef std::function<void ()> hotplug_attach_cb_fn;
typedef std::function<void ()> hotplug_detach_cb_fn;

class Connection
{
private:
  libusb_context * ctx_;
  libusb_device_handle * devh_;
  libusb_device * dev_;

// hotplug
  hotplug_attach_cb_fn hp_attach_cb_fn_;
  hotplug_detach_cb_fn hp_detach_cb_fn_;
  libusb_hotplug_callback_handle hp_[2];

  int log_level_;
  int vendor_id_;
  int product_id_;
  std::string serial_str_;
  int class_id_;
  int ep_data_out_addr_;
  int ep_data_in_addr_;
  int ep_comms_in_addr_;
  u_int8_t num_interfaces_ = 0;
  unsigned int timeout_ms_;

// asynchronous comms
  connection_out_cb_fn out_cb_fn_;
  connection_in_cb_fn in_cb_fn_;
  connection_exception_cb_fn exception_cb_fn_;
  struct timeval timeout_tv_;
  bool keep_running_;
  bool attached_;

  size_t err_count_ = 0;

  std::deque<std::shared_ptr<transfer_t>> transfer_queue_;

private:
  libusb_device_handle * open_device_with_serial_string(
    libusb_context * ctx, int vendor_id,
    int product_id, std::string serial_str);
// this is called after the out transfer to USB from HOST has been received by libusb
  void callback_out(struct libusb_transfer * transfer);
// this is called when the stat for in is available - from USB in HOST
  void callback_in(struct libusb_transfer * transfer);

  int hotplug_attach_callback(
    libusb_context * ctx, libusb_device * dev,
    libusb_hotplug_event event, void * user_data);
  int hotplug_detach_callback(
    libusb_context * ctx, libusb_device * dev,
    libusb_hotplug_event event, void * user_data);


  std::shared_ptr<transfer_t> make_transfer_in();
  std::shared_ptr<transfer_t> make_transer_out(u_char * buf, size_t size);
  void submit_transfer(
    std::shared_ptr<transfer_t> transfer,
    const std::string msg = "Error submit transfer: ",
    bool wait_for_completed = true);
  void cleanup_transfer_queue();
  void close_devh();

public:
  void init();  // throws exception on failure
  bool open_device();  // returns false on failure
  void init_async();  // throws exception on failure
  Connection(
    int vendor_id, int product_id, std::string serial_str,
    int log_level = LIBUSB_OPTION_LOG_LEVEL);
  ~Connection();
  void set_in_callback(connection_in_cb_fn in_cb_fn)
  {
    in_cb_fn_ = in_cb_fn;
  }
  void set_out_callback(connection_out_cb_fn out_cb_fn)
  {
    out_cb_fn_ = out_cb_fn;
  }
  void set_exception_callback(connection_exception_cb_fn exception_fb_fn)
  {
    exception_cb_fn_ = exception_fb_fn;
  }
  void set_hotplug_attach_callback(hotplug_attach_cb_fn hp_attach_cb_fn)
  {
    hp_attach_cb_fn_ = hp_attach_cb_fn;
  }
  void set_hotplug_detach_callback(hotplug_detach_cb_fn hp_detach_cb_fn)
  {
    hp_detach_cb_fn_ = hp_detach_cb_fn;
  }
  int vendor_id()
  {
    return vendor_id_;
  }
  int product_id()
  {
    return product_id_;
  }
  std::string serial_str()
  {
    return serial_str_;
  }
  bool inline devh_valid()
  {
    return dev_ != nullptr;
  }
  int bus_number()
  {
    return devh_valid() ? libusb_get_bus_number(dev_) : 0;
  }
  int device_address()
  {
    return devh_valid() ? libusb_get_device_address(dev_) : 0;
  }
  int device_speed()
  {
    return devh_valid() ? libusb_get_device_speed(dev_) : 0;
  }
  char * device_speed_txt();
  u_int8_t port_number()
  {
    return devh_valid() ? libusb_get_port_number(dev_) : 0;
  }
  int read_chars(u_char * data, size_t size);
  void write_char(u_char c);
  void write_buffer(u_char * buf, size_t size);
  void write_buffer_async(u_char * buf, size_t size, void * user_data);
  u_int8_t num_interfaces()
  {
    return num_interfaces_;
  }
  int ep_data_out_addr()
  {
    return ep_data_out_addr_;
  }
  int ep_data_in_addr()
  {
    return ep_data_in_addr_;
  }
  int ep_comms_in_addr()
  {
    return ep_comms_in_addr_;
  }
  unsigned int timeout_ms()
  {
    return timeout_ms_;
  }
  bool keep_running()
  {
    return keep_running_;
  }
  bool attached()
  {
    return attached_;
  }

// number of existing transfer in in the queue that are not complete
  size_t queued_transfer_in_num();

// needs to be called for libusb to action asynchronos events
  void handle_usb_events();
  void shutdown();
};
}   // end namespace usb

#endif  // UBLOX_DGNSS_NODE__USB_HPP_
