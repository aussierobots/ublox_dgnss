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

#include <functional>
#include <chrono>
#include <thread>
#include <cstring>
#include <string>
#include <memory>
#include "ublox_dgnss_node/callback.hpp"
#include "ublox_dgnss_node/usb.hpp"

using namespace std::placeholders;

namespace usb
{
Connection::Connection(int vendor_id, int product_id, std::string serial_str, int log_level)
{
  vendor_id_ = vendor_id;
  product_id_ = product_id;
  serial_str_ = serial_str;
  class_id_ = LIBUSB_HOTPLUG_MATCH_ANY;
  log_level_ = log_level;
  ctx_ = NULL;
  devh_ = NULL;
  timeout_ms_ = 45;
  timeout_tv_ = {1, 0};       // default the timeout to 1 seconds
  keep_running_ = true;
}

void Connection::init()
{
  int rc = libusb_init(&ctx_);
  if (rc < 0) {
    throw std::string("Error initialising libusb: ") + libusb_error_name(rc);
  }

  if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
    throw std::string("Hotplug capabilities are not supported on this platform!");
  }

  // setup C style to C++ style callback
  hotplug_attach_callback_t<int(libusb_context * ctx, libusb_device * device,
    libusb_hotplug_event event, void * user_data)>::func = std::bind<int>(
    &Connection::hotplug_attach_callback, this, _1, _2, _3, _4);
  libusb_hotplug_callback_fn hotplug_attach_callback_fn =
    static_cast<libusb_hotplug_callback_fn>(hotplug_attach_callback_t<int(libusb_context * ctx,
    libusb_device * device, libusb_hotplug_event event, void * user_data)>::callback);

  rc = libusb_hotplug_register_callback(
    ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_ENUMERATE, vendor_id_,
    product_id_, class_id_, hotplug_attach_callback_fn, NULL, &hp_[0]);
  if (LIBUSB_SUCCESS != rc) {
    throw std::string("Error registering hotplug attach callback");
  }

  // setup C style to C++ style callback
  hotplug_detach_callback_t<int(libusb_context * ctx, libusb_device * device,
    libusb_hotplug_event event, void * user_data)>::func = std::bind<int>(
    &Connection::hotplug_detach_callback, this, _1, _2, _3, _4);
  libusb_hotplug_callback_fn hotplug_detach_callback_fn =
    static_cast<libusb_hotplug_callback_fn>(hotplug_detach_callback_t<int(libusb_context * ctx,
    libusb_device * device, libusb_hotplug_event event, void * user_data)>::callback);

  rc = libusb_hotplug_register_callback(
    ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_ENUMERATE, vendor_id_,
    product_id_, class_id_, hotplug_detach_callback_fn, NULL, &hp_[1]);
  if (LIBUSB_SUCCESS != rc) {
    throw std::string("Error registering hotplug detach callback");
  }

  /* Set debugging output level.
   */
    #if LIBUSB_API_VERSION >= 0x01000106
  libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, log_level_);
    #else
  libusb_set_debug(ctx_, log_level_);
    #endif
}

// Function to open a USB device with a specific Vendor ID, Product ID, and serial number string
libusb_device_handle * Connection::open_device_with_serial_string(
  libusb_context * ctx,
  int vendor_id, int product_id,
  std::string serial_str)
{
  libusb_device_handle * devHandle = nullptr;
  int rc = 0;

  // Get a list of USB devices
  libusb_device ** deviceList;

  ssize_t deviceCount;
  rc = libusb_get_device_list(ctx, &deviceList);
  if (rc < 0) {
    throw std::string("Error getting device list: ") + libusb_error_name(rc);
  } else {
    deviceCount = rc;
  }

  // Iterate through the list to find the desired device
  for (ssize_t i = 0; i < deviceCount; i++) {
    libusb_device * device = deviceList[i];
    libusb_device_descriptor desc;

    rc = libusb_get_device_descriptor(device, &desc);
    if (rc < 0) {
      throw std::string("Error getting device descriptor: ") + libusb_error_name(rc);
    }

    if (desc.idVendor != vendor_id || desc.idProduct != product_id) {
      continue;
    }

    // Open the device
    rc = libusb_open(device, &devHandle);
    if (rc < 0) {
      throw std::string("Error opening device: ") + libusb_error_name(rc);
    }

    char serial_num_string[256];
    // Read the serial number string
    rc = libusb_get_string_descriptor_ascii(
      devHandle, desc.iSerialNumber,
      reinterpret_cast<unsigned char *>(serial_num_string), sizeof(serial_num_string));
    if (rc < 0 && rc != LIBUSB_ERROR_INVALID_PARAM) {
      throw std::string("Error getting string descriptor ascii: ") + libusb_error_name(rc);
    }

    // if specified serial string is empty, we can just return now but assign
    if (serial_str.empty()) {
      break;
    }

    if (sizeof(serial_num_string) >= 0) {
      if (serial_str == serial_num_string) {
        // Device found and matched
        break;
      }
    }
    // Close the device if it didn't match
    libusb_close(devHandle);
    devHandle = nullptr;
  }
  // Free the device list
  libusb_free_device_list(deviceList, 1);

  return devHandle;
}


bool Connection::open_device()
{
  devh_ = open_device_with_serial_string(ctx_, vendor_id_, product_id_, serial_str_);
  if (!devh_) {
    if (serial_str_.empty()) {
      throw std::string("Error finding USB device");
      // std::cerr << "Error finding ublox USB device (no serial string supplied)";
    } else {
      throw std::string("Error finding USB device with specified serial string");
      // std::cerr << "Error finding ublox USB device with specified serial string";
    }
    return false;
  }

  // retrieve

  int rc = libusb_set_auto_detach_kernel_driver(devh_, true);
  if (rc < 0) {
    throw std::string("Error set auto detach kernel driver: ") + libusb_error_name(rc);
  }

  /* As we are dealing with a CDC-ACM device, it's highly probable that
   * Linux already attached the cdc-acm driver to this device.
   * We need to detach the drivers from all the USB interfaces. The CDC-ACM
   * Class defines two interfaces: the Control interface and the
   * Data interface.
   */
  for (int if_num = 0; if_num < 2; if_num++) {
    if (libusb_kernel_driver_active(devh_, if_num)) {
      libusb_detach_kernel_driver(devh_, if_num);
    }
    rc = libusb_claim_interface(devh_, if_num);
    if (rc < 0) {
      throw std::string("Error claiming interface: ") + libusb_error_name(rc);
    }
  }

  dev_ = libusb_get_device(devh_);

  /* get the device descriptor - newer libusb versions always succeed */
  struct libusb_device_descriptor dev_desc;
  rc = libusb_get_device_descriptor(dev_, &dev_desc);
  if (rc < 0) {
    throw std::string("Error getting device descriptor: ") + *libusb_error_name(rc);
  }
  auto num_configurations = dev_desc.bNumConfigurations;       // this should be 1
  if (num_configurations != 1) {
    throw std::string("Error bNumConfigurations is not 1 - dont know which configuration to use");
  }

  /* get the active USB configuration descriptor */
  struct libusb_config_descriptor * conf_desc;
  rc = libusb_get_active_config_descriptor(dev_, &conf_desc);
  if (rc < 0) {
    throw std::string("Error getting active configuration descriptor: ") + libusb_error_name(rc);
  }

  num_interfaces_ = conf_desc->bNumInterfaces;
  if (num_interfaces_ != 2) {
    throw std::string("Error config bNumInterfaces != 2");
  }

  for (uint8_t i = 0; i < num_interfaces_; i++) {
    auto interface = &conf_desc->interface[i];
    for (uint8_t j = 0; j < interface->num_altsetting; j++) {
      auto interface_desc = &interface->altsetting[j];

      switch (interface_desc->bInterfaceClass) {
        case LIBUSB_CLASS_COMM:
          // should only have one endpoint
          ep_comms_in_addr_ = interface_desc->endpoint[0].bEndpointAddress;
          break;
        case LIBUSB_CLASS_DATA:
          ep_data_out_addr_ = interface_desc->endpoint[0].bEndpointAddress;
          ep_data_in_addr_ = interface_desc->endpoint[1].bEndpointAddress;
          break;
        default:
          throw std::string("Error unknown bInterfaceClass");
      }
    }
  }
  libusb_free_config_descriptor(conf_desc);

  /* Start configuring the device:
   * - set line state
   */
  rc = libusb_control_transfer(
    devh_, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS,
    0, NULL, 0, 0);
  if (rc < 0 && rc != LIBUSB_ERROR_BUSY) {
    throw libusb_error_name(rc);
  }

  return true;
}

char * Connection::device_speed_txt()
{
  char * speed_txt;
  switch (device_speed()) {
    case LIBUSB_SPEED_LOW:
      speed_txt = const_cast<char *>("SPEED_LOW (1.5 MBit/s)");
      break;
    case LIBUSB_SPEED_HIGH:
      speed_txt = const_cast<char *>("SPEED_HIGH (480 MBit/s)");
      break;
    case LIBUSB_SPEED_FULL:
      speed_txt = const_cast<char *>("SPEED_FULL (12 MBit/s)");
      break;
    case LIBUSB_SPEED_SUPER:
      speed_txt = const_cast<char *>("SPEED_SUPER (5000 MBit/s)");
      break;
    case LIBUSB_SPEED_SUPER_PLUS:
      speed_txt = const_cast<char *>("SPEED_SUPER_PLUS (10000 MBit/s)");
      break;
    default:
      speed_txt = const_cast<char *>("SPEED_UNKNOWN");
      break;
  }
  return speed_txt;
}

int Connection::hotplug_attach_callback(
  libusb_context * ctx, libusb_device * dev,
  libusb_hotplug_event event, void * user_data)
{
  (void)ctx;
  (void)dev;
  (void)event;
  (void)user_data;
  // if device already attached, don't attempt to open further devices
  if (!attached_) {
    if (open_device()) {
      attached_ = true;
      (hp_attach_cb_fn_)();
      return 0;
    }
  }
  return 0;
}

int Connection::hotplug_detach_callback(
  libusb_context * ctx, libusb_device * dev,
  libusb_hotplug_event event, void * user_data)
{
  (void)ctx;
  (void)dev;
  (void)event;
  (void)user_data;
  if (attached_) {
    close_devh();
    attached_ = false;
    (hp_detach_cb_fn_)();
  }
  return 0;
}


int Connection::read_chars(u_char * data, size_t size)
{
  /* To receive characters from the device initiate a bulk_transfer to the
   * Endpoint with address ep_in_addr.
   */
  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_in_addr_ | LIBUSB_ENDPOINT_IN, data, size, &actual_length,
    timeout_ms_);
  if (rc == LIBUSB_ERROR_TIMEOUT) {
    throw TimeoutException();
  } else if (rc < 0) {
    std::string exception_msg("Error while waiting for char: ");
    exception_msg.append(libusb_error_name(rc));
    // std::cout << "exception_msg: " << exception_msg << std::endl;
    throw UsbException(exception_msg);
  }

  return actual_length;
}

void Connection::write_char(u_char c)
{
  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT, &c, 1,
    &actual_length, 0);
  if (rc < 0) {
    std::string exception_msg("Error while sending char: ");
    exception_msg.append(libusb_error_name(rc));
    throw UsbException(exception_msg);
  }
}

void Connection::write_buffer(u_char * buf, size_t size)
{
  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT, buf, size,
    &actual_length, 0);
  if (rc < 0) {
    std::string exception_msg("Error while sending buf: ");
    exception_msg.append(libusb_error_name(rc));
    throw UsbException(exception_msg);
  }

  if (actual_length != static_cast<int>(size)) {
    std::string exception_msg("Error didn't send full buf - size: ");
    exception_msg.append(std::to_string(size));
    exception_msg.append(" actual_length: ");
    exception_msg.append(std::to_string(actual_length));
    throw UsbException(exception_msg);
  }
}

void Connection::callback_out(struct libusb_transfer * transfer)
{
  if (transfer->status == libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED) {
    (out_cb_fn_)(transfer);
  } else {
    std::string msg;
    switch (transfer->status) {
      case LIBUSB_TRANSFER_ERROR:
        msg = "Transfer failed";
        return;
        break;
      case LIBUSB_TRANSFER_TIMED_OUT:
        msg = "Transfer timed out";
        break;
      case LIBUSB_TRANSFER_CANCELLED:
        msg = "Transfer cancelled";
        break;
      case LIBUSB_TRANSFER_STALL:
        msg = "Transfer stalled";
        break;
      case LIBUSB_TRANSFER_NO_DEVICE:
        msg = "Transfer device disconnected";
        break;
      case LIBUSB_TRANSFER_OVERFLOW:
        msg = "Transfer overflow. Device sent more data than requested";
        break;

      default:
        msg = "Unknown USB error - status: ";
        msg.append(std::to_string(transfer->status));
        break;
    }
    (exception_cb_fn_)(UsbException(msg), transfer->user_data);
  }
  libusb_free_transfer(transfer);
  // usb_event_completed_ = 1;
  bool * completed = reinterpret_cast<bool *>(transfer->user_data);
  *completed = true;

  // only queue another transfer in if none outstanding
  if (queued_transfer_in_num() == 0) {
    auto transfer_in = make_transfer_in();
    submit_transfer(transfer_in, "async submit transfer out - in: ");
  }
}

void Connection::callback_in(struct libusb_transfer * transfer)
{
  if (transfer->status == libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED) {
    (in_cb_fn_)(transfer);
    err_count_ = 0;
  } else {
    std::string msg;
    switch (transfer->status) {
      case LIBUSB_TRANSFER_ERROR:
        msg = "Transfer failed";
        break;
      case LIBUSB_TRANSFER_TIMED_OUT:
        msg = "Transfer timed out";
        break;
      case LIBUSB_TRANSFER_CANCELLED:
        msg = "Transfer cancelled";
        break;
      case LIBUSB_TRANSFER_STALL:
        msg = "Transfer stalled";
        break;
      case LIBUSB_TRANSFER_NO_DEVICE: {
          attached_ = false;
          msg = "Transfer device disconnected";
        }
        break;
      case LIBUSB_TRANSFER_OVERFLOW:
        msg = "Transfer overflow. Device sent more data than requested";
        break;

      default:
        msg = "Unknown USB error - status: ";
        msg.append(std::to_string(transfer->status));
        break;
    }
    if (++err_count_ < 10) {
      (exception_cb_fn_)(UsbException(msg), transfer->user_data);
    }
  }

  libusb_free_transfer(transfer);

  bool * completed = reinterpret_cast<bool *>(transfer->user_data);
  *completed = true;

  // only queue another transfer in if none outstanding
  if (attached_ && queued_transfer_in_num() == 0) {
    try {
      auto transfer_in = make_transfer_in();
      submit_transfer(transfer_in, "callback_in submit transfer: ");
    } catch (const UsbException & e) {
      (exception_cb_fn_)(e, nullptr);
    }
  }
}

void Connection::write_buffer_async(u_char * buf, size_t size, void * user_data)
{
  (void)user_data;
  if (out_cb_fn_ == nullptr) {
    throw UsbException("No out callback function set");
  }
  if (exception_cb_fn_ == nullptr) {
    throw UsbException("No exception callback function set");
  }

  auto transfer_out = make_transer_out(buf, size);
  submit_transfer(transfer_out, "async submit transfer out: ");
}

std::shared_ptr<transfer_t> Connection::make_transer_out(u_char * buf, size_t size)
{
  auto transfer_out = libusb_alloc_transfer(0);

  auto transfer = std::make_shared<transfer_t>();
  transfer->transfer = transfer_out;
  transfer->type = USB_OUT;
  transfer->buffer->resize(size);
  std::memcpy(transfer->buffer->data(), buf, size);
  transfer->completed = false;

  void * user_data = &transfer->completed;

  // setup C style to C++ style callback
  callback_out_t<void(struct libusb_transfer * transfer_out)>::func = std::bind(
    &Connection::callback_out, this, std::placeholders::_1);
  libusb_transfer_cb_fn callback_out_fn =
    static_cast<libusb_transfer_cb_fn>(callback_out_t<void(struct libusb_transfer * transfer_out)>::
    callback);

  transfer_out->flags = LIBUSB_TRANSFER_SHORT_NOT_OK;
  libusb_fill_bulk_transfer(
    transfer_out, devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT,
    // buf, size,
    transfer->buffer->data(), transfer->buffer->size(),
    callback_out_fn, user_data, 0
  );

  return transfer;
}

std::shared_ptr<transfer_t> Connection::make_transfer_in()
{
  auto transfer_in = libusb_alloc_transfer(0);

  auto transfer = std::make_shared<transfer_t>();
  transfer->transfer = transfer_in;
  transfer->type = USB_IN;
  transfer->buffer->resize(IN_BUFFER_SIZE);
  transfer->completed = false;

  void * user_data = &transfer->completed;

  // setup C style to C++ style callback
  callback_in_t<void(struct libusb_transfer * transfer_in)>::func = std::bind(
    &Connection::callback_in, this, std::placeholders::_1);
  libusb_transfer_cb_fn callback_in_fn =
    static_cast<libusb_transfer_cb_fn>(callback_in_t<void(struct libusb_transfer * transfer_in)>::
    callback);

  // setup asynchronous transfer in to host from usb
  libusb_fill_bulk_transfer(
    transfer_in, devh_, ep_data_in_addr_ | LIBUSB_ENDPOINT_IN,
    // in_buffer_, IN_BUFFER_SIZE,
    transfer->buffer->data(), transfer->buffer->size(),
    callback_in_fn, user_data, 0);             // no timeout

  return transfer;
}

void Connection::submit_transfer(
  std::shared_ptr<transfer_t> transfer, const std::string msg,
  bool wait_for_completed)
{
  (void)wait_for_completed;

  if (keep_running_ && attached_) {
    if (transfer->transfer == nullptr) {
      throw UsbException("transfer->transfer is null");
    }
    int rc = libusb_submit_transfer(transfer->transfer);
    if (rc < 0) {
      std::string exception_msg = msg;
      exception_msg.append(libusb_error_name(rc));
      throw UsbException(exception_msg);
    }

    // only adding those that were succesfully submitted to the queue
    transfer_queue_.push_back(transfer);

    // remove completed from the queue
    cleanup_transfer_queue();
  }
}

void Connection::cleanup_transfer_queue()
{
  if (transfer_queue_.size() == 0) {return;}

  // remove all completed transfer entries
  auto it = transfer_queue_.begin();
  while (it != transfer_queue_.end()) {
    if ((*it)->completed) {
      transfer_queue_.erase(it++);
    } else {
      ++it;
    }
  }
}

size_t Connection::queued_transfer_in_num()
{
  if (transfer_queue_.size() == 0) {return 0;}

  size_t num = 0;
  // remove all completed transfer entries
  for (auto it = transfer_queue_.begin(); it != transfer_queue_.end(); ++it) {
    auto t = it->get();
    if (!t->completed && t->type == USB_IN) {
      ++num;
    }
  }
  return num;
}

void Connection::init_async()
{
  if (in_cb_fn_ == nullptr) {
    throw UsbException("No in callback function set");
  }
  if (out_cb_fn_ == nullptr) {
    throw UsbException("No out callback function set");
  }
  if (exception_cb_fn_ == nullptr) {
    throw UsbException("No exception callback function set");
  }

  // submit initial transfer in request
  // - at first get a few zero length records
  auto transfer_in = make_transfer_in();
  submit_transfer(transfer_in, "init_async transfer: ", false);
}

void Connection::handle_usb_events()
{
  if (!keep_running_) {return;}

  // int rc = libusb_handle_events_timeout_completed(ctx_, &timeout_tv_, &usb_event_completed_);
  int rc = libusb_handle_events_timeout(ctx_, &timeout_tv_);
  switch (rc) {
    case LIBUSB_ERROR_INTERRUPTED:
      keep_running_ = false;
      break;
    case LIBUSB_ERROR_NO_DEVICE: {
        attached_ = false;
        // close_devh();
      }
      break;
    default:
      break;
  }
  if (rc < 0) {
    throw UsbException(libusb_error_name(rc));
  }
}

void Connection::close_devh()
{
  if (devh_) {
    for (int if_num = 0; if_num < 2; if_num++) {
      int rc = libusb_release_interface(devh_, if_num);
      if (rc >= 0) {
        libusb_attach_kernel_driver(devh_, if_num);
      }
    }
    libusb_close(devh_);             // hangs if the device has been detached already
    devh_ = nullptr;
    attached_ = false;
  }
}

void Connection::shutdown()
{
  keep_running_ = false;

  // de register hotplug callbacks
  if (!hp_[0]) {
    libusb_hotplug_deregister_callback(ctx_, hp_[0]);
  }
  if (!hp_[1]) {
    libusb_hotplug_deregister_callback(ctx_, hp_[1]);
  }

  close_devh();
}

Connection::~Connection()
{
  shutdown();

  libusb_exit(ctx_);
}
}  // namespace usb
