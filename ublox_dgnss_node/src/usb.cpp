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
#include <sstream>
#include <iomanip>
#include "ublox_dgnss_node/callback.hpp"
#include "ublox_dgnss_node/usb.hpp"

using namespace std::placeholders;

namespace usb
{
Connection::Connection(
  int vendor_id, const std::vector<uint16_t> & product_ids, std::string serial_str,
  ublox_dgnss::DeviceFamily device_family, int log_level)
{
  vendor_id_ = vendor_id;
  product_ids_ = product_ids;
  connected_product_id_ = 0;  // Initialize to 0, will be set when device connects
  serial_str_ = serial_str;
  device_family_ = device_family;
  class_id_ = LIBUSB_HOTPLUG_MATCH_ANY;

  log_level_ = log_level;
  ctx_ = NULL;
  devh_ = NULL;
  timeout_ms_ = 45;
  timeout_tv_ = {1, 0};       // default the timeout to 1 seconds
  keep_running_ = true;

  driver_state_ = USBDriverState::DISCONNECTED;

  // setup C style to C++ style callback
  callback_out_t<void(struct libusb_transfer * transfer_out)>::func = std::bind(
    &Connection::callback_out, this, std::placeholders::_1);
  callback_in_t<void(struct libusb_transfer * transfer_in)>::func = std::bind(
    &Connection::callback_in, this, std::placeholders::_1);

  callback_out_fn_ =
    static_cast<libusb_transfer_cb_fn>(callback_out_t<void(struct libusb_transfer * transfer_out)>::
    callback);
  callback_in_fn_ =
    static_cast<libusb_transfer_cb_fn>(callback_in_t<void(struct libusb_transfer * transfer_in)>::
    callback);
}

void Connection::init()
{
  // Check if already initialized
  if (ctx_ != nullptr) {
    if (debug_cb_fn_) {
      (debug_cb_fn_)("init() already called - skipping");
    }
    return;
  }

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

  // Register hotplug callbacks for ALL product IDs in device family
  hp_attach_.resize(product_ids_.size());
  for (size_t i = 0; i < product_ids_.size(); i++) {
    rc = libusb_hotplug_register_callback(
      ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_ENUMERATE, vendor_id_,
      product_ids_[i], class_id_, hotplug_attach_callback_fn, NULL, &hp_attach_[i]);
    if (LIBUSB_SUCCESS != rc) {
      throw std::string("Error registering hotplug attach callback for product ID: ") +
            std::to_string(product_ids_[i]);
    }
  }

  // setup C style to C++ style callback
  hotplug_detach_callback_t<int(libusb_context * ctx, libusb_device * device,
    libusb_hotplug_event event, void * user_data)>::func = std::bind<int>(
    &Connection::hotplug_detach_callback, this, _1, _2, _3, _4);
  libusb_hotplug_callback_fn hotplug_detach_callback_fn =
    static_cast<libusb_hotplug_callback_fn>(hotplug_detach_callback_t<int(libusb_context * ctx,
    libusb_device * device, libusb_hotplug_event event, void * user_data)>::callback);

  hp_detach_.resize(product_ids_.size());
  for (size_t i = 0; i < product_ids_.size(); i++) {
    rc = libusb_hotplug_register_callback(
      ctx_, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_ENUMERATE, vendor_id_,
      product_ids_[i], class_id_, hotplug_detach_callback_fn, NULL, &hp_detach_[i]);
    if (LIBUSB_SUCCESS != rc) {
      throw std::string("Error registering hotplug detach callback for product ID: ") +
            std::to_string(product_ids_[i]);
    }
  }

  /* Set debugging output level.
   */
    #if LIBUSB_API_VERSION >= 0x01000106
  libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, log_level_);
    #else
  libusb_set_debug(ctx_, log_level_);
    #endif
}

libusb_device_handle * Connection::open_device_with_serial_string(
  libusb_context * ctx,
  int vendor_id, const std::vector<uint16_t> & product_ids,
  std::string serial_str,
  char * serial_num_string)
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

    // Check if device matches vendor and any of the product IDs
    if (desc.idVendor != vendor_id) {
      continue;
    }

    bool product_id_match = false;
    for (uint16_t product_id : product_ids) {
      if (desc.idProduct == product_id) {
        product_id_match = true;
        break;
      }
    }

    if (!product_id_match) {
      continue;
    }

    // Open the device
    rc = libusb_open(device, &devHandle);
    if (rc < 0) {
      throw std::string("Error opening device: ") + libusb_error_name(rc);
    }

    // Enhanced serial number reading with device family awareness
    rc = libusb_get_string_descriptor_ascii(
      devHandle, desc.iSerialNumber,
      reinterpret_cast<unsigned char *>(serial_num_string), SERIAL_STRING_BUFFER_SIZE);

    // Product ID-specific serial string handling
    auto device_info = ublox_dgnss::get_device_family_info(device_family_);
    bool reliable_iserial = false;

    if (device_family_ == ublox_dgnss::DeviceFamily::X20P) {
      // X20P: Serial behavior depends on which USB device we're using
      if (desc.idProduct == 0x01ab) {
        // 0x01ab: F9P/F9R-compatible behavior (IDENTICAL serial handling)
        reliable_iserial = false;  // User-programmed via u-center, may be empty
      } else {
        // 0x050c/0x050d: Vendor-specific behavior (DIFFERENT serial handling)
        reliable_iserial = true;  // Factory programmed, always reliable
      }
    } else {
      // F9P/F9R: Standard behavior
      reliable_iserial = device_info.reliable_iserial;
    }

    if (rc < 0) {
      if (reliable_iserial) {
        // Factory iSerial should be reliable (X20P 0x050c/0x050d)
        throw std::string("Serial string read failed (unexpected): ") + libusb_error_name(rc);
      } else {
        // User-programmed iSerial may not be reliable (F9P/F9R + X20P 0x01ab)
        if (rc != LIBUSB_ERROR_INVALID_PARAM) {
          throw std::string("Serial string read failed: ") + libusb_error_name(rc);
        }
        // LIBUSB_ERROR_INVALID_PARAM is expected if serial not programmed
      }
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
  driver_state_ = USBDriverState::CONNECTING;

  char serial_num_string[SERIAL_STRING_BUFFER_SIZE];
  devh_ = open_device_with_serial_string(
    ctx_, vendor_id_, product_ids_, serial_str_,
    serial_num_string);
  if (!devh_) {
    if (serial_str_.empty()) {
      throw std::string("Error finding USB device");
      // std::cerr << "Error finding ublox USB device (no serial string supplied)";
    } else {
      throw std::string("Error finding USB device with specified serial string, looking for \"") +
            serial_str_ + "\" however \"" + serial_num_string + "\" was found.";
      // std::cerr << "Error finding ublox USB device with specified serial string";
    }
    driver_state_ = USBDriverState::ERROR;
    return false;
  }

  int rc = libusb_set_auto_detach_kernel_driver(devh_, true);
  if (rc < 0) {
    throw std::string("Error set auto detach kernel driver: ") + libusb_error_name(rc);
  }

  /*
   * Different device families have different interface patterns:
   * - X20P/F9P/F9R: CDC-ACM with 2 interfaces (Control + Data)
   * - X20P: Vendor Specific with 1 interface
   * Interface claiming is now done after we determine the actual interface count.
   */

  dev_ = libusb_get_device(devh_);

  /* get the device descriptor - newer libusb versions always succeed */
  struct libusb_device_descriptor dev_desc;
  rc = libusb_get_device_descriptor(dev_, &dev_desc);
  if (rc < 0) {
    throw std::string("Error getting device descriptor: ") + *libusb_error_name(rc);
  }

  connected_product_id_ = dev_desc.idProduct;

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

  /*
   * CRITICAL: X20P presents as THREE SEPARATE USB DEVICES from one physical module:
   * - 0x01a9: F9P/F9R-compatible CDC-ACM (IDENTICAL architecture, 2 interfaces)
   * - 0x01ab: X20P-compatible CDC-ACM (IDENTICAL architecture, 2 interfaces)
   * - 0x050c: Vendor-specific UART1 (DIFFERENT architecture, 1 interface)
   * - 0x050d: Vendor-specific UART2 (DIFFERENT architecture, 1 interface)
   */
  auto device_info = ublox_dgnss::get_device_family_info(device_family_);
  uint8_t expected_interfaces;

  if (device_family_ == ublox_dgnss::DeviceFamily::X20P) {
    // X20P: Architecture depends on which USB device we're connecting to
    if (dev_desc.idProduct == 0x01ab) {
      // 0x01ab: F9P/F9R-compatible CDC-ACM interface (IDENTICAL to F9P/F9R)
      expected_interfaces = 2;
    } else {
      // 0x050c/0x050d: Vendor-specific UART interfaces (DIFFERENT from F9P/F9R)
      expected_interfaces = 1;
    }
  } else {
    // F9P/F9R: CDC-ACM architecture (2 interfaces: Control + Data)
    expected_interfaces = 2;
  }

  if (num_interfaces_ != expected_interfaces) {
    throw std::string("Interface count mismatch for ") + device_info.name +
          ": expected " + std::to_string(expected_interfaces) +
          ", got " + std::to_string(num_interfaces_);
  }

  // Product ID-aware serial string handling for display
  // Now that we have device descriptor, handle serial string display logic
  if (device_family_ == ublox_dgnss::DeviceFamily::X20P && strlen(serial_num_string) > 0) {
    // X20P: Only use factory iSerial for 0x050c/0x050d (reliable factory serial)
    if (dev_desc.idProduct == 0x050c || dev_desc.idProduct == 0x050d) {
      serial_str_ = std::string(serial_num_string);
    }
    // For 0x01ab: Keep original search parameter (F9P-style, may be user-programmed)
  } else {
    // F9P/F9R: Keep original search parameter (iSerial may be empty/unreliable)
    // serial_str_ remains as the user-provided search parameter
  }

  // Device interface claiming
  for (uint8_t if_num = 0; if_num < num_interfaces_; if_num++) {
    if (libusb_kernel_driver_active(devh_, if_num)) {
      libusb_detach_kernel_driver(devh_, if_num);
    }
    rc = libusb_claim_interface(devh_, if_num);
    if (rc < 0) {
      throw std::string("Error claiming interface ") + std::to_string(if_num) +
            " for " + device_info.name + ": " + libusb_error_name(rc);
    }
  }

  for (uint8_t i = 0; i < num_interfaces_; i++) {
    auto interface = &conf_desc->interface[i];
    for (uint8_t j = 0; j < interface->num_altsetting; j++) {
      auto interface_desc = &interface->altsetting[j];

      switch (interface_desc->bInterfaceClass) {
        case LIBUSB_CLASS_COMM:
          // CDC-ACM Control interface - should only have one endpoint
          ep_comms_in_addr_ = interface_desc->endpoint[0].bEndpointAddress;
          break;
        case LIBUSB_CLASS_DATA:
          // CDC-ACM Data interface - has IN and OUT endpoints
          ep_data_out_addr_ = interface_desc->endpoint[0].bEndpointAddress;
          ep_data_in_addr_ = interface_desc->endpoint[1].bEndpointAddress;
          break;
        case 255:  // LIBUSB_CLASS_VENDOR_SPEC
          // X20P devices use vendor-specific class with bulk endpoints on single interface
          for (uint8_t ep = 0; ep < interface_desc->bNumEndpoints; ep++) {
            const struct libusb_endpoint_descriptor * ep_desc = &interface_desc->endpoint[ep];

            if ((ep_desc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK) {
              if (ep_desc->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                ep_data_in_addr_ = ep_desc->bEndpointAddress;
                data_in_max_packet_size_ = ep_desc->wMaxPacketSize;
              } else {
                ep_data_out_addr_ = ep_desc->bEndpointAddress;
              }
            }
          }
          break;
        default:
          throw std::string("Error unknown bInterfaceClass: ") +
                std::to_string(interface_desc->bInterfaceClass);
      }
    }
  }
  libusb_free_config_descriptor(conf_desc);

  /*
   * CRITICAL DISTINCTION:
   * - F9P/F9R: Always CDC-ACM, always need control transfers
   * - X20P 0x01ab: F9P/F9R-compatible CDC-ACM, IDENTICAL behavior (needs control)
   * - X20P 0x050c/0x050d: Vendor-specific, NO CDC-ACM, causes LIBUSB_ERROR_PIPE if attempted
   */
  bool use_cdc_control = false;
  if (device_family_ == ublox_dgnss::DeviceFamily::X20P) {
    // X20P: CDC control ONLY for F9P-compatible interface (0x01ab)
    use_cdc_control = (dev_desc.idProduct == 0x01ab);
  } else {
    // F9P/F9R: Always use CDC-ACM control (standard behavior)
    use_cdc_control = true;
  }

  if (use_cdc_control) {
    // CDC-ACM line state setup for F9P/F9R and X20P 0x01ab
    rc = libusb_control_transfer(
      devh_, 0x21, USB_CDC_SET_CONTROL_LINE_STATE, ACM_CTRL_DTR | ACM_CTRL_RTS,
      0, NULL, 0, 0);
    if (rc < 0 && rc != LIBUSB_ERROR_BUSY) {
      throw libusb_error_name(rc);
    }
  } else {
    // X20P UART1/UART2 interfaces are not currently supported
    if (device_family_ == ublox_dgnss::DeviceFamily::X20P &&
      (dev_desc.idProduct == 0x050c || dev_desc.idProduct == 0x050d))
    {
      const char * interface_name = (dev_desc.idProduct == 0x050c) ? "UART1" : "UART2";
      std::string error_msg = std::string("X20P ") + interface_name + " interface (0x" +
        std::to_string(dev_desc.idProduct) + ") is not currently supported. " +
        "Please use the main X20P interface (0x01ab) instead. " +
        "See GitHub issue: https://github.com/aussierobots/ublox_dgnss/issues/48";

      if (debug_cb_fn_) {
        (debug_cb_fn_)(error_msg);
      }

      throw UsbException(error_msg);
    }
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
    // future libusb version not available yet in ubuntu 24.04
    // case LIBUSB_SPEED_SUPER_PLUS_X2:
    //   speed_txt = const_cast<char *>("SPEED_SUPER_PLUS_X2(20000 MBit/s)");
    //   break;
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
      driver_state_ = USBDriverState::CONNECTED;
      (debug_cb_fn_)("device opened");
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
    driver_state_ = USBDriverState::DISCONNECTED;
    (debug_cb_fn_)("device closed");
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
  if (debug_cb_fn_) {
    std::ostringstream oss;
    oss << "write_buffer: sending " << size << " bytes to endpoint 0x"
        << std::hex << ep_data_out_addr_ << std::dec;
    (debug_cb_fn_)(oss.str());
  }

  int actual_length;
  int rc = libusb_bulk_transfer(
    devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT, buf, size,
    &actual_length, timeout_ms_);  // Use timeout instead of 0

  if (debug_cb_fn_) {
    std::ostringstream oss;
    oss << "write_buffer: result rc=" << rc << " actual_length=" << actual_length
        << " requested=" << size;
    (debug_cb_fn_)(oss.str());
  }

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
        return;
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

  (debug_cb_fn_)("callback_out: status=" + std::to_string(transfer->status) +
  " length=" + std::to_string(transfer->actual_length));

  // completed flag from transfer_t stored in queue
  // bool * completed = reinterpret_cast<bool *>(transfer->user_data);

  auto sp = static_cast<std::shared_ptr<transfer_t> *>(transfer->user_data);
  (*sp)->completed = true;
  delete sp;  // cleanup

  // Always queue another transfer in if none outstanding and attached
  // (debug_cb_fn_)("callback_out: queue_count=" + std::to_string(queued_transfer_in_num()));
  if (attached_ && queued_transfer_in_num() == 0) {
    try {
      (debug_cb_fn_)("callback_out: queueing new IN transfer");
      auto transfer_in = make_transfer_in();
      submit_transfer(transfer_in, "callback_out submit transfer: ");
    } catch (const UsbException & e) {
      (exception_cb_fn_)(e, nullptr);
    }
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
          msg = "Transfer device disconnected";
        }
        return;
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

  (debug_cb_fn_)("callback_in: status=" + std::to_string(transfer->status) +
  " length=" + std::to_string(transfer->actual_length));

  // completed flag from transfer_t stored in queue
  // bool * completed = reinterpret_cast<bool *>(transfer->user_data);

  auto sp = static_cast<std::shared_ptr<transfer_t> *>(transfer->user_data);
  (*sp)->completed = true;
  delete sp;  // cleanup

  // only queue another transfer in if none outstanding and attached
  // (debug_cb_fn_)("callback_in: queue_count=" + std::to_string(queued_transfer_in_num()));
  if (attached_ && queued_transfer_in_num() == 0) {
    try {
      (debug_cb_fn_)("callback_in: queueing new IN transfer");
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

  auto transfer_out = make_transfer_out(buf, size);
  submit_transfer(transfer_out, "async submit transfer out: ");
}

std::shared_ptr<transfer_t> Connection::make_transfer_out(u_char * buf, size_t size)
{
  auto transfer = std::make_shared<transfer_t>();
  transfer->type = USB_OUT;
  transfer->buffer->resize(size);
  std::memcpy(transfer->buffer->data(), buf, size);
  transfer->completed = false;

  // void * user_data = &transfer->completed;
  transfer->usb_transfer->user_data = new std::shared_ptr<transfer_t>(transfer);
  transfer->usb_transfer->flags = LIBUSB_TRANSFER_SHORT_NOT_OK;

  libusb_fill_bulk_transfer(
    transfer->usb_transfer, devh_, ep_data_out_addr_ | LIBUSB_ENDPOINT_OUT,
    // buf, size,
    transfer->buffer->data(), transfer->buffer->size(),
    callback_out_fn_, transfer->usb_transfer->user_data, 0
  );

  return transfer;
}

std::shared_ptr<transfer_t> Connection::make_transfer_in()
{
  auto transfer = std::make_shared<transfer_t>();
  transfer->type = USB_IN;
  transfer->buffer->resize(IN_BUFFER_SIZE);
  transfer->completed = false;

  // void * user_data = &transfer->completed;
  transfer->usb_transfer->user_data = new std::shared_ptr<transfer_t>(transfer);

  // setup asynchronous transfer in to host from usb
  libusb_fill_bulk_transfer(
    transfer->usb_transfer, devh_, ep_data_in_addr_ | LIBUSB_ENDPOINT_IN,
    // in_buffer_, IN_BUFFER_SIZE,
    transfer->buffer->data(), transfer->buffer->size(),
    callback_in_fn_, transfer->usb_transfer->user_data, 0);

  return transfer;
}

void Connection::submit_transfer(
  std::shared_ptr<transfer_t> transfer, const std::string msg,
  bool wait_for_completed)
{
  (void)wait_for_completed;

  if (keep_running_ && attached_) {
    if (!transfer->usb_transfer) {
      throw UsbException("transfer->usb_transfer is null");
    }
    int rc = libusb_submit_transfer(transfer->usb_transfer);
    if (rc < 0) {
      std::string exception_msg = msg;
      exception_msg.append(libusb_error_name(rc));
      throw UsbException(exception_msg);
    }

    // (debug_cb_fn_)("submit_transfer: " + msg + " rc=" + std::to_string(rc));

    // only adding those that were succesfully submitted to the queue
    {
      const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);
      transfer_queue_.push_back(transfer);
    }

    // remove completed from the queue
    cleanup_transfer_queue();
  }
}

void Connection::cleanup_transfer_queue()
{
  if (transfer_queue_.size() == 0) {return;}

  if (debug_cb_fn_ == nullptr) {
    throw UsbException("cleanup_transfer_queue - debug_cb_fn_ not set");
  }

  const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);

  // remove all completed transfer entries
  auto it = transfer_queue_.begin();
  while (it != transfer_queue_.end()) {
    if ((*it)->completed) {
      // build up debug message
      std::string msg = "cleanup_transfer_queue: ";
      auto type = (*it)->type;
      if (type == USB_IN) {
        msg.append(" USB_IN completed. transfer - ");
      } else {
        msg.append(" USB_OUT completed. transfer - ");
      }
      switch ((*it)->usb_transfer->status) {
        case LIBUSB_TRANSFER_COMPLETED:
          msg.append("LIBUSB_TRANSFER_COMPLETED");
          break;
        case LIBUSB_TRANSFER_ERROR:
          msg.append("LIBUSB_TRANSFER_ERROR");
          break;
        case LIBUSB_TRANSFER_TIMED_OUT:
          msg.append("LIBUSB_TRANSFER_TIMED_OUT");
          break;
        case LIBUSB_TRANSFER_CANCELLED:
          msg.append("LIBUSB_TRANSFER_CANCELLED");
          break;
        case LIBUSB_TRANSFER_STALL:
          msg.append("LIBUSB_TRANSFER_STALL");
          break;
        case LIBUSB_TRANSFER_NO_DEVICE:
          msg.append("LIBUSB_TRANSFER_STALL");
          break;
        case LIBUSB_TRANSFER_OVERFLOW:
          msg.append("LIBUSB_TRANSFER_STALL");
          break;
        default:
          msg.append(" UNKNOWN LIBUSB TRANSFER STATUS");
      }
      (debug_cb_fn_)(msg);

      transfer_queue_.erase(it++);
    } else {
      ++it;
    }
  }
}

void Connection::cleanup_all_transfers()
{
  if (transfer_queue_.size() == 0) {
    return;
  }

  if (debug_cb_fn_ == nullptr) {
    throw UsbException("cleanup_all_transfers - debug_cb_fn_ not set");
  }

  (debug_cb_fn_)("cleanup_all_transfers: canceling " + std::to_string(transfer_queue_.size()) +
  " transfers");

  {
    const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);

    // Cancel all active transfers
    for (auto & transfer : transfer_queue_) {
      if (!transfer->completed && transfer->usb_transfer) {
        int rc = libusb_cancel_transfer(transfer->usb_transfer);
        if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_NOT_FOUND) {
          (debug_cb_fn_)("cleanup_all_transfers: failed to cancel transfer: " +
          std::string(libusb_error_name(rc)));
        }
      }
    }

    // Clear the entire queue
    transfer_queue_.clear();
  }

  (debug_cb_fn_)("cleanup_all_transfers: transfer queue cleared");
}

size_t Connection::queued_transfer_in_num()
{
  if (transfer_queue_.size() == 0) {return 0;}

  const std::lock_guard<std::mutex> lock(transfer_queue_mutex_);

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
  if (devh_ == nullptr) {
    throw UsbException("No device handle set");
  }
  if (dev_ == nullptr) {
    throw UsbException("No device set");
  }
  if (in_cb_fn_ == nullptr) {
    throw UsbException("No in callback function set");
  }
  if (out_cb_fn_ == nullptr) {
    throw UsbException("No out callback function set");
  }
  if (exception_cb_fn_ == nullptr) {
    throw UsbException("No exception callback function set");
  }
  if (debug_cb_fn_ == nullptr) {
    throw UsbException("No debug callback function set");
  }

  (debug_cb_fn_)("init_async: submitting initial IN transfer");

  // submit initial transfer in request
  // - at first get a few zero length records
  auto transfer_in = make_transfer_in();
  submit_transfer(transfer_in, "init_async transfer: ", false);
}

void Connection::handle_usb_events()
{
  if (!keep_running_) {return;}

  // don’t call into libusb until init() has succeeded
  if (ctx_ == nullptr) {return;}

  // int rc = libusb_handle_events_timeout_completed(ctx_, &timeout_tv_, &usb_event_completed_);
  int rc = libusb_handle_events_timeout(ctx_, &timeout_tv_);
  switch (rc) {
    case LIBUSB_ERROR_INTERRUPTED:
      keep_running_ = false;
      break;
    case LIBUSB_ERROR_NO_DEVICE: {
        if (++no_device_streak_ >= kNoDeviceThreshold) {
          attached_ = false;
        }
      }
      break;
    default:
      break;
  }

  if (rc >= 0) {
    no_device_streak_ = 0;
  }

  if (rc < 0) {
    throw UsbException(libusb_error_name(rc));
  }
}

void Connection::close_devh()
{
  // Clean up all pending transfers before closing device
  cleanup_transfer_queue();  // completed
  cleanup_all_transfers();  // any remaining
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
  for (auto handle : hp_attach_) {
    if (handle) {
      libusb_hotplug_deregister_callback(ctx_, handle);
    }
  }
  for (auto handle : hp_detach_) {
    if (handle) {
      libusb_hotplug_deregister_callback(ctx_, handle);
    }
  }

  close_devh();
}

Connection::~Connection()
{
  shutdown();

  libusb_exit(ctx_);
}
}  // namespace usb
