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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_CFG_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_CFG_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <vector>
#include <string>
#include "ublox_dgnss_node/ubx/ubx_cfg_item.hpp"
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::cfg
{
// layers_t used for val set
struct layers_t
{
  union {
    x1_t all;
    struct
    {
      l_t ram : 1;
      l_t bbr : 1;
      l_t flash : 1;
    } bits;
  };
};

// layer_t used for val get
enum layer_t : u1_t {RAM_LAYER = 0, BBR_LAYER = 1, FLASH_LAYER = 2, DEFAULT_LAYER = 7};

class CfgValGetPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_CFG;
  static const msg_id_t MSG_ID = UBX_CFG_VALGET;

  u1_t version;                 // message versions (0x00 for this version)
  u1_t layer;                   // 0 - RAM, 1 - BBR, 2 - Flash, 7 - Default
  u2_t position;                // skip this many key values before constructin output message
  std::vector<ubx_key_id_t> keys;       // configuration id of keys
  std::vector<key_value_t> cfg_data;       // configuration data - key & value pairs

public:
  CfgValGetPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    version = 0x00;
    layer = layer_t::RAM_LAYER;
    position = 0x0000;
  }

  CfgValGetPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = payload_[0];
    layer = payload_[1];
    position = *reinterpret_cast<u2_t *>(&payload_[2]);
    size_t idx = 4;
    // exttract key value data -
    // key will be 4 bytes + at least 1 byte for value
    while (idx < static_cast<size_t>(size) - 4) {
      // create key and increment payload ptr index
      u4_t key_id = *reinterpret_cast<u4_t *>(&payload_[idx]);
      auto ubx_key_id = ubx_key_id_t {key_id};
      idx += sizeof(u4_t);

      // extract packed value and increment payload ptr index by its storage size
      unsigned char bytes[8];
      memset(&bytes, 0x00, sizeof(bytes));
      size_t value_size = ubx_key_id.storage_size();
      memcpy(&bytes, &payload_[idx], value_size);
      idx += value_size;

      // create and append key value configuration data
      auto key_value = key_value_t {ubx_key_id, *bytes};
      cfg_data.push_back(key_value);
    }
  }
  std::tuple<u1_t *, size_t> make_poll_payload() override
  {
    payload_.clear();
    payload_.push_back(version);
    payload_.push_back(layer);
    buf_append_u2(&payload_, position);
    for (auto k : keys) {
      buf_append_u4(&payload_, k.key_id());
    }
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: 0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << +version;
    oss << " layer: 0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << +layer;
    oss << " position: 0x" << std::setfill('0') << std::setw(4) << std::right << std::hex <<
      +position;
    oss << " cfg_data - key values(" << cfg_data.size() << "):";
    for (auto kv : cfg_data) {
      oss << " " << kv.ubx_key_id.to_hex() << ":0x";
      for (size_t i = 0; i < kv.ubx_key_id.storage_size(); i++) {
        oss << std::setfill('0') << std::setw(2) << std::right << std::hex <<
          +kv.ubx_value.bytes[i];
      }
    }
    return oss.str();
  }
};


inline void buf_append_keyvalue(std::vector<u1_t> * buf, key_value_t * kv)
{
  // key will be 4 bytes
  buf_append_u4(buf, kv->ubx_key_id.key_id());

  ubx_cfg_item_t cfg_item;

  auto cfg_item_search = ubxKeyCfgItemMap.find(kv->ubx_key_id);
  if (cfg_item_search == ubxKeyCfgItemMap.end()) {
    std::string msg = "ubx_key_id: " + kv->ubx_key_id.to_hex() + " not found in ubxKeyCfgItemMap";
    throw UbxValueException(msg);
  } else {
    cfg_item = cfg_item_search->second;
  }

  // value will be variable from 1 to 8 byte
  switch (cfg_item.ubx_type) {
    case L:
      buf->push_back((u1_t)kv->ubx_value.l);
      break;
    case U1:
      buf->push_back(kv->ubx_value.u1);
      break;
    case I1:
      buf->push_back(kv->ubx_value.i1);
      break;
    case X1:
      buf->push_back(kv->ubx_value.x1);
      break;
    case E1:
      buf->push_back(kv->ubx_value.u1);
      break;
    case U2:
      buf_append_u2(buf, kv->ubx_value.u2);
      break;
    case I2:
      buf_append_i2(buf, kv->ubx_value.i2);
      break;
    case X2:
      buf_append_x2(buf, kv->ubx_value.x2);
      break;
    case U4:
      buf_append_u4(buf, kv->ubx_value.u2);
      break;
    case I4:
      buf_append_i4(buf, kv->ubx_value.i2);
      break;
    case X4:
      buf_append_x4(buf, kv->ubx_value.x2);
      break;
    case R4:
      buf_append_r4(buf, kv->ubx_value.r4);
      break;
    case R8:
      buf_append_r8(buf, kv->ubx_value.r8);
      break;
    default:
      std::string msg = "ubx_type: " + std::to_string(cfg_item.ubx_type) + " not implemented";
      throw UbxValueException(msg);
  }
}

// transaction_t used for val set
struct transaction_t
{
  union {
    u1_t all;
    struct
    {
      u1_t action : 2;  // refer UBX Protocol 0=transactionless, 1 = restart, 2 = ongoing,
                        // 3 = apply and end a set transaction
    } bits;
  };
};

class CfgValSetPayload : protected UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_CFG;
  static const msg_id_t MSG_ID = UBX_CFG_VALSET;

  u1_t version;                       // message versions (0x00 for this version)
  layers_t layers;                    // bit 0 - RAM, bit 1 - BBR, bit 2 - Flash
  transaction_t transaction;
  u1_t reserved0;
  std::vector<key_value_t> cfg_data;  // configuration data - key & value pairs

  CfgValSetPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    version = 0x00;
    layers.all = 0x00000000;
    transaction.all = 0x00000000;
    reserved0 = 0x0;
    cfg_data.clear();
  }
  std::tuple<u1_t *, size_t> make_poll_payload() override
  {
    payload_.clear();
    payload_.push_back(version);
    payload_.push_back(layers.all);
    payload_.push_back(transaction.all);
    payload_.push_back(reserved0);
    for (auto kv : cfg_data) {
      buf_append_keyvalue(&payload_, &kv);
    }
    return std::make_tuple(payload_.data(), payload_.size());
  }
};

struct valset_payload_poll_t : public CfgValSetPayload, public UBXPayloadOutputPrint
{
  std::tuple<u1_t *, size_t> make_poll_payload() override
  {
    return CfgValSetPayload::make_poll_payload();
  }
};

// BBR sections to clear. The following special sets apply:
// - 0x0000 Hot start
// - 0x0001 Warm start
// - 0xFFFF Cold start
struct nav_bbr_mask_t
{
  union {
    x2_t all;
    struct
    {
      l_t eph : 1;                   // Ephemeris
      l_t alm : 1;                   // Almanac
      l_t health : 1;                   // Health
      l_t klob : 1;                   // Klobuchar parameters
      l_t pos : 1;                   // Position
      l_t clkd : 1;                   // Clock drift
      l_t osc : 1;                   // Oscillator drift
      l_t utc : 1;                   // UTC Correction + GPS Leap seconds parameters
      l_t rtc : 1;                   // RTC
      l_t aop : 1;
    } bits;
  };
};
const nav_bbr_mask_t NAV_BBR_HOT_START {0x0000};
const nav_bbr_mask_t NAV_BBR_WARM_START {0x0001};
const nav_bbr_mask_t NAV_BBR_COLD_START {0xFFFF};

class CfgRstPayload : protected UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_CFG;
  static const msg_id_t MSG_ID = UBX_CFG_RST;

  nav_bbr_mask_t navBbrMask;
  u1_t resetMode;
  u1_t reserved0 = 0x00;

  CfgRstPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  std::tuple<u1_t *, size_t> make_poll_payload() override
  {
    payload_.clear();
    buf_append_x2(&payload_, navBbrMask.all);
    payload_.push_back(resetMode);
    payload_.push_back(reserved0);
    return std::make_tuple(payload_.data(), payload_.size());
  }
};


class UbxCfg
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<FrameContainer<cfg::CfgValGetPayload>> cfg_valget_;

  std::shared_ptr<ubx::FrameValSet> valset_frame_poll_;      // ubx frame for val set
  std::shared_ptr<valset_payload_poll_t> valset_payload_poll_;

  std::shared_ptr<FrameContainer<cfg::CfgRstPayload>> cfg_rst_;

public:
  explicit UbxCfg(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    cfg_valget_ = std::make_shared<FrameContainer<cfg::CfgValGetPayload>>();
    valset_payload_poll_ = std::make_shared<valset_payload_poll_t>();
    cfg_rst_ = std::make_shared<FrameContainer<cfg::CfgRstPayload>>();
  }
  std::shared_ptr<ubx::Frame> cfg_val_get_frame_poll()
  {
    return cfg_valget_->frame_poll();
  }
  std::shared_ptr<Payload<cfg::CfgValGetPayload>> cfg_val_get_payload()
  {
    return cfg_valget_->payload();
  }
  std::shared_ptr<ubx::Frame> cfg_val_get_frame()
  {
    return cfg_valget_->frame();
  }
  std::shared_ptr<PayloadPoll<cfg::CfgValGetPayload>> cfg_val_get_payload_poll()
  {
    return cfg_valget_->payload_poll();
  }
  std::shared_ptr<ubx::Frame> cfg_rst_frame_poll()
  {
    return cfg_rst_->frame_poll();
  }
  std::shared_ptr<PayloadPoll<cfg::CfgRstPayload>> cfg_rst_payload_poll()
  {
    return cfg_rst_->payload_poll();
  }
  std::shared_ptr<valset_payload_poll_t> cfg_val_set_payload_poll()
  {
    return valset_payload_poll_;
  }
  void cfg_val_get_key_append(ubx_cfg_item_t ubx_cfg_item)
  {
    cfg_val_get_key_append(ubx_cfg_item.ubx_key_id);
  }
  void cfg_val_get_key_append(ubx_key_id_t ubx_key_id)
  {
    cfg_val_get_payload_poll()->keys.push_back(ubx_key_id);
  }
  void cfg_val_get_keys_clear()
  {
    cfg_val_get_payload_poll()->keys.clear();
  }
  size_t cfg_val_get_keys_size()
  {
    return cfg_val_get_payload_poll()->keys.size();
  }
  void cfg_set_val_get_layer_ram()
  {
    cfg_val_get_payload_poll()->layer = layer_t::RAM_LAYER;
  }
  void cfg_set_val_get_layer_BBR()
  {
    cfg_val_get_payload_poll()->layer = layer_t::BBR_LAYER;
  }
  void cfg_set_val_get_layer_flash()
  {
    cfg_val_get_payload_poll()->layer = layer_t::FLASH_LAYER;
  }
  void cfg_set_val_get_layer_default()
  {
    cfg_val_get_payload_poll()->layer = layer_t::DEFAULT_LAYER;
  }
  void cfg_val_set_key_append(ubx_cfg_item_t ubx_cfg_item, l_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for :";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.l = static_cast<bool>(value);
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.l=(bool)value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_u1(ubx_cfg_item_t ubx_cfg_item, u1_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for :";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.u1 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.u1 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_i1(ubx_cfg_item_t ubx_cfg_item, i1_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for :";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.i1 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.i1 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_x1(ubx_cfg_item_t ubx_cfg_item, x1_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.x1 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.x1 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_u2(ubx_cfg_item_t ubx_cfg_item, u2_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.u2 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.u2 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_i2(ubx_cfg_item_t ubx_cfg_item, i2_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.i2 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.i2 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_x2(ubx_cfg_item_t ubx_cfg_item, x2_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.x2 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.x2 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_u4(ubx_cfg_item_t ubx_cfg_item, u4_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.u4 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.u4 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_i4(ubx_cfg_item_t ubx_cfg_item, i4_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.i4 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.i4 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_x4(ubx_cfg_item_t ubx_cfg_item, x4_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.x4 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.x4 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_r4(ubx_cfg_item_t ubx_cfg_item, r4_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.r4 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.r4 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_u8(ubx_cfg_item_t ubx_cfg_item, u8_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.u8 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.u8 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_i8(ubx_cfg_item_t ubx_cfg_item, i8_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.i8 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.i8 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_r8(ubx_cfg_item_t ubx_cfg_item, r8_t value)
  {
    if (sizeof(value) != ubx_cfg_item.ubx_key_id.storage_size()) {
      std::string msg = "wrong value type for ";
      msg.append(ubx_cfg_item.ubx_key_id.to_hex());
      throw UbxValueException(msg);
    }
    value_t v;
    v.r8 = value;
    // cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value_t {.r8 = value});
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, v);
  }
  void cfg_val_set_key_append_binary(ubx_cfg_item_t ubx_cfg_item, u1_t * value_p)
  {
    value_t value;
    memset(value.bytes, 0x00, sizeof(value.bytes));
    memcpy(&value.bytes[0], value_p, ubx_cfg_item.ubx_key_id.storage_size());
    cfg_val_set_key_append(ubx_cfg_item.ubx_key_id, value);
  }
  void cfg_val_set_key_append(ubx_key_id_t ubx_key_id, value_t value)
  {
    key_value_t vs {ubx_key_id, value};
    valset_payload_poll_->cfg_data.push_back(vs);
  }
  void cfg_val_set_cfgdata_clear()
  {
    valset_payload_poll_->cfg_data.clear();
  }
  size_t cfg_val_set_cfgdata_size()
  {
    return valset_payload_poll_->cfg_data.size();
  }
  void cfg_val_set_layer_ram(bool bit)
  {
    valset_payload_poll_->layers.bits.ram = bit;
  }
  void cfg_val_set_layer_BBR(bool bit)
  {
    valset_payload_poll_->layers.bits.bbr = bit;
  }
  void cfg_val_set_layer_flash(bool bit)
  {
    valset_payload_poll_->layers.bits.flash = bit;
  }
  void cfg_val_set_transaction(u1_t action)
  {
    if (action > 3) {
      throw UbxValueException("transaction action value must be between 0 and 3");
    }
    valset_payload_poll_->transaction.bits.action = action;
  }

  std::shared_ptr<ubx::FrameValSet> cfg_val_set_frame()
  {
    return valset_frame_poll_;
  }

  void cfg_val_set_frame_reset()
  {
    valset_frame_poll_.reset();
  }

  std::shared_ptr<ubx::FrameValSet> cfg_val_set_frame_poll()
  {
    u1_t * payload;
    size_t payload_size;

    std::tie(payload, payload_size) = valset_payload_poll_->make_poll_payload();

    auto val_set_frame = std::make_shared<ubx::FrameValSet>();
    val_set_frame->msg_class = UBX_CFG;
    val_set_frame->msg_id = UBX_CFG_VALSET;
    val_set_frame->payload = reinterpret_cast<ch_t *>(payload);
    val_set_frame->length = payload_size;
    std::tie(val_set_frame->ck_a, val_set_frame->ck_b) = val_set_frame->ubx_check_sum();
    val_set_frame->build_frame_buf();

    return val_set_frame;
  }
  void set_cfg_val_get_frame(std::shared_ptr<ubx::Frame> frame)
  {
    cfg_valget_->frame(frame);
  }

  void cfg_val_get_poll_async()
  {
    auto frame_poll = cfg_valget_->make_frame_poll();
    usbc_->write_buffer_async(frame_poll->buf.data(), frame_poll->buf.size(), NULL);
  }

  void cfg_val_get_poll_async_all_layers()
  {
    // not all layers have values set
    // so start with defaults moving between the different layers
    // such that we endup wih the active layer's values
    cfg_set_val_get_layer_default();
    cfg_val_get_poll_async();
    // cfg_set_val_get_layer_flash();
    // cfg_val_get_poll_async();
    // cfg_set_val_get_layer_BBR();
    // cfg_val_get_poll_async();
    cfg_set_val_get_layer_ram();
    cfg_val_get_poll_async();
  }

  void cfg_val_set_poll_async()
  {
    valset_frame_poll_ = cfg_val_set_frame_poll();
    usbc_->write_buffer_async(valset_frame_poll_->buf.data(), valset_frame_poll_->buf.size(), NULL);
  }

  void cfg_val_set_poll_retry_async()
  {
    cfg_val_set_transaction(0);
    valset_frame_poll_ = cfg_val_set_frame_poll();
    usbc_->write_buffer_async(valset_frame_poll_->buf.data(), valset_frame_poll_->buf.size(), NULL);
  }

  void cfg_rst_set_nav_bbr_mask(nav_bbr_mask_t navBbrMask)
  {
    cfg_rst_payload_poll()->navBbrMask = navBbrMask;
  }
  void cfg_rst_set_reset_mode(u1_t resetMode)
  {
    cfg_rst_payload_poll()->resetMode = resetMode;
  }
  void cfg_rst_command_async()
  {
    auto frame_poll = cfg_rst_->make_frame_poll();
    usbc_->write_buffer_async(frame_poll->buf.data(), frame_poll->buf.size(), NULL);
  }
};
}  // namespace ubx::cfg

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_CFG_HPP_
