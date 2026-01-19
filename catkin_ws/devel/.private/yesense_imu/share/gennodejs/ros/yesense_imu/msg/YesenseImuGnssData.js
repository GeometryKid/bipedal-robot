// Auto-generated. Do not edit!

// (in-package yesense_imu.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let YesenseImuMasterGnssData = require('./YesenseImuMasterGnssData.js');
let YesenseImuSlaveGnssData = require('./YesenseImuSlaveGnssData.js');

//-----------------------------------------------------------

class YesenseImuGnssData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.master = null;
      this.slave = null;
    }
    else {
      if (initObj.hasOwnProperty('master')) {
        this.master = initObj.master
      }
      else {
        this.master = new YesenseImuMasterGnssData();
      }
      if (initObj.hasOwnProperty('slave')) {
        this.slave = initObj.slave
      }
      else {
        this.slave = new YesenseImuSlaveGnssData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseImuGnssData
    // Serialize message field [master]
    bufferOffset = YesenseImuMasterGnssData.serialize(obj.master, buffer, bufferOffset);
    // Serialize message field [slave]
    bufferOffset = YesenseImuSlaveGnssData.serialize(obj.slave, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseImuGnssData
    let len;
    let data = new YesenseImuGnssData(null);
    // Deserialize message field [master]
    data.master = YesenseImuMasterGnssData.deserialize(buffer, bufferOffset);
    // Deserialize message field [slave]
    data.slave = YesenseImuSlaveGnssData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseImuGnssData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ae993c6fa242f94866ceb31ffc6e7d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    yesense_imu/YesenseImuMasterGnssData master
    yesense_imu/YesenseImuSlaveGnssData slave
    ================================================================================
    MSG: yesense_imu/YesenseImuMasterGnssData
    yesense_imu/YesenseImuUtcTime utc_time
    yesense_imu/YesenseImuLocation location
    yesense_imu/YesenseImuLocation location_error
    float32 speed
    float32 yaw
    uint8 status
    uint8 star_cnt
    float32 p_dop
    uint8 site_id
    ================================================================================
    MSG: yesense_imu/YesenseImuUtcTime
    uint16 year
    uint8 month
    uint8 date
    uint8 hour
    uint8 min
    uint8 sec
    uint16 ms
    ================================================================================
    MSG: yesense_imu/YesenseImuLocation
    float64 longtidue
    float64 latitude
    float32 altidue
    ================================================================================
    MSG: yesense_imu/YesenseImuSlaveGnssData
    float32 dual_ant_yaw
    float32 dual_ant_yaw_error
    float32 dual_ant_baseline_len
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YesenseImuGnssData(null);
    if (msg.master !== undefined) {
      resolved.master = YesenseImuMasterGnssData.Resolve(msg.master)
    }
    else {
      resolved.master = new YesenseImuMasterGnssData()
    }

    if (msg.slave !== undefined) {
      resolved.slave = YesenseImuSlaveGnssData.Resolve(msg.slave)
    }
    else {
      resolved.slave = new YesenseImuSlaveGnssData()
    }

    return resolved;
    }
};

module.exports = YesenseImuGnssData;
