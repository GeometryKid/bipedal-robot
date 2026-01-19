// Auto-generated. Do not edit!

// (in-package yesense_imu.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class YesenseImuSlaveGnssData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dual_ant_yaw = null;
      this.dual_ant_yaw_error = null;
      this.dual_ant_baseline_len = null;
    }
    else {
      if (initObj.hasOwnProperty('dual_ant_yaw')) {
        this.dual_ant_yaw = initObj.dual_ant_yaw
      }
      else {
        this.dual_ant_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('dual_ant_yaw_error')) {
        this.dual_ant_yaw_error = initObj.dual_ant_yaw_error
      }
      else {
        this.dual_ant_yaw_error = 0.0;
      }
      if (initObj.hasOwnProperty('dual_ant_baseline_len')) {
        this.dual_ant_baseline_len = initObj.dual_ant_baseline_len
      }
      else {
        this.dual_ant_baseline_len = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseImuSlaveGnssData
    // Serialize message field [dual_ant_yaw]
    bufferOffset = _serializer.float32(obj.dual_ant_yaw, buffer, bufferOffset);
    // Serialize message field [dual_ant_yaw_error]
    bufferOffset = _serializer.float32(obj.dual_ant_yaw_error, buffer, bufferOffset);
    // Serialize message field [dual_ant_baseline_len]
    bufferOffset = _serializer.float32(obj.dual_ant_baseline_len, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseImuSlaveGnssData
    let len;
    let data = new YesenseImuSlaveGnssData(null);
    // Deserialize message field [dual_ant_yaw]
    data.dual_ant_yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dual_ant_yaw_error]
    data.dual_ant_yaw_error = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dual_ant_baseline_len]
    data.dual_ant_baseline_len = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseImuSlaveGnssData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8343f347c57f056bf546331417a9a305';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new YesenseImuSlaveGnssData(null);
    if (msg.dual_ant_yaw !== undefined) {
      resolved.dual_ant_yaw = msg.dual_ant_yaw;
    }
    else {
      resolved.dual_ant_yaw = 0.0
    }

    if (msg.dual_ant_yaw_error !== undefined) {
      resolved.dual_ant_yaw_error = msg.dual_ant_yaw_error;
    }
    else {
      resolved.dual_ant_yaw_error = 0.0
    }

    if (msg.dual_ant_baseline_len !== undefined) {
      resolved.dual_ant_baseline_len = msg.dual_ant_baseline_len;
    }
    else {
      resolved.dual_ant_baseline_len = 0.0
    }

    return resolved;
    }
};

module.exports = YesenseImuSlaveGnssData;
