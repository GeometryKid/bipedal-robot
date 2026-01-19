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

class YesenseImuStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fusion_status = null;
      this.gnss_status = null;
    }
    else {
      if (initObj.hasOwnProperty('fusion_status')) {
        this.fusion_status = initObj.fusion_status
      }
      else {
        this.fusion_status = 0;
      }
      if (initObj.hasOwnProperty('gnss_status')) {
        this.gnss_status = initObj.gnss_status
      }
      else {
        this.gnss_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseImuStatus
    // Serialize message field [fusion_status]
    bufferOffset = _serializer.uint8(obj.fusion_status, buffer, bufferOffset);
    // Serialize message field [gnss_status]
    bufferOffset = _serializer.uint8(obj.gnss_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseImuStatus
    let len;
    let data = new YesenseImuStatus(null);
    // Deserialize message field [fusion_status]
    data.fusion_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gnss_status]
    data.gnss_status = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseImuStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce0abe5eeaff955681f09594f0484f51';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 fusion_status
    uint8 gnss_status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YesenseImuStatus(null);
    if (msg.fusion_status !== undefined) {
      resolved.fusion_status = msg.fusion_status;
    }
    else {
      resolved.fusion_status = 0
    }

    if (msg.gnss_status !== undefined) {
      resolved.gnss_status = msg.gnss_status;
    }
    else {
      resolved.gnss_status = 0
    }

    return resolved;
    }
};

module.exports = YesenseImuStatus;
