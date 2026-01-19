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

class YesenseImuLocation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.longtidue = null;
      this.latitude = null;
      this.altidue = null;
    }
    else {
      if (initObj.hasOwnProperty('longtidue')) {
        this.longtidue = initObj.longtidue
      }
      else {
        this.longtidue = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('altidue')) {
        this.altidue = initObj.altidue
      }
      else {
        this.altidue = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseImuLocation
    // Serialize message field [longtidue]
    bufferOffset = _serializer.float64(obj.longtidue, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [altidue]
    bufferOffset = _serializer.float32(obj.altidue, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseImuLocation
    let len;
    let data = new YesenseImuLocation(null);
    // Deserialize message field [longtidue]
    data.longtidue = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altidue]
    data.altidue = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseImuLocation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cf93867aa740e2448c799a9da22809d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 longtidue
    float64 latitude
    float32 altidue
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YesenseImuLocation(null);
    if (msg.longtidue !== undefined) {
      resolved.longtidue = msg.longtidue;
    }
    else {
      resolved.longtidue = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.altidue !== undefined) {
      resolved.altidue = msg.altidue;
    }
    else {
      resolved.altidue = 0.0
    }

    return resolved;
    }
};

module.exports = YesenseImuLocation;
