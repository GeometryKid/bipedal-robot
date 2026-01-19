// Auto-generated. Do not edit!

// (in-package yesense_imu.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let YesenseImuUtcTime = require('./YesenseImuUtcTime.js');
let YesenseImuLocation = require('./YesenseImuLocation.js');

//-----------------------------------------------------------

class YesenseImuMasterGnssData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.utc_time = null;
      this.location = null;
      this.location_error = null;
      this.speed = null;
      this.yaw = null;
      this.status = null;
      this.star_cnt = null;
      this.p_dop = null;
      this.site_id = null;
    }
    else {
      if (initObj.hasOwnProperty('utc_time')) {
        this.utc_time = initObj.utc_time
      }
      else {
        this.utc_time = new YesenseImuUtcTime();
      }
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = new YesenseImuLocation();
      }
      if (initObj.hasOwnProperty('location_error')) {
        this.location_error = initObj.location_error
      }
      else {
        this.location_error = new YesenseImuLocation();
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('star_cnt')) {
        this.star_cnt = initObj.star_cnt
      }
      else {
        this.star_cnt = 0;
      }
      if (initObj.hasOwnProperty('p_dop')) {
        this.p_dop = initObj.p_dop
      }
      else {
        this.p_dop = 0.0;
      }
      if (initObj.hasOwnProperty('site_id')) {
        this.site_id = initObj.site_id
      }
      else {
        this.site_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseImuMasterGnssData
    // Serialize message field [utc_time]
    bufferOffset = YesenseImuUtcTime.serialize(obj.utc_time, buffer, bufferOffset);
    // Serialize message field [location]
    bufferOffset = YesenseImuLocation.serialize(obj.location, buffer, bufferOffset);
    // Serialize message field [location_error]
    bufferOffset = YesenseImuLocation.serialize(obj.location_error, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [star_cnt]
    bufferOffset = _serializer.uint8(obj.star_cnt, buffer, bufferOffset);
    // Serialize message field [p_dop]
    bufferOffset = _serializer.float32(obj.p_dop, buffer, bufferOffset);
    // Serialize message field [site_id]
    bufferOffset = _serializer.uint8(obj.site_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseImuMasterGnssData
    let len;
    let data = new YesenseImuMasterGnssData(null);
    // Deserialize message field [utc_time]
    data.utc_time = YesenseImuUtcTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [location]
    data.location = YesenseImuLocation.deserialize(buffer, bufferOffset);
    // Deserialize message field [location_error]
    data.location_error = YesenseImuLocation.deserialize(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [star_cnt]
    data.star_cnt = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [p_dop]
    data.p_dop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [site_id]
    data.site_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseImuMasterGnssData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '707ba72b8fe698890901dc5ceaf55863';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YesenseImuMasterGnssData(null);
    if (msg.utc_time !== undefined) {
      resolved.utc_time = YesenseImuUtcTime.Resolve(msg.utc_time)
    }
    else {
      resolved.utc_time = new YesenseImuUtcTime()
    }

    if (msg.location !== undefined) {
      resolved.location = YesenseImuLocation.Resolve(msg.location)
    }
    else {
      resolved.location = new YesenseImuLocation()
    }

    if (msg.location_error !== undefined) {
      resolved.location_error = YesenseImuLocation.Resolve(msg.location_error)
    }
    else {
      resolved.location_error = new YesenseImuLocation()
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.star_cnt !== undefined) {
      resolved.star_cnt = msg.star_cnt;
    }
    else {
      resolved.star_cnt = 0
    }

    if (msg.p_dop !== undefined) {
      resolved.p_dop = msg.p_dop;
    }
    else {
      resolved.p_dop = 0.0
    }

    if (msg.site_id !== undefined) {
      resolved.site_id = msg.site_id;
    }
    else {
      resolved.site_id = 0
    }

    return resolved;
    }
};

module.exports = YesenseImuMasterGnssData;
