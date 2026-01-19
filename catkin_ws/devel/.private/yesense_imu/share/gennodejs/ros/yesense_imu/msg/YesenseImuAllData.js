// Auto-generated. Do not edit!

// (in-package yesense_imu.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let YesenseImuQuaternion = require('./YesenseImuQuaternion.js');
let YesenseImuEulerAngle = require('./YesenseImuEulerAngle.js');
let YesenseImuLocation = require('./YesenseImuLocation.js');
let YesenseImuStatus = require('./YesenseImuStatus.js');
let YesenseImuGnssData = require('./YesenseImuGnssData.js');
let YesenseImuGpsData = require('./YesenseImuGpsData.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class YesenseImuAllData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.temperature = null;
      this.sample_timestamp = null;
      this.sync_timestamp = null;
      this.accel = null;
      this.quaternion = null;
      this.eulerAngle = null;
      this.location = null;
      this.status = null;
      this.gnss = null;
      this.gps = null;
    }
    else {
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
      }
      if (initObj.hasOwnProperty('sample_timestamp')) {
        this.sample_timestamp = initObj.sample_timestamp
      }
      else {
        this.sample_timestamp = 0;
      }
      if (initObj.hasOwnProperty('sync_timestamp')) {
        this.sync_timestamp = initObj.sync_timestamp
      }
      else {
        this.sync_timestamp = 0;
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = new geometry_msgs.msg.Accel();
      }
      if (initObj.hasOwnProperty('quaternion')) {
        this.quaternion = initObj.quaternion
      }
      else {
        this.quaternion = new YesenseImuQuaternion();
      }
      if (initObj.hasOwnProperty('eulerAngle')) {
        this.eulerAngle = initObj.eulerAngle
      }
      else {
        this.eulerAngle = new YesenseImuEulerAngle();
      }
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = new YesenseImuLocation();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new YesenseImuStatus();
      }
      if (initObj.hasOwnProperty('gnss')) {
        this.gnss = initObj.gnss
      }
      else {
        this.gnss = new YesenseImuGnssData();
      }
      if (initObj.hasOwnProperty('gps')) {
        this.gps = initObj.gps
      }
      else {
        this.gps = new YesenseImuGpsData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseImuAllData
    // Serialize message field [temperature]
    bufferOffset = _serializer.float32(obj.temperature, buffer, bufferOffset);
    // Serialize message field [sample_timestamp]
    bufferOffset = _serializer.uint32(obj.sample_timestamp, buffer, bufferOffset);
    // Serialize message field [sync_timestamp]
    bufferOffset = _serializer.uint32(obj.sync_timestamp, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = geometry_msgs.msg.Accel.serialize(obj.accel, buffer, bufferOffset);
    // Serialize message field [quaternion]
    bufferOffset = YesenseImuQuaternion.serialize(obj.quaternion, buffer, bufferOffset);
    // Serialize message field [eulerAngle]
    bufferOffset = YesenseImuEulerAngle.serialize(obj.eulerAngle, buffer, bufferOffset);
    // Serialize message field [location]
    bufferOffset = YesenseImuLocation.serialize(obj.location, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = YesenseImuStatus.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [gnss]
    bufferOffset = YesenseImuGnssData.serialize(obj.gnss, buffer, bufferOffset);
    // Serialize message field [gps]
    bufferOffset = YesenseImuGpsData.serialize(obj.gps, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseImuAllData
    let len;
    let data = new YesenseImuAllData(null);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sample_timestamp]
    data.sample_timestamp = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sync_timestamp]
    data.sync_timestamp = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = geometry_msgs.msg.Accel.deserialize(buffer, bufferOffset);
    // Deserialize message field [quaternion]
    data.quaternion = YesenseImuQuaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [eulerAngle]
    data.eulerAngle = YesenseImuEulerAngle.deserialize(buffer, bufferOffset);
    // Deserialize message field [location]
    data.location = YesenseImuLocation.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = YesenseImuStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [gnss]
    data.gnss = YesenseImuGnssData.deserialize(buffer, bufferOffset);
    // Deserialize message field [gps]
    data.gps = YesenseImuGpsData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += YesenseImuGpsData.getMessageSize(object.gps);
    return length + 186;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseImuAllData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a618208a575071271ec15c81f06b0bca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 temperature
    uint32 sample_timestamp
    uint32 sync_timestamp
    geometry_msgs/Accel accel
    yesense_imu/YesenseImuQuaternion quaternion
    yesense_imu/YesenseImuEulerAngle eulerAngle
    yesense_imu/YesenseImuLocation location
    yesense_imu/YesenseImuStatus status
    yesense_imu/YesenseImuGnssData gnss
    yesense_imu/YesenseImuGpsData gps
    ================================================================================
    MSG: geometry_msgs/Accel
    # This expresses acceleration in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: yesense_imu/YesenseImuQuaternion
    float32 q0
    float32 q1
    float32 q2
    float32 q3
    
    ================================================================================
    MSG: yesense_imu/YesenseImuEulerAngle
    float32 roll
    float32 pitch
    float32 yaw
    
    ================================================================================
    MSG: yesense_imu/YesenseImuLocation
    float64 longtidue
    float64 latitude
    float32 altidue
    ================================================================================
    MSG: yesense_imu/YesenseImuStatus
    uint8 fusion_status
    uint8 gnss_status
    ================================================================================
    MSG: yesense_imu/YesenseImuGnssData
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
    MSG: yesense_imu/YesenseImuSlaveGnssData
    float32 dual_ant_yaw
    float32 dual_ant_yaw_error
    float32 dual_ant_baseline_len
    ================================================================================
    MSG: yesense_imu/YesenseImuGpsData
    string[] raw_data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YesenseImuAllData(null);
    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
    }

    if (msg.sample_timestamp !== undefined) {
      resolved.sample_timestamp = msg.sample_timestamp;
    }
    else {
      resolved.sample_timestamp = 0
    }

    if (msg.sync_timestamp !== undefined) {
      resolved.sync_timestamp = msg.sync_timestamp;
    }
    else {
      resolved.sync_timestamp = 0
    }

    if (msg.accel !== undefined) {
      resolved.accel = geometry_msgs.msg.Accel.Resolve(msg.accel)
    }
    else {
      resolved.accel = new geometry_msgs.msg.Accel()
    }

    if (msg.quaternion !== undefined) {
      resolved.quaternion = YesenseImuQuaternion.Resolve(msg.quaternion)
    }
    else {
      resolved.quaternion = new YesenseImuQuaternion()
    }

    if (msg.eulerAngle !== undefined) {
      resolved.eulerAngle = YesenseImuEulerAngle.Resolve(msg.eulerAngle)
    }
    else {
      resolved.eulerAngle = new YesenseImuEulerAngle()
    }

    if (msg.location !== undefined) {
      resolved.location = YesenseImuLocation.Resolve(msg.location)
    }
    else {
      resolved.location = new YesenseImuLocation()
    }

    if (msg.status !== undefined) {
      resolved.status = YesenseImuStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new YesenseImuStatus()
    }

    if (msg.gnss !== undefined) {
      resolved.gnss = YesenseImuGnssData.Resolve(msg.gnss)
    }
    else {
      resolved.gnss = new YesenseImuGnssData()
    }

    if (msg.gps !== undefined) {
      resolved.gps = YesenseImuGpsData.Resolve(msg.gps)
    }
    else {
      resolved.gps = new YesenseImuGpsData()
    }

    return resolved;
    }
};

module.exports = YesenseImuAllData;
