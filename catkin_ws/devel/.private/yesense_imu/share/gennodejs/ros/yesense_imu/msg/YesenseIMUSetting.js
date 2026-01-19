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

class YesenseIMUSetting {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.class = null;
      this.id = null;
      this.length = null;
      this.msg = null;
    }
    else {
      if (initObj.hasOwnProperty('class')) {
        this.class = initObj.class
      }
      else {
        this.class = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0;
      }
      if (initObj.hasOwnProperty('msg')) {
        this.msg = initObj.msg
      }
      else {
        this.msg = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YesenseIMUSetting
    // Serialize message field [class]
    bufferOffset = _serializer.uint8(obj.class, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.uint8(obj.length, buffer, bufferOffset);
    // Serialize message field [msg]
    bufferOffset = _arraySerializer.uint8(obj.msg, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YesenseIMUSetting
    let len;
    let data = new YesenseIMUSetting(null);
    // Deserialize message field [class]
    data.class = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [msg]
    data.msg = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.msg.length;
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yesense_imu/YesenseIMUSetting';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e7a91ef26451a647daf3866c66ac74c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #uint8 CLASS_PRODUCTION_INFO   = 0
    #uint8 CLASS_BAUDRATE          = 2
    #uint8 CLASS_OUTPUT_FREEQUENCY = 3
    #uint8 CLASS_OUTPUT_CONTENT    = 4
    #uint8 CLASS_STANDARD_PARAM    = 5
    #uint8 CLASS_MODE              = 77   #0x4D
    #uint8 CLASS_NMEA_OUTPUT       = 78   #0x4E
    
    #uint8 ID_REQUEST              = 0
    #uint8 ID_MEMERY               = 1
    #uint8 ID_FLASH                = 2
    
    uint8 class
    uint8 id
    uint8 length
    uint8[] msg
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YesenseIMUSetting(null);
    if (msg.class !== undefined) {
      resolved.class = msg.class;
    }
    else {
      resolved.class = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0
    }

    if (msg.msg !== undefined) {
      resolved.msg = msg.msg;
    }
    else {
      resolved.msg = []
    }

    return resolved;
    }
};

module.exports = YesenseIMUSetting;
