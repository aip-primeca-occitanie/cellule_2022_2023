// Auto-generated. Do not edit!

// (in-package robots.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Msg_numrobot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_robot = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('num_robot')) {
        this.num_robot = initObj.num_robot
      }
      else {
        this.num_robot = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Msg_numrobot
    // Serialize message field [num_robot]
    bufferOffset = _serializer.int32(obj.num_robot, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _serializer.int32(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Msg_numrobot
    let len;
    let data = new Msg_numrobot(null);
    // Deserialize message field [num_robot]
    data.num_robot = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robots/Msg_numrobot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '47287769d49229e4971e3462ae2d0aac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 num_robot
    int32 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Msg_numrobot(null);
    if (msg.num_robot !== undefined) {
      resolved.num_robot = msg.num_robot;
    }
    else {
      resolved.num_robot = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    return resolved;
    }
};

module.exports = Msg_numrobot;
