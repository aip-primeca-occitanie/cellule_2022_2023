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

class FaireTacheMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_robot = null;
      this.position = null;
      this.duree = null;
    }
    else {
      if (initObj.hasOwnProperty('num_robot')) {
        this.num_robot = initObj.num_robot
      }
      else {
        this.num_robot = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
      if (initObj.hasOwnProperty('duree')) {
        this.duree = initObj.duree
      }
      else {
        this.duree = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaireTacheMsg
    // Serialize message field [num_robot]
    bufferOffset = _serializer.int32(obj.num_robot, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.int32(obj.position, buffer, bufferOffset);
    // Serialize message field [duree]
    bufferOffset = _serializer.int32(obj.duree, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaireTacheMsg
    let len;
    let data = new FaireTacheMsg(null);
    // Deserialize message field [num_robot]
    data.num_robot = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [duree]
    data.duree = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robots/FaireTacheMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d9a6fbf544e79eb45c59d3957770241';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 num_robot
    int32 position
    int32 duree
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaireTacheMsg(null);
    if (msg.num_robot !== undefined) {
      resolved.num_robot = msg.num_robot;
    }
    else {
      resolved.num_robot = 0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    if (msg.duree !== undefined) {
      resolved.duree = msg.duree;
    }
    else {
      resolved.duree = 0
    }

    return resolved;
    }
};

module.exports = FaireTacheMsg;
