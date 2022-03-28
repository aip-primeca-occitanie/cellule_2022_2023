// Auto-generated. Do not edit!

// (in-package shuttles.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class msgShuttleChange {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.argument = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = 0;
      }
      if (initObj.hasOwnProperty('argument')) {
        this.argument = initObj.argument
      }
      else {
        this.argument = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgShuttleChange
    // Serialize message field [handle]
    bufferOffset = _serializer.int32(obj.handle, buffer, bufferOffset);
    // Serialize message field [argument]
    bufferOffset = _serializer.int32(obj.argument, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgShuttleChange
    let len;
    let data = new msgShuttleChange(null);
    // Deserialize message field [handle]
    data.handle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [argument]
    data.argument = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'shuttles/msgShuttleChange';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02c8ec5acb6ca55401e7f0fabb5b8e17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # argument = destination or product
    
    int32 handle
    int32 argument 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgShuttleChange(null);
    if (msg.handle !== undefined) {
      resolved.handle = msg.handle;
    }
    else {
      resolved.handle = 0
    }

    if (msg.argument !== undefined) {
      resolved.argument = msg.argument;
    }
    else {
      resolved.argument = 0
    }

    return resolved;
    }
};

module.exports = msgShuttleChange;
