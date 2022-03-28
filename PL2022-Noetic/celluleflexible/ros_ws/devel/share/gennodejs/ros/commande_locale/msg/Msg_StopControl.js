// Auto-generated. Do not edit!

// (in-package commande_locale.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Msg_StopControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.STOP = null;
      this.GO = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('STOP')) {
        this.STOP = initObj.STOP
      }
      else {
        this.STOP = new Array(25).fill(0);
      }
      if (initObj.hasOwnProperty('GO')) {
        this.GO = initObj.GO
      }
      else {
        this.GO = new Array(25).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Msg_StopControl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [STOP] has the right length
    if (obj.STOP.length !== 25) {
      throw new Error('Unable to serialize array field STOP - length must be 25')
    }
    // Serialize message field [STOP]
    bufferOffset = _arraySerializer.bool(obj.STOP, buffer, bufferOffset, 25);
    // Check that the constant length array field [GO] has the right length
    if (obj.GO.length !== 25) {
      throw new Error('Unable to serialize array field GO - length must be 25')
    }
    // Serialize message field [GO]
    bufferOffset = _arraySerializer.bool(obj.GO, buffer, bufferOffset, 25);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Msg_StopControl
    let len;
    let data = new Msg_StopControl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [STOP]
    data.STOP = _arrayDeserializer.bool(buffer, bufferOffset, 25)
    // Deserialize message field [GO]
    data.GO = _arrayDeserializer.bool(buffer, bufferOffset, 25)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 50;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commande_locale/Msg_StopControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3351a875ff673c8814a96d783877c306';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    bool[25] STOP
    bool[25] GO
    
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Msg_StopControl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.STOP !== undefined) {
      resolved.STOP = msg.STOP;
    }
    else {
      resolved.STOP = new Array(25).fill(0)
    }

    if (msg.GO !== undefined) {
      resolved.GO = msg.GO;
    }
    else {
      resolved.GO = new Array(25).fill(0)
    }

    return resolved;
    }
};

module.exports = Msg_StopControl;
