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

class Msg_PinControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.PINON = null;
      this.PINOFF = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('PINON')) {
        this.PINON = initObj.PINON
      }
      else {
        this.PINON = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('PINOFF')) {
        this.PINOFF = initObj.PINOFF
      }
      else {
        this.PINOFF = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Msg_PinControl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [PINON] has the right length
    if (obj.PINON.length !== 9) {
      throw new Error('Unable to serialize array field PINON - length must be 9')
    }
    // Serialize message field [PINON]
    bufferOffset = _arraySerializer.bool(obj.PINON, buffer, bufferOffset, 9);
    // Check that the constant length array field [PINOFF] has the right length
    if (obj.PINOFF.length !== 9) {
      throw new Error('Unable to serialize array field PINOFF - length must be 9')
    }
    // Serialize message field [PINOFF]
    bufferOffset = _arraySerializer.bool(obj.PINOFF, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Msg_PinControl
    let len;
    let data = new Msg_PinControl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [PINON]
    data.PINON = _arrayDeserializer.bool(buffer, bufferOffset, 9)
    // Deserialize message field [PINOFF]
    data.PINOFF = _arrayDeserializer.bool(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commande_locale/Msg_PinControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bdef2ee2c42b42b5f1efd9be2b5cd147';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    bool[9] PINON   
    bool[9] PINOFF
    
    
    
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
    const resolved = new Msg_PinControl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.PINON !== undefined) {
      resolved.PINON = msg.PINON;
    }
    else {
      resolved.PINON = new Array(9).fill(0)
    }

    if (msg.PINOFF !== undefined) {
      resolved.PINOFF = msg.PINOFF;
    }
    else {
      resolved.PINOFF = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Msg_PinControl;
