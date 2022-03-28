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

class Msg_SwitchControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.LOCK = null;
      this.RD = null;
      this.RG = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('LOCK')) {
        this.LOCK = initObj.LOCK
      }
      else {
        this.LOCK = new Array(13).fill(0);
      }
      if (initObj.hasOwnProperty('RD')) {
        this.RD = initObj.RD
      }
      else {
        this.RD = new Array(13).fill(0);
      }
      if (initObj.hasOwnProperty('RG')) {
        this.RG = initObj.RG
      }
      else {
        this.RG = new Array(13).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Msg_SwitchControl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [LOCK] has the right length
    if (obj.LOCK.length !== 13) {
      throw new Error('Unable to serialize array field LOCK - length must be 13')
    }
    // Serialize message field [LOCK]
    bufferOffset = _arraySerializer.bool(obj.LOCK, buffer, bufferOffset, 13);
    // Check that the constant length array field [RD] has the right length
    if (obj.RD.length !== 13) {
      throw new Error('Unable to serialize array field RD - length must be 13')
    }
    // Serialize message field [RD]
    bufferOffset = _arraySerializer.bool(obj.RD, buffer, bufferOffset, 13);
    // Check that the constant length array field [RG] has the right length
    if (obj.RG.length !== 13) {
      throw new Error('Unable to serialize array field RG - length must be 13')
    }
    // Serialize message field [RG]
    bufferOffset = _arraySerializer.bool(obj.RG, buffer, bufferOffset, 13);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Msg_SwitchControl
    let len;
    let data = new Msg_SwitchControl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [LOCK]
    data.LOCK = _arrayDeserializer.bool(buffer, bufferOffset, 13)
    // Deserialize message field [RD]
    data.RD = _arrayDeserializer.bool(buffer, bufferOffset, 13)
    // Deserialize message field [RG]
    data.RG = _arrayDeserializer.bool(buffer, bufferOffset, 13)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 39;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commande_locale/Msg_SwitchControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0797883d1e8c7216af064c8c90c37c74';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    bool[13] LOCK
    bool[13] RD
    bool[13] RG
    
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
    const resolved = new Msg_SwitchControl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.LOCK !== undefined) {
      resolved.LOCK = msg.LOCK;
    }
    else {
      resolved.LOCK = new Array(13).fill(0)
    }

    if (msg.RD !== undefined) {
      resolved.RD = msg.RD;
    }
    else {
      resolved.RD = new Array(13).fill(0)
    }

    if (msg.RG !== undefined) {
      resolved.RG = msg.RG;
    }
    else {
      resolved.RG = new Array(13).fill(0)
    }

    return resolved;
    }
};

module.exports = Msg_SwitchControl;
