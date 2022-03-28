// Auto-generated. Do not edit!

// (in-package schneider.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Msg_SensorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.CPI = null;
      this.CP = null;
      this.PS = null;
      this.DG = null;
      this.DD = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('CPI')) {
        this.CPI = initObj.CPI
      }
      else {
        this.CPI = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('CP')) {
        this.CP = initObj.CP
      }
      else {
        this.CP = new Array(11).fill(0);
      }
      if (initObj.hasOwnProperty('PS')) {
        this.PS = initObj.PS
      }
      else {
        this.PS = new Array(25).fill(0);
      }
      if (initObj.hasOwnProperty('DG')) {
        this.DG = initObj.DG
      }
      else {
        this.DG = new Array(13).fill(0);
      }
      if (initObj.hasOwnProperty('DD')) {
        this.DD = initObj.DD
      }
      else {
        this.DD = new Array(13).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Msg_SensorState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Check that the constant length array field [CPI] has the right length
    if (obj.CPI.length !== 9) {
      throw new Error('Unable to serialize array field CPI - length must be 9')
    }
    // Serialize message field [CPI]
    bufferOffset = _arraySerializer.bool(obj.CPI, buffer, bufferOffset, 9);
    // Check that the constant length array field [CP] has the right length
    if (obj.CP.length !== 11) {
      throw new Error('Unable to serialize array field CP - length must be 11')
    }
    // Serialize message field [CP]
    bufferOffset = _arraySerializer.bool(obj.CP, buffer, bufferOffset, 11);
    // Check that the constant length array field [PS] has the right length
    if (obj.PS.length !== 25) {
      throw new Error('Unable to serialize array field PS - length must be 25')
    }
    // Serialize message field [PS]
    bufferOffset = _arraySerializer.bool(obj.PS, buffer, bufferOffset, 25);
    // Check that the constant length array field [DG] has the right length
    if (obj.DG.length !== 13) {
      throw new Error('Unable to serialize array field DG - length must be 13')
    }
    // Serialize message field [DG]
    bufferOffset = _arraySerializer.bool(obj.DG, buffer, bufferOffset, 13);
    // Check that the constant length array field [DD] has the right length
    if (obj.DD.length !== 13) {
      throw new Error('Unable to serialize array field DD - length must be 13')
    }
    // Serialize message field [DD]
    bufferOffset = _arraySerializer.bool(obj.DD, buffer, bufferOffset, 13);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Msg_SensorState
    let len;
    let data = new Msg_SensorState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [CPI]
    data.CPI = _arrayDeserializer.bool(buffer, bufferOffset, 9)
    // Deserialize message field [CP]
    data.CP = _arrayDeserializer.bool(buffer, bufferOffset, 11)
    // Deserialize message field [PS]
    data.PS = _arrayDeserializer.bool(buffer, bufferOffset, 25)
    // Deserialize message field [DG]
    data.DG = _arrayDeserializer.bool(buffer, bufferOffset, 13)
    // Deserialize message field [DD]
    data.DD = _arrayDeserializer.bool(buffer, bufferOffset, 13)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 75;
  }

  static datatype() {
    // Returns string type for a message object
    return 'schneider/Msg_SensorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd7658e6685c951b5f6486f4677d2be9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    int32 id
    bool[9] CPI
    bool[11] CP
    bool[25] PS
    bool[13] DG
    bool[13] DD
    
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
    const resolved = new Msg_SensorState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.CPI !== undefined) {
      resolved.CPI = msg.CPI;
    }
    else {
      resolved.CPI = new Array(9).fill(0)
    }

    if (msg.CP !== undefined) {
      resolved.CP = msg.CP;
    }
    else {
      resolved.CP = new Array(11).fill(0)
    }

    if (msg.PS !== undefined) {
      resolved.PS = msg.PS;
    }
    else {
      resolved.PS = new Array(25).fill(0)
    }

    if (msg.DG !== undefined) {
      resolved.DG = msg.DG;
    }
    else {
      resolved.DG = new Array(13).fill(0)
    }

    if (msg.DD !== undefined) {
      resolved.DD = msg.DD;
    }
    else {
      resolved.DD = new Array(13).fill(0)
    }

    return resolved;
    }
};

module.exports = Msg_SensorState;
