// Auto-generated. Do not edit!

// (in-package commande_locale.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TacheFinieMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_poste = null;
      this.num_produit = null;
      this.duree = null;
    }
    else {
      if (initObj.hasOwnProperty('num_poste')) {
        this.num_poste = initObj.num_poste
      }
      else {
        this.num_poste = 0;
      }
      if (initObj.hasOwnProperty('num_produit')) {
        this.num_produit = initObj.num_produit
      }
      else {
        this.num_produit = 0;
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
    // Serializes a message object of type TacheFinieMsg
    // Serialize message field [num_poste]
    bufferOffset = _serializer.int32(obj.num_poste, buffer, bufferOffset);
    // Serialize message field [num_produit]
    bufferOffset = _serializer.int32(obj.num_produit, buffer, bufferOffset);
    // Serialize message field [duree]
    bufferOffset = _serializer.int32(obj.duree, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TacheFinieMsg
    let len;
    let data = new TacheFinieMsg(null);
    // Deserialize message field [num_poste]
    data.num_poste = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_produit]
    data.num_produit = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [duree]
    data.duree = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commande_locale/TacheFinieMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4cf6cdaae99079fd26f29aa9dd559651';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 num_poste
    int32 num_produit
    int32 duree
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TacheFinieMsg(null);
    if (msg.num_poste !== undefined) {
      resolved.num_poste = msg.num_poste;
    }
    else {
      resolved.num_poste = 0
    }

    if (msg.num_produit !== undefined) {
      resolved.num_produit = msg.num_produit;
    }
    else {
      resolved.num_produit = 0
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

module.exports = TacheFinieMsg;
