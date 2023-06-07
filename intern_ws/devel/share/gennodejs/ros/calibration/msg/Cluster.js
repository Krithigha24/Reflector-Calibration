// Auto-generated. Do not edit!

// (in-package calibration.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Cluster {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.label = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = [];
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cluster
    // Serialize message field [index]
    bufferOffset = _arraySerializer.int32(obj.index, buffer, bufferOffset, null);
    // Serialize message field [label]
    bufferOffset = _arraySerializer.int32(obj.label, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cluster
    let len;
    let data = new Cluster(null);
    // Deserialize message field [index]
    data.index = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [label]
    data.label = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.index.length;
    length += 4 * object.label.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'calibration/Cluster';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b0088670d95be7298b39001111310db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #float32 angle_min
    #float32 angle_increment
    int32[] index
    int32[] label
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Cluster(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = []
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = []
    }

    return resolved;
    }
};

module.exports = Cluster;
