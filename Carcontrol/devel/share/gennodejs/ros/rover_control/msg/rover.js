// Auto-generated. Do not edit!

// (in-package rover_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class rover {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rover_px = null;
      this.rover_py = null;
      this.rover_vx = null;
      this.rover_vy = null;
      this.rover_w = null;
      this.rover_state = null;
    }
    else {
      if (initObj.hasOwnProperty('rover_px')) {
        this.rover_px = initObj.rover_px
      }
      else {
        this.rover_px = 0.0;
      }
      if (initObj.hasOwnProperty('rover_py')) {
        this.rover_py = initObj.rover_py
      }
      else {
        this.rover_py = 0.0;
      }
      if (initObj.hasOwnProperty('rover_vx')) {
        this.rover_vx = initObj.rover_vx
      }
      else {
        this.rover_vx = 0.0;
      }
      if (initObj.hasOwnProperty('rover_vy')) {
        this.rover_vy = initObj.rover_vy
      }
      else {
        this.rover_vy = 0.0;
      }
      if (initObj.hasOwnProperty('rover_w')) {
        this.rover_w = initObj.rover_w
      }
      else {
        this.rover_w = 0.0;
      }
      if (initObj.hasOwnProperty('rover_state')) {
        this.rover_state = initObj.rover_state
      }
      else {
        this.rover_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rover
    // Serialize message field [rover_px]
    bufferOffset = _serializer.float64(obj.rover_px, buffer, bufferOffset);
    // Serialize message field [rover_py]
    bufferOffset = _serializer.float64(obj.rover_py, buffer, bufferOffset);
    // Serialize message field [rover_vx]
    bufferOffset = _serializer.float64(obj.rover_vx, buffer, bufferOffset);
    // Serialize message field [rover_vy]
    bufferOffset = _serializer.float64(obj.rover_vy, buffer, bufferOffset);
    // Serialize message field [rover_w]
    bufferOffset = _serializer.float64(obj.rover_w, buffer, bufferOffset);
    // Serialize message field [rover_state]
    bufferOffset = _serializer.int32(obj.rover_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rover
    let len;
    let data = new rover(null);
    // Deserialize message field [rover_px]
    data.rover_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rover_py]
    data.rover_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rover_vx]
    data.rover_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rover_vy]
    data.rover_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rover_w]
    data.rover_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rover_state]
    data.rover_state = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rover_control/rover';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8939377560b48465bde83d9b3af7dfb7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 rover_px
    float64 rover_py
    float64 rover_vx
    float64 rover_vy
    float64 rover_w
    int32 rover_state
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rover(null);
    if (msg.rover_px !== undefined) {
      resolved.rover_px = msg.rover_px;
    }
    else {
      resolved.rover_px = 0.0
    }

    if (msg.rover_py !== undefined) {
      resolved.rover_py = msg.rover_py;
    }
    else {
      resolved.rover_py = 0.0
    }

    if (msg.rover_vx !== undefined) {
      resolved.rover_vx = msg.rover_vx;
    }
    else {
      resolved.rover_vx = 0.0
    }

    if (msg.rover_vy !== undefined) {
      resolved.rover_vy = msg.rover_vy;
    }
    else {
      resolved.rover_vy = 0.0
    }

    if (msg.rover_w !== undefined) {
      resolved.rover_w = msg.rover_w;
    }
    else {
      resolved.rover_w = 0.0
    }

    if (msg.rover_state !== undefined) {
      resolved.rover_state = msg.rover_state;
    }
    else {
      resolved.rover_state = 0
    }

    return resolved;
    }
};

module.exports = rover;
