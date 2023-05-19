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

class corner {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wheel_pos_actual = null;
      this.wheel_pos_desired = null;
      this.wheel_vel_actual = null;
      this.wheel_vel_desired = null;
      this.steer_pos_actual = null;
      this.steer_pos_desired = null;
    }
    else {
      if (initObj.hasOwnProperty('wheel_pos_actual')) {
        this.wheel_pos_actual = initObj.wheel_pos_actual
      }
      else {
        this.wheel_pos_actual = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_pos_desired')) {
        this.wheel_pos_desired = initObj.wheel_pos_desired
      }
      else {
        this.wheel_pos_desired = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_vel_actual')) {
        this.wheel_vel_actual = initObj.wheel_vel_actual
      }
      else {
        this.wheel_vel_actual = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_vel_desired')) {
        this.wheel_vel_desired = initObj.wheel_vel_desired
      }
      else {
        this.wheel_vel_desired = 0.0;
      }
      if (initObj.hasOwnProperty('steer_pos_actual')) {
        this.steer_pos_actual = initObj.steer_pos_actual
      }
      else {
        this.steer_pos_actual = 0.0;
      }
      if (initObj.hasOwnProperty('steer_pos_desired')) {
        this.steer_pos_desired = initObj.steer_pos_desired
      }
      else {
        this.steer_pos_desired = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type corner
    // Serialize message field [wheel_pos_actual]
    bufferOffset = _serializer.float64(obj.wheel_pos_actual, buffer, bufferOffset);
    // Serialize message field [wheel_pos_desired]
    bufferOffset = _serializer.float64(obj.wheel_pos_desired, buffer, bufferOffset);
    // Serialize message field [wheel_vel_actual]
    bufferOffset = _serializer.float64(obj.wheel_vel_actual, buffer, bufferOffset);
    // Serialize message field [wheel_vel_desired]
    bufferOffset = _serializer.float64(obj.wheel_vel_desired, buffer, bufferOffset);
    // Serialize message field [steer_pos_actual]
    bufferOffset = _serializer.float64(obj.steer_pos_actual, buffer, bufferOffset);
    // Serialize message field [steer_pos_desired]
    bufferOffset = _serializer.float64(obj.steer_pos_desired, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type corner
    let len;
    let data = new corner(null);
    // Deserialize message field [wheel_pos_actual]
    data.wheel_pos_actual = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wheel_pos_desired]
    data.wheel_pos_desired = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wheel_vel_actual]
    data.wheel_vel_actual = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wheel_vel_desired]
    data.wheel_vel_desired = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steer_pos_actual]
    data.steer_pos_actual = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steer_pos_desired]
    data.steer_pos_desired = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rover_control/corner';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '516e2e5c8624f5f3853fb88a0ab5fb10';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 wheel_pos_actual
    float64 wheel_pos_desired
    float64 wheel_vel_actual
    float64 wheel_vel_desired
    float64 steer_pos_actual
    float64 steer_pos_desired
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new corner(null);
    if (msg.wheel_pos_actual !== undefined) {
      resolved.wheel_pos_actual = msg.wheel_pos_actual;
    }
    else {
      resolved.wheel_pos_actual = 0.0
    }

    if (msg.wheel_pos_desired !== undefined) {
      resolved.wheel_pos_desired = msg.wheel_pos_desired;
    }
    else {
      resolved.wheel_pos_desired = 0.0
    }

    if (msg.wheel_vel_actual !== undefined) {
      resolved.wheel_vel_actual = msg.wheel_vel_actual;
    }
    else {
      resolved.wheel_vel_actual = 0.0
    }

    if (msg.wheel_vel_desired !== undefined) {
      resolved.wheel_vel_desired = msg.wheel_vel_desired;
    }
    else {
      resolved.wheel_vel_desired = 0.0
    }

    if (msg.steer_pos_actual !== undefined) {
      resolved.steer_pos_actual = msg.steer_pos_actual;
    }
    else {
      resolved.steer_pos_actual = 0.0
    }

    if (msg.steer_pos_desired !== undefined) {
      resolved.steer_pos_desired = msg.steer_pos_desired;
    }
    else {
      resolved.steer_pos_desired = 0.0
    }

    return resolved;
    }
};

module.exports = corner;
