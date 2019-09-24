// Auto-generated. Do not edit!

// (in-package three_omuni.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class button {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.move_angle = null;
      this.move_speed = null;
      this.turn_right = null;
      this.turn_left = null;
      this.arm_data = null;
      this.calibration = null;
    }
    else {
      if (initObj.hasOwnProperty('move_angle')) {
        this.move_angle = initObj.move_angle
      }
      else {
        this.move_angle = 0.0;
      }
      if (initObj.hasOwnProperty('move_speed')) {
        this.move_speed = initObj.move_speed
      }
      else {
        this.move_speed = 0.0;
      }
      if (initObj.hasOwnProperty('turn_right')) {
        this.turn_right = initObj.turn_right
      }
      else {
        this.turn_right = false;
      }
      if (initObj.hasOwnProperty('turn_left')) {
        this.turn_left = initObj.turn_left
      }
      else {
        this.turn_left = false;
      }
      if (initObj.hasOwnProperty('arm_data')) {
        this.arm_data = initObj.arm_data
      }
      else {
        this.arm_data = false;
      }
      if (initObj.hasOwnProperty('calibration')) {
        this.calibration = initObj.calibration
      }
      else {
        this.calibration = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type button
    // Serialize message field [move_angle]
    bufferOffset = _serializer.float64(obj.move_angle, buffer, bufferOffset);
    // Serialize message field [move_speed]
    bufferOffset = _serializer.float64(obj.move_speed, buffer, bufferOffset);
    // Serialize message field [turn_right]
    bufferOffset = _serializer.bool(obj.turn_right, buffer, bufferOffset);
    // Serialize message field [turn_left]
    bufferOffset = _serializer.bool(obj.turn_left, buffer, bufferOffset);
    // Serialize message field [arm_data]
    bufferOffset = _serializer.bool(obj.arm_data, buffer, bufferOffset);
    // Serialize message field [calibration]
    bufferOffset = _serializer.bool(obj.calibration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type button
    let len;
    let data = new button(null);
    // Deserialize message field [move_angle]
    data.move_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [move_speed]
    data.move_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [turn_right]
    data.turn_right = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [turn_left]
    data.turn_left = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [arm_data]
    data.arm_data = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [calibration]
    data.calibration = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'three_omuni/button';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c703a9d0e274f7444bd1093549d0f0d5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 move_angle
    float64 move_speed
    bool turn_right
    bool turn_left
    bool arm_data
    bool calibration
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new button(null);
    if (msg.move_angle !== undefined) {
      resolved.move_angle = msg.move_angle;
    }
    else {
      resolved.move_angle = 0.0
    }

    if (msg.move_speed !== undefined) {
      resolved.move_speed = msg.move_speed;
    }
    else {
      resolved.move_speed = 0.0
    }

    if (msg.turn_right !== undefined) {
      resolved.turn_right = msg.turn_right;
    }
    else {
      resolved.turn_right = false
    }

    if (msg.turn_left !== undefined) {
      resolved.turn_left = msg.turn_left;
    }
    else {
      resolved.turn_left = false
    }

    if (msg.arm_data !== undefined) {
      resolved.arm_data = msg.arm_data;
    }
    else {
      resolved.arm_data = false
    }

    if (msg.calibration !== undefined) {
      resolved.calibration = msg.calibration;
    }
    else {
      resolved.calibration = false
    }

    return resolved;
    }
};

module.exports = button;
