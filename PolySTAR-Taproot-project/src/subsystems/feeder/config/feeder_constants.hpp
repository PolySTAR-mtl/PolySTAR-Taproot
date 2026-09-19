#ifndef FEEDER_CONSTANTS_HPP_
#define FEEDER_CONSTANTS_HPP_

#include "algorithms/feed_forward.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/dji_motor_encoder.hpp"
/**
 * Feeder position PID: A PID controller for feeder position. The PID parameters for the
 * controller are listed below.
 */
static constexpr tap::algorithms::SmoothPidConfig FEEDER_PID_CONFIG(
    0.075f, // kP
    0.f, // kI
    -7.5f, // kD
    5000.f, // Max error sum
    16000.f, // Max output
    1.f, // TQ Derivative Kalman
    0.f, // TR Derivative Kalman
    1.f, // TQ Proportional Kalman
    0.f, // TR Proportional Kalman
    0.f, // Error Deadzone
    0.f // Error derivative floor
);

/**
 * Turret Position FeedForward: Feed Forward controllers for feeder position. The FF parameters for the
 * controller are listed below.
 */
static constexpr src::algorithms::FeedForwardConfig FEEDER_FF_CONFIG(
    400.f, // kS
    0.f, // kV
    0.f, // kG
    1000.f // maxVelocity
);

/**
 * Unit conversion constants
 */
static constexpr float DEGREE_TO_TICK = tap::motor::DjiMotorEncoder::ENC_RESOLUTION * 36.f / 360.f; // 8192 Ticks per turn, 36:1 gear ratio

/**
 * Feeder PID constants
 */
static constexpr float FEEDER_PID_KP = 20.f;
static constexpr float FEEDER_PID_KI = 5.f;
static constexpr float FEEDER_PID_KD = 0.f;
static constexpr float FEEDER_PID_MAX_ERROR_SUM = 5000.f;
static constexpr float FEEDER_PID_MAX_OUTPUT = 8000.f;

/*
 * Hardware constants, not specific to any particular feeder.
 */
static constexpr tap::motor::MotorId FEEDER_MOTOR_ID = tap::motor::MOTOR8;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

#endif