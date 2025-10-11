#ifndef CHASSIS_LQR_HPP_
#define CHASSIS_LQR_HPP_

#include <array>

namespace control
{
namespace chassis::algorithms
{

/* LQR controller for 4-wheel omni chassis.
   Regulates body-frame velocities (vx, vy, w) to ramp/joystick references.
   Produces 4 motor commands (FL, FR, BL, BR) in motor units. */
class ChassisLqrController
{
public:
    struct Gains2 { float k_pos; float k_vel; };

    /* mass: chassis mass (kg)
       inertia: chassis yaw inertia about CoM (kg·m²)
       motorOutputMax: clamp for motor command (C620 DJI currently)
       axisToMotorScale: maps axis effort to per-wheel command (rpmScaleFactor) */
    ChassisLqrController(float mass,
                         float inertia,
                         float motorOutputMax = 8000.0f,
                         float axisToMotorScale = 3500.0f);

    // Optional: inject gains computed offline (CARE).
    void setGains(const Gains2& Kx, const Gains2& Ky, const Gains2& Kt);

    /* Update controller.
       Inputs are CURRENT body velocities and references, normalized to [-1,1] (state-based control).
       Derivatives dvx/dvy/dw can be 0 if not available.
       Returns motor commands: {FL, FR, BL, BR} clamped to motorOutputMax. */
    std::array<float,4> update(float vx, float dvx, float vx_ref,
                               float vy, float dvy, float vy_ref,
                               float w,  float dw,  float w_ref);

private:
    // Simple default gains, will replace once robot has been simulated to get CARE-computed gains.
    static Gains2 defaultGains(float dyn);

private:
    float m_;
    float I_;
    float maxOut_;
    float scale_;

    Gains2 Kx_{};
    Gains2 Ky_{};
    Gains2 Kt_{};
};

}  // namespace chassis::algorithms
}  // namespace control

#endif  // CHASSIS_LQR_HPP_
