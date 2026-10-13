---
applyTo: "libraries/AC_AttitudeControl/**,libraries/AC_PID/**,libraries/AC_WPNav/**,libraries/AC_Avoidance/**,libraries/APM_Control/**,libraries/AP_TECS/**,libraries/AP_L1_Control/**,libraries/AR_WPNav/**,libraries/AR_Motors/**,libraries/AP_Motors/**,ArduCopter/mode*.cpp,ArduPlane/mode*.cpp,Rover/mode*.cpp,ArduSub/mode*.cpp"
---

# Control and navigation rules

- Check units on every changed line. Older interfaces use centidegrees and
  centimetres (`_cd`, `_cm` suffixes) while newer ones use SI units; a
  conversion missed at a boundary is a factor of 100 error.
- Check dependence on loop rate: filters, rate limits and integrators must use
  the real `dt`, and behave sensibly when the loop rate changes or a loop is
  missed.
- Check limits and saturation: integrator anti-windup, output limits, and what
  happens when the vehicle cannot follow the demand.
- Mode changes: behaviour on entry and exit, initialisation from the current
  vehicle state, and transitions in both directions.
- Failsafes and edge cases: loss of position estimate, low airspeed, very
  large or zero inputs, and the vehicle on the ground or disarmed.
- Shared libraries serve several vehicles. Check every vehicle that uses the
  changed code, not only the one the PR targets.
- Changes to flight behaviour need an autotest exercising them, or SITL or
  flight logs showing the effect.
