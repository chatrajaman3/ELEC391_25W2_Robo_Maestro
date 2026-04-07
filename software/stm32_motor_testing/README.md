# Testing program for the motor and encoder:


## Wiring:

AS5600 Pinout SOIC-8 Pin-Out
─────────────────────────────────────────
        ┌───────────┐
 VDD5V ─┤ 1       8 ├── DIR
 VDD3V3─┤ 2       7 ├── SCL
  OUT  ─┤ 3       6 ├── SDA
  GND  ─┤ 4       5 ├── PGO
        └───────────┘

Pin Descriptions
─────────────────────────────────────────
 1  VDD5V  5V supply    ─┐ tie together,
 2  VDD3V3 3.3V supply  ─┘ connect to 3.3V
 3  OUT    Analog/PWM output (unused if using I2C)
 4  GND    Ground
 5  PGO    OTP programming (leave unconnected)
 6  SDA    I2C data
 7  SCL    I2C clock
 8  DIR    Direction select (GND = CW↑, 3.3V = CCW↑)

### pwm signal:
D7 (PA8) -> IN1 (Driver) 
D8 (PA9) -> IN2 (Driver)

### motor driver:
VM -> 24V powersupply
GND -> ground powersupply
MOTOR1 -> CW motor signal
MOTO2 -> CCW motor signal