# 14683C – Driver Control Module (PROS)
**Kang Chiao VEX Robotics Team 14683C**

This module implements the **tele-op driver control loop** for our 2025–26 VEX robot using **PROS**.  
It handles drivetrain motion shaping, auxiliary (linkage/arm) mirroring, and intake actuation with safe braking behavior.

---

## 🚗 Features at a Glance

- **Arcade Drive (Left Stick)**  
  - `Axis3 (LEFT_Y)` → Forward/Backward  
  - `Axis4 (LEFT_X)` → Turn Left/Right  
  - Includes **deadband** and **square-with-sign** shaping for smoother low-speed control  
  - Outputs are clamped to **±100%** and converted to motor voltage  

- **Forward Preference Deadband**  
  - Ignores small forward inputs below **5%**  
  - Makes robot decide clearly between forward driving or point-turning  

- **Drive / Turn Arbitration**
  - When **forward ≠ 0** → both drive sides follow forward percentage (straight drive)  
  - When **forward = 0 & turn < 0** → spin left  
  - When **forward = 0 & turn > 0** → spin right  

- **Auxiliary Mirroring (PORT1 & PORT10)**
  - Mirrors the forward stick input:  
    - `fwd > 0` → both aux motors spin forward  
    - `fwd < 0` → both reverse  
    - `fwd = 0` → both hold position  

- **Intake Control (R1 / R2)**
  - `R2` → intake forward @ 100%  
  - `R1` → intake reverse @ 100%  
  - Neither → hold  

- **Brake Modes**
  - Drive groups (`leftDrive`, `rightDrive`) use **brake** when idle  
  - Aux and intake motors use **hold** when idle  

---

## ⚙️ Control Mapping

| Action                      | Control                                 |
|-----------------------------|------------------------------------------|
| Forward / Backward          | Left stick **Y** (`ANALOG_LEFT_Y`)       |
| Turn Left / Right           | Left stick **X** (`ANALOG_LEFT_X`)       |
| Intake Forward (intake in)  | **R2** (hold)                            |
| Intake Reverse (outtake)    | **R1** (hold)                            |

---

## 🧠 Motion Shaping Details

1. **Deadband Filtering**  
   Removes small joystick noise around the center position.  

2. **Square-With-Sign**  
   Preserves sign but squares magnitude → gentler response near zero while maintaining full output at extremes.  

3. **Forward Preference**  
   If `|forwardPct| < 5%`, it’s treated as zero. This prevents drift and makes point-turns cleaner.  

4. **Voltage-Based Control**  
   Converts percent output to voltage (`pctToMillivolts`) for consistent response under varying battery levels.  

5. **Brake Behavior**  
   - Drivetrain → **brake** when idle  
   - Aux motors → **hold**  
   - Intake → **hold**  

---

## 🧩 Code Architecture

### Main Loop: `control()`
The core driver control task:
- Reads joystick inputs  
- Applies shaping and deadband  
- Decides between forward or turn drive  
- Sends voltage commands to drive, aux, and intake motors  

### Other Functions
- `disabled()` → Runs when robot is disabled (currently empty)  
- `competition_initialize()` → Setup before match (placeholder)  
- `autonomous()` → For autonomous routines (empty for now)

### ⚙️ Tuning Parameters
Parameter	Description	Default
kLoopDelayMs	Control loop delay (ms)	10
kForwardPreferenceDeadband	Forward-input threshold (%)	5.0

Adjust the deadband for a sharper or more forgiving joystick feel.
Shorter loop delay increases responsiveness but uses more CPU.

### 🧭 Future Extensions

-Add PID or autonomous path-following in autonomous()

-Implement slew-rate limiting for smoother acceleration

-Scale turning speed dynamically based on forward velocity

-Integrate safety features (e.g., current limits, temperature checks)
