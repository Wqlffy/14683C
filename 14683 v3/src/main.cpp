#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include <algorithm>
#include <cmath>

constexpr int FL_PORTS[2] = { -1,  2 };
constexpr int FR_PORTS[2] = {  3, -4 }; 
constexpr int BL_PORTS[2] = { -5,  6 }; 
constexpr int BR_PORTS[2] = {  7, -8 }; 

//add ports here
pros::Imu imu(10);
pros::Rotation horizontalEnc(20);
pros::Rotation verticalEnc(-11);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75); //distance from center
lemlib::TrackingWheel vertical(&verticalEnc,   lemlib::Omniwheel::NEW_275, -2.5);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup mFL({ FL_PORTS[0], FL_PORTS[1] }, pros::MotorGearset::green);
pros::MotorGroup mFR({ FR_PORTS[0], FR_PORTS[1] }, pros::MotorGearset::green);
pros::MotorGroup mBL({ BL_PORTS[0], BL_PORTS[1] }, pros::MotorGearset::green);
pros::MotorGroup mBR({ BR_PORTS[0], BR_PORTS[1] }, pros::MotorGearset::green);

pros::MotorGroup leftDrive ({ FL_PORTS[0], FL_PORTS[1], BL_PORTS[0], BL_PORTS[1] }, pros::MotorGearset::green);
pros::MotorGroup rightDrive({ FR_PORTS[0], FR_PORTS[1], BR_PORTS[0], BR_PORTS[1] }, pros::MotorGearset::green);

// Dummy drivetrain shell (values don’t affect our holonomic mixer)
lemlib::Drivetrain drivetrain(
  &leftDrive, &rightDrive,
  10, 
  lemlib::Omniwheel::NEW_325,  
  333,
  2
);

// Controllers (start here; tune later)
lemlib::ControllerSettings linearController(
  10, 
  0, 
  3, 
  3,
  1,
  100,
  3,
  500,
  20
);
lemlib::ControllerSettings angularController(
  2, 
  0, 
  10, 
  3,
  1, 
  100,
  3, 
  500,
  0
);

lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve strafeCurve(3, 10, 1.019);

// LemLib chassis (for pose/telemetry only)
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

static inline void setHolonomic(double vx, double vy, double w) {
  // Mix: vy=fwd, vx=strafe, w=turn
  double fl =  vy + vx + w;
  double fr =  vy - vx - w;
  double bl =  vy - vx + w;
  double br =  vy + vx - w;

  double maxMag = std::max({ std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br), 1.0 });
  fl /= maxMag; fr /= maxMag; bl /= maxMag; br /= maxMag;

  const double MAX_MV = 12000.0;
  mFL.move_voltage(static_cast<int16_t>(fl * MAX_MV));
  mFR.move_voltage(static_cast<int16_t>(fr * MAX_MV));
  mBL.move_voltage(static_cast<int16_t>(bl * MAX_MV));
  mBR.move_voltage(static_cast<int16_t>(br * MAX_MV));
}

static inline double shapeAxis(int raw, int deadband, int minOut127, double expoGain) {
    if (std::abs(raw) <= deadband) return 0.0;

    double s = raw / 127.0;
    double sign = (s >= 0) ? 1.0 : -1.0;
    s = std::abs(s);

    double e = std::pow(s, expoGain) * sign;

    double floor = std::clamp(minOut127 / 127.0, 0.0, 0.99);
    if (e == 0.0) return 0.0;
    return floor * ((e > 0) ? 1.0 : -1.0) + (1.0 - floor) * e;
}


struct Slew {
  double rate; 
  double out = 0;
  double step(double target) {
    double d = target - out;
    if (std::abs(d) > rate) out += (d > 0 ? rate : -rate);
    else out = target;
    return out;
  }
};
Slew slewVX{0.06}, slewVY{0.06}, slewW{0.06};

static inline double degToRad(double d){ return d * M_PI / 180.0; }
static inline double wrapDeg(double e){ while (e>180) e-=360; while (e<-180) e+=360; return e; }
static inline void fieldToRobot(double& x, double& y, double headingDeg) {
  double h = -degToRad(headingDeg);
  double cs = std::cos(h), sn = std::sin(h);
  double xr = x * cs - y * sn;
  double yr = x * sn + y * cs;
  x = xr; y = yr;
}

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();

  pros::Task screenTask([&](){
    while (true) {
      auto p = chassis.getPose();
      pros::lcd::print(0, "X: %.2f in", p.x);
      pros::lcd::print(1, "Y: %.2f in", p.y);
      pros::lcd::print(2, "Th: %.2f deg", p.theta);
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      pros::delay(50);
    }
  });
}

void disabled() {}
void competition_initialize() {}

void holonomicMoveToPose(double xT, double yT, double thT, int timeoutMs,
                         double kx=0.10, double ky=0.10, double kw=0.015,
                         double posTol=0.75, double angTol=2.0) {
  const int dt = 10;
  int t = 0;
  while (t < timeoutMs) {
    auto p = chassis.getPose();
    double ex = xT - p.x, ey = yT - p.y, eTh = wrapDeg(thT - p.theta);
    double vx = ex, vy = ey;
    fieldToRobot(vx, vy, p.theta);

    double cmd_vx = std::clamp(kx * vx, -1.0, 1.0);
    double cmd_vy = std::clamp(ky * vy, -1.0, 1.0);
    double cmd_w  = std::clamp(kw * eTh, -1.0, 1.0);

    setHolonomic(slewVX.step(cmd_vx), slewVY.step(cmd_vy), slewW.step(cmd_w));

    if (std::hypot(ex, ey) < posTol && std::abs(eTh) < angTol) break;

    pros::delay(dt);
    t += dt;
  }
  setHolonomic(0,0,0);
}

void autonomous() {
  holonomicMoveToPose(20, 15,  90, 4000);
  holonomicMoveToPose( 0,  0, 270, 4000);
  // Example: rotate-in-place to 90 at current XY
  auto p = chassis.getPose();
  holonomicMoveToPose(p.x, p.y, 90, 1200);
}

void opcontrol() {
  for (auto* g : { &mFL, &mFR, &mBL, &mBR })
    g->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
	double vy = shapeAxis(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),  3, 10, 1.019);
	double vx = shapeAxis(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),  3, 10, 1.019);
	double w  = shapeAxis(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3, 10, 1.019);

    auto pose = chassis.getPose();
    double hx = vx, hy = vy;
    fieldToRobot(hx, hy, pose.theta);

    setHolonomic(slewVX.step(hx), slewVY.step(hy), slewW.step(w));
    pros::delay(10);
  }
}
