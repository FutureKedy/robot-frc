package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  /*
  ===========================================================
   LOGITECH F310 CONTROLLER BUTTON MAPPING (DIRECT INPUT MODE)
   -----------------------------------------------------------
   1  → A (Bottom)
   2  → B (Right)
   3  → X (Left / Square)
   4  → Y (Top / Triangle)
   5  → LB (Left Bumper)
   6  → RB (Right Bumper)
   7  → Back / Select
   8  → Start
   9  → Left Stick Press (L3)
   10 → Right Stick Press (R3)

   AXES:
   0  → Left Stick X
   1  → Left Stick Y
   2  → Right Stick X
   3  → Right Stick Y
   ===========================================================
  */

  // ===== Motor Definitions =====
  private static final SparkMax frontLeftMotor = new SparkMax(12, MotorType.kBrushed);
  private static final SparkMax backLeftMotor = new SparkMax(11, MotorType.kBrushed);
  private static final SparkMax frontRightMotor = new SparkMax(13, MotorType.kBrushed);
  private static final SparkMax backRightMotor = new SparkMax(16, MotorType.kBrushed);
  private static final SparkMax upMotor = new SparkMax(15, MotorType.kBrushed);

  // ===== Controller and Timer =====
  private static final Joystick driverJoystick = new Joystick(0);
  private static final Timer timer = new Timer();

  // ===== Configurable Constants =====
  private static final double LIFT_SPEED = 0.4;
  private static final double AUTO_DRIVE_SPEED = 0.4;
  private static final double AUTO_LIFT_SPEED = 0.4;
  private static final double DEADZONE = 0.05;
  private static final double SHORT_LIFT_DURATION = 0.65;
  private static final double NORMAL_SENSITIVITY = 0.5;
  private static final double TURBO_SENSITIVITY = 0.8;

  // ===== Gear System =====
  private static final int GEAR_SLOW = 0;
  private static final int GEAR_NORMAL = 1;
  private static final int GEAR_FAST = 2;
  private int currentGear = GEAR_NORMAL;

  private static final double[] GEAR_MAX_SPEED = { 0.1, 0.5, 1.0 };
  private static final double[] GEAR_SENSITIVITY = { 0.4, 0.6, 0.8 };
  private static final double GEAR_MIN_SPEED = 0.1;

  // ===== Button Assignments =====
  private static final int BUTTON_A = 1;
  private static final int BUTTON_B = 2;
  private static final int BUTTON_X = 3; // Square
  private static final int BUTTON_Y = 4;
  private static final int BUTTON_LB = 5;
  private static final int BUTTON_RB = 6;
  private static final int BUTTON_BACK = 7;
  private static final int BUTTON_START = 8;
  private static final int BUTTON_L3 = 9;
  private static final int BUTTON_R3 = 10;

  // ===== Custom Control Bindings =====
  private static final int BUTTON_GEAR_UP = BUTTON_X; // Square
  private static final int BUTTON_GEAR_DOWN = BUTTON_A; // X (A button)
  private static final int BUTTON_SHORT_LIFT = BUTTON_Y;
  private static final int BUTTON_MANUAL_LIFT = BUTTON_B;
  private static final int BUTTON_TURBO = BUTTON_RB;

  // ===== State Tracking =====
  private boolean gearUpPressedLast = false;
  private boolean gearDownPressedLast = false;

  private boolean liftActive = false;
  private double liftStartTime = 0.0;

  @Override
  public void robotInit() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    frontLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    upMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double time = timer.get();

    if (time < 3.0) {
      setDrive(AUTO_DRIVE_SPEED, 0);
      upMotor.set(0);
    } else if (time < 4.0) {
      setDrive(0, 0);
      upMotor.set(0);
    } else if (time < 5.0) {
      setDrive(0, 0);
      upMotor.set(AUTO_LIFT_SPEED);
    } else {
      stopAll();
    }
  }

  @Override
  public void teleopInit() {
    liftActive = false;
    currentGear = GEAR_NORMAL;
  }

  @Override
  public void teleopPeriodic() {
    // --- Gear shifting ---
    boolean gearUpPressed = driverJoystick.getRawButton(BUTTON_GEAR_UP);
    boolean gearDownPressed = driverJoystick.getRawButton(BUTTON_GEAR_DOWN);

    if (gearUpPressed && !gearUpPressedLast) {
      if (currentGear < GEAR_FAST) {
        currentGear++;
      }
    }

    if (gearDownPressed && !gearDownPressedLast) {
      if (currentGear > GEAR_SLOW) {
        currentGear--;
      }
    }

    gearUpPressedLast = gearUpPressed;
    gearDownPressedLast = gearDownPressed;

    // --- Drive control ---
    double rawSpeed = driverJoystick.getRawAxis(1);
    double rawTurn = -driverJoystick.getRawAxis(2);

    rawSpeed = applyDeadband(rawSpeed);
    rawTurn = applyDeadband(rawTurn);

    double curvedSpeed = Math.copySign(Math.pow(Math.abs(rawSpeed), 1.5), rawSpeed);
    double curvedTurn = Math.copySign(Math.pow(Math.abs(rawTurn), 1.5), rawTurn);

    boolean turboMode = driverJoystick.getRawButton(BUTTON_TURBO);
    double sensitivity = turboMode ? TURBO_SENSITIVITY : NORMAL_SENSITIVITY;

    double effectiveSensitivity = (sensitivity * 0.7) + (GEAR_SENSITIVITY[currentGear] * 0.3);
    double maxSpeed = GEAR_MAX_SPEED[currentGear];

    double left = (curvedSpeed + curvedTurn) * effectiveSensitivity;
    double right = (curvedSpeed - curvedTurn) * effectiveSensitivity;

    left = clampWithMin(left, -maxSpeed, maxSpeed, GEAR_MIN_SPEED);
    right = clampWithMin(right, -maxSpeed, maxSpeed, GEAR_MIN_SPEED);

    setDrive(left, right);

    // --- Lift control ---
    boolean shortLiftPressed = driverJoystick.getRawButton(BUTTON_SHORT_LIFT);
    boolean manualLiftPressed = driverJoystick.getRawButton(BUTTON_MANUAL_LIFT);

    if (shortLiftPressed && !liftActive) {
      liftActive = true;
      liftStartTime = Timer.getFPGATimestamp();
    }

    if (liftActive) {
      if (Timer.getFPGATimestamp() - liftStartTime < SHORT_LIFT_DURATION) {
        upMotor.set(LIFT_SPEED * 0.25);
      } else {
        liftActive = false;
        upMotor.set(0);
      }
    } else if (manualLiftPressed) {
      upMotor.set(LIFT_SPEED);
    } else {
      upMotor.set(0);
    }
  }

  // ===== Utility functions =====
  private void setDrive(double left, double right) {
    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(-right);
    backRightMotor.set(-right);
  }

  private void stopAll() {
    setDrive(0, 0);
    upMotor.set(0);
  }

  private double applyDeadband(double value) {
    return Math.abs(value) < DEADZONE ? 0.0 : value;
  }

  private double clampWithMin(double val, double min, double max, double minMagnitude) {
    if (val == 0) return 0;
    double sign = Math.signum(val);
    double abs = Math.abs(val);
    if (abs < minMagnitude) abs = minMagnitude;
    if (abs > max) abs = max;
    return sign * abs;
  }

  @Override
  public void disabledInit() {
    stopAll();
  }

  @Override
  public void testInit() {
    stopAll();
  }
}
