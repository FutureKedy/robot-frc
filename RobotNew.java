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
  private static final double SHORT_LIFT_DURATION = 0.65; // seconds
  private static final double NORMAL_SENSITIVITY = 0.5; 
  private static final double TURBO_SENSITIVITY = 0.8;

  // ===== Gear System =====
  private static final int GEAR_SLOW = 0;
  private static final int GEAR_NORMAL = 1;
  private static final int GEAR_FAST = 2;
  private int currentGear = GEAR_NORMAL;

  // Gear-specific speed caps
  private static final double[] GEAR_MAX_SPEED = { 0.3, 0.5, 0.8 };
  private static final double[] GEAR_SENSITIVITY = { 0.4, 0.6, 0.8 };

  // Button IDs (change if needed)
  private static final int BUTTON_GEAR_UP = 7;   // L2
  private static final int BUTTON_GEAR_DOWN = 8; // R2

  private boolean gearUpPressedLast = false;
  private boolean gearDownPressedLast = false;

  // Lift control timing variables
  private boolean liftActive = false;
  private double liftStartTime = 0.0;

  @Override
  public void robotInit() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    // Apply configuration to all motors
    frontLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    upMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== AUTONOMOUS =====
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

  // ===== TELEOPERATED =====
  @Override
  public void teleopInit() {
    liftActive = false;
    currentGear = GEAR_NORMAL;
  }

  @Override
  public void teleopPeriodic() {
    // ----- Gear shifting -----
    boolean gearUpPressed = driverJoystick.getRawButton(BUTTON_GEAR_UP);
    boolean gearDownPressed = driverJoystick.getRawButton(BUTTON_GEAR_DOWN);

    if (gearUpPressed && !gearUpPressedLast) {
      if (currentGear < GEAR_FAST) currentGear++;
    }
    if (gearDownPressed && !gearDownPressedLast) {
      if (currentGear > GEAR_SLOW) currentGear--;
    }

    gearUpPressedLast = gearUpPressed;
    gearDownPressedLast = gearDownPressed;

    // ----- Drive control -----
    double rawSpeed = driverJoystick.getRawAxis(1);  
    double rawTurn = -driverJoystick.getRawAxis(4);  

    rawSpeed = applyDeadband(rawSpeed);
    rawTurn = applyDeadband(rawTurn);

    double curvedSpeed = Math.copySign(Math.pow(Math.abs(rawSpeed), 1.5), rawSpeed);
    double curvedTurn = Math.copySign(Math.pow(Math.abs(rawTurn), 1.5), rawTurn);

    boolean turboMode = driverJoystick.getRawButton(5);
    double sensitivity = turboMode ? TURBO_SENSITIVITY : NORMAL_SENSITIVITY;

    // Combine gear sensitivity and normal sensitivity
    double effectiveSensitivity = (sensitivity * 0.7) + (GEAR_SENSITIVITY[currentGear] * 0.3);
    double maxSpeed = GEAR_MAX_SPEED[currentGear];

    double left = (curvedSpeed + curvedTurn) * effectiveSensitivity;
    double right = (curvedSpeed - curvedTurn) * effectiveSensitivity;

    // Apply gear-based max speed cap
    left = clamp(left, -maxSpeed, maxSpeed);
    right = clamp(right, -maxSpeed, maxSpeed);

    setDrive(left, right);

    // ----- Lift control -----
    boolean shortLiftPressed = driverJoystick.getRawButton(3);
    boolean manualLiftPressed = driverJoystick.getRawButton(1);

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

  // ===== UTILITY FUNCTIONS =====
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

  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
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
