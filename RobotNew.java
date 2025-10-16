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
  private static final double DRIVE_SPEED_LIMIT = 0.4;
  private static final double TURN_SPEED_LIMIT = 0.3;
  private static final double LIFT_SPEED = 0.4;
  private static final double AUTO_DRIVE_SPEED = 0.4;
  private static final double AUTO_LIFT_SPEED = 0.4;
  private static final double DEADZONE = 0.05;
  private static final double SHORT_LIFT_DURATION = 0.65; // seconds

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

    // Phase 1: drive forward (0–3s)
    if (time < 3.0) {
      setDrive(AUTO_DRIVE_SPEED, 0);
      upMotor.set(0);
    }
    // Phase 2: stop (3–4s)
    else if (time < 4.0) {
      setDrive(0, 0);
      upMotor.set(0);
    }
    // Phase 3: lift (4–5s)
    else if (time < 5.0) {
      setDrive(0, 0);
      upMotor.set(AUTO_LIFT_SPEED);
    }
    // Phase 4: stop everything after 5s
    else {
      stopAll();
    }
  }

  // ===== TELEOPERATED =====
  @Override
  public void teleopInit() {
    liftActive = false;
  }

  @Override
  public void teleopPeriodic() {
    // ----- Drive control -----
    double speed = applyDeadband(driverJoystick.getRawAxis(1)) * DRIVE_SPEED_LIMIT;
    double turn = -applyDeadband(driverJoystick.getRawAxis(4)) * TURN_SPEED_LIMIT;

    double left = speed + turn;
    double right = speed - turn;

    setDrive(left, right);

    // ----- Lift control -----
    boolean shortLiftPressed = driverJoystick.getRawButton(3);
    boolean manualLiftPressed = driverJoystick.getRawButton(1);

    if (shortLiftPressed && !liftActive) {
      liftActive = true;
      liftStartTime = Timer.getFPGATimestamp();
    }

    // Handle short timed lift (non-blocking)
    if (liftActive) {
      if (Timer.getFPGATimestamp() - liftStartTime < SHORT_LIFT_DURATION) {
        upMotor.set(LIFT_SPEED * 0.25); // gentle short lift
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

  // ===== OPTIONAL PLACEHOLDERS =====
  @Override
  public void disabledInit() {
    stopAll();
  }

  @Override
  public void testInit() {
    stopAll();
  }
}

