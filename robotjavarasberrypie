package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  private static final SparkMax frontLeftMotor = new SparkMax(12, MotorType.kBrushed);
  private static final SparkMax backLeftMotor = new SparkMax(11, MotorType.kBrushed);
  private static final SparkMax frontRightMotor = new SparkMax(13, MotorType.kBrushed);
  private static final SparkMax backRightMotor = new SparkMax(16, MotorType.kBrushed);
  private static final SparkMax upMotor = new SparkMax(15, MotorType.kBrushed);

  private static final Joystick driverJoystick = new Joystick(0);

  // === SERIAL (Raspberry Pi) ===
  private SerialPort piSerial;
  private boolean sawBlack = false;
  private double blackStartTime = 0;

  @Override
  public void robotInit() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    frontLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    upMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SERIAL INIT
    try {
      piSerial = new SerialPort(115200, SerialPort.Port.kUSB);
      System.out.println("Serial port opened!");
    } catch (Exception e) {
      System.out.println("Could not open serial port!");
    }
  }

  // === READ SERIAL FROM PI ===
  private String readPiSerial() {
    try {
      if (piSerial != null && piSerial.getBytesReceived() > 0) {
        String s = piSerial.readString().trim();
        return s;
      }
    } catch (Exception e) {
      System.out.println("Serial read error: " + e.getMessage());
    }
    return "";
  }

  @Override
  public void teleopPeriodic() {
    // --------- READ PI SERIAL ---------
    String data = readPiSerial();
    if (data.equals("BLACK") && !sawBlack) {
      sawBlack = true;
      blackStartTime = Timer.getFPGATimestamp();
      System.out.println("BLACK DETECTED → MOVING FORWARD 1 SEC");
    }

    // --------- BLACK EVENT ACTIVE ---------
    if (sawBlack) {
      double elapsed = Timer.getFPGATimestamp() - blackStartTime;

      if (elapsed < 1.0) {
        // DRIVE FORWARD FOR 1 SECOND
        frontLeftMotor.set(0.4);
        backLeftMotor.set(0.4);
        frontRightMotor.set(-0.4);
        backRightMotor.set(-0.4);
        return; // skip manual controls until done
      } else {
        sawBlack = false;
      }
    }

    // --------- NORMAL MANUAL DRIVE ---------
    double speed = driverJoystick.getRawAxis(1) * 0.2;
    double turn = -driverJoystick.getRawAxis(4) * 0.2;

    double left = speed + turn;
    double right = speed - turn;

    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(-right);
    backRightMotor.set(-right);

    // --------- LIFT MOTOR CONTROL ---------
    if (driverJoystick.getRawButton(1)) {
      upMotor.set(0.4);
    } else {
      upMotor.set(0);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // You can add Serial behavior here too if needed.
  }
}
