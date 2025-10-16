package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.SignalsConfig; *//
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  private static final SparkMax frontLeftMotor = new SparkMax(12, MotorType.kBrushed);
  private static final SparkMax backLeftMotor = new SparkMax(11, MotorType.kBrushed);
  private static final SparkMax frontRightMotor = new SparkMax(13, MotorType.kBrushed);
  private static final SparkMax backRightMotor = new SparkMax(16, MotorType.kBrushed);
  private static final SparkMax upMotor = new SparkMax(15, MotorType.kBrushed);/*eyvallah*/

  private static final Joystick driverJoystick = new Joystick(0);
  private static final Timer timerobj = new Timer();

  @Override
  public void robotInit() {
    // Create SparkMaxConfig for motor configuration
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    
    // Set idle mode to brake
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    // Create SignalsConfig for signal-related settings
   // SignalsConfig signalsConfig = new SignalsConfig(); ** //
    // Disable unnecessary signals (equivalent to signals(false) in older versions)

    // Apply motor configuration to all motors
    frontLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    upMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void robotPeriodic() {
   /* 
    double speed = driverJoystick.getRawAxis(1) * 0.4;
    double turn = -driverJoystick.getRawAxis(2) * 0.3;

    double left = speed + turn;
    double right = speed - turn;
    // System.out.println("Left: " + left);
    // System.out.println("Right: " + right);

    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(-right);
    backRightMotor.set(-right);
    */
    /*
    double sspeed = driverJoystick.getButtonCount()*0.2;
    double tturn = -driverJoystick.getButtonCount() * 0.3;
    
    double lleft = sspeed + tturn;
    double rright = sspeed - tturn; */

    
    

    

  }

  @Override
  public void autonomousInit() {
    // Autonomous initialization settings
    timerobj.reset();
   // timerobj.start();
  }

  @Override
  public void autonomousPeriodic() {
    double time = timerobj.get();
    
    timerobj.start();
    if (time < 3) 
    {
      
      frontLeftMotor.set(0.4);
      backLeftMotor.set(0.4);
      frontRightMotor.set(-0.4);
      backRightMotor.set(-0.4);
      
    } else if (time > 3 && time < 4) {
      frontLeftMotor.set(0);
      backLeftMotor.set(0);
      frontRightMotor.set(0);
      backRightMotor.set(0);
      
      
    }else if(time > 5 && time <6){
      upMotor.set(0.4);
    }
    else 
    {
      frontLeftMotor.set(0);
      backLeftMotor.set(0);
      frontRightMotor.set(0);
      backRightMotor.set(0);
      upMotor.set(0);
    }
  }

  @Override
  public void teleopInit() {
    // Teleoperation initialization settings
  }

  @Override
  public void teleopPeriodic() {
    double liftPower = 0.2;

    double speed = driverJoystick.getRawAxis(1) * 0.2;
    double turn = -driverJoystick.getRawAxis(4) * 0.2;

    double left = speed + turn;
    double right = speed - turn;
    // System.out.println("Left: " + left);
    // System.out.println("Right: " + right);

    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(-right);
    backRightMotor.set(-right);

    /* 
    if (driverJoystick.getRawButton(2)) {
      liftPower = 3;
      upMotor.set(8); // Note: This seems incorrect; likely should set to liftPower
      upMotor.set(-liftPower);
      Timer.delay(0.65);                                                                  //Düzeltme
      upMotor.set(0);
    } */
    if (driverJoystick.getRawButton(3)) {
      liftPower = 0.1;
      upMotor.set(0.01); // Note: This seems incorrect; likely should set to liftPower
      upMotor.set(liftPower);
      Timer.delay(0.65);
      upMotor.set(0);
    }
    /* 
    if(driverJoystick.getRawButton(2))
    {
      upMotor.set(-0.7);                          //Düzeltme
    }*/
    if(driverJoystick.getRawButton(1))
    {
      upMotor.set(0.4);
    }
    else
    {
      upMotor.set(0);
    }
  }

  @Override
  public void disabledInit() {
    // Actions when disabled
  }

  @Override
  public void disabledPeriodic() {
    // Periodic actions when disabled
  }

  @Override
  public void testInit() {
    // Test initialization settings
  }

  @Override
  public void testPeriodic() {
    // Test periodic operations
  }

  @Override
  public void simulationInit() {
    // Simulation initialization settings
  }

  @Override
  public void simulationPeriodic() {
    // Simulation periodic operations
  }
} 