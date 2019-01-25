
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  Spark pidOutput = new Spark(9);
  WPI_TalonSRX left1 = new WPI_TalonSRX(1);
  WPI_TalonSRX left2 = new WPI_TalonSRX(2);
  WPI_TalonSRX left3 = new WPI_TalonSRX(3);
  WPI_TalonSRX right1 = new WPI_TalonSRX(4);
  WPI_TalonSRX right2 = new WPI_TalonSRX(5);
  WPI_TalonSRX right3 = new WPI_TalonSRX(6);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2, left3);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2, right3);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);
  
  Joystick driver = new Joystick(0);//Joystick for the driver
  DigitalInput in1 = new DigitalInput(0);
  DigitalInput in2 = new DigitalInput(1);
  //Joystick manip = new Joystick(1); //Joystick for the manipulator
  AHRS navx = new AHRS(Port.kMXP);  //NavX
  Limelight limelight = new Limelight();//Limelight object to handle getting the data

  PIDController turnPID = new PIDController(0.04, 0, 0, limelight, pidOutput);

  @Override
  public void disabledInit() {
    turnPID.disable();
  }

  @Override
  public void disabledPeriodic() {
    read();
  }

  @Override
  public void robotInit() {
    mainDrive.setSafetyEnabled(false);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }
  @Override
  public void teleopPeriodic() {
    if(driver.getRawButtonPressed(4)) {
      turnPID.reset();
      turnPID.enable();
    }
    if(driver.getRawButtonReleased(4)) {
      turnPID.disable();
    }
    double y = driver.getRawAxis(1)*0.8;
    double rot = driver.getRawAxis(4)*-0.8;
    if(Math.abs(y) >= 0.1 || Math.abs(rot) >= 0.1) {
      mainDrive.arcadeDrive(y, rot);
    } else if(driver.getRawButton(4)) {
      if(limelight.hasValidTarget()) {
        double height = limelight.getHeight();
        double speed = 0;
        if(9 <= height && height <= 11) {
          speed = 0;
        } else {
          speed = (height-10) * 0.2;
        }
        mainDrive.arcadeDrive(speed, turnPID.get());
      } else {
        mainDrive.arcadeDrive(0, 0);
      }
    } else {
      mainDrive.arcadeDrive(0, 0);
    }

    read();//Read from sensors
  }

  private void read() {
    SmartDashboard.putNumber("Center-x", limelight.getX());
    SmartDashboard.putNumber("NavX", navx.getAngle());
    SmartDashboard.putNumber("Skew", limelight.getSkew());
    SmartDashboard.putNumber("Center-y", limelight.getY());
    SmartDashboard.putBoolean("in1", in1.get());
    SmartDashboard.putBoolean("in2", in2.get());
    SmartDashboard.putNumber("Height", limelight.getHeight());
  }

  @Override
  public void testPeriodic() {
  }
}
