
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
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
  Joystick manip = new Joystick(1); //Joystick for the manipulator
  //AHRS navx = new AHRS(Port.kMXP);  //NavX
  Limelight limelight = new Limelight();//Limelight object to handle getting the data

  @Override
  public void disabledPeriodic() {
    read();
  }

  @Override
  public void robotInit() {
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
  public void teleopPeriodic() {
    double y = driver.getRawAxis(1);
    double rot = driver.getRawAxis(0);
    if(Math.abs(y) >= 0.1 || Math.abs(rot) >= 0.1) {
      mainDrive.arcadeDrive(y, rot);
    } else {
      mainDrive.arcadeDrive(0, 0);
    }

    read();//Read from sensors
  }

  private void read() {
    SmartDashboard.putNumber("Center-x", limelight.getX());
    SmartDashboard.putNumber("Skew", limelight.getSkew());
  }

  @Override
  public void testPeriodic() {
  }
}
