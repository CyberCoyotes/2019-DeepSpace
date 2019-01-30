
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
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
  AHRS navx = new AHRS(Port.kMXP);  //NavX
  Limelight limelight = new Limelight();//Limelight object to handle getting the data

  double goalHeight = 10;//Used in vision processing
  double goalThreshold = 1;
  double distanceKP = 0.2;
  double rotationKP = 0.04;

  @Override
  public void disabledInit() {
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
    double y = driver.getRawAxis(1)*0.8;//Get the joystick values and reduce the max speed by 20%
    double rot = driver.getRawAxis(4)*-0.8;

    if(Math.abs(y) >= 0.1 || Math.abs(rot) >= 0.1) {//Check the threshholds
      mainDrive.arcadeDrive(y, rot);//Drive with joystick values
    } else if(driver.getRawButton(4)) {//If the Y button is pressed, activate vision tracking mode
      if(limelight.hasValidTarget()) {//If the limelight sees something...
        /*
        goalHeight is the perceived object height the robot is trying to reach.
        A larger height means closer. You can set the threshold tolerance by
        adjusting the goalThreshold value. Currently it is set to not move when
        the perceived height is between 9 and 11. If you change distanceKP, it
        changes how fast the robot with turn towards the target. Be careful: if
        it is too high, the robot will oscillate out of control.
        */
        double height = limelight.getHeight();//Get the perceived height of the object (used for front-to-back position)
        double speed = 0;//Create a variable to store the calculated forwards speed
        if(goalHeight-goalThreshold <= height &&  height <= goalHeight+goalThreshold) {//If the front to back is within a threshhold...
          speed = 0;//Set the speed to 0
        } else {//If the object is not in the threshold...
          speed = (height-goalHeight) * distanceKP;//Set the speed to the height minus the goal times kP
        }
        mainDrive.arcadeDrive(speed, limelight.getX()*rotationKP); //Drive the robot
      } else {//If there are no valid targets...
        mainDrive.arcadeDrive(0, 0);//Stop
      }
    } else {//If the user does not want to drive...
      mainDrive.arcadeDrive(0, 0);//Stop
    }

    read();//Read from sensors
  }

  private void read() {
    SmartDashboard.putNumber("Center-x", limelight.getX());
    SmartDashboard.putNumber("NavX", navx.getAngle());
    SmartDashboard.putNumber("Center-y", limelight.getY());
    SmartDashboard.putNumber("Height", limelight.getHeight());
  }

  @Override
  public void testPeriodic() {
  }
}
