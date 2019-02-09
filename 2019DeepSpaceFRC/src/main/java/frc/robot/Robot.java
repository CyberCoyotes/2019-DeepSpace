package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  WPI_TalonSRX left1 = new WPI_TalonSRX(3);
  WPI_VictorSPX left2 = new WPI_VictorSPX(2);
  WPI_VictorSPX left3 = new WPI_VictorSPX(1);
  WPI_TalonSRX right1 = new WPI_TalonSRX(9);
  WPI_VictorSPX right2 = new WPI_VictorSPX(10);
  WPI_VictorSPX right3 = new WPI_VictorSPX(11);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2, left3);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2, right3);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);

  WPI_VictorSPX intake = new WPI_VictorSPX(7);//Victor for intake motor
  WPI_VictorSPX wrist = new WPI_VictorSPX(6);//Victor for wrist motor
  WPI_TalonSRX elevator1 = new WPI_TalonSRX(4);//Talon for the elevator
  WPI_VictorSPX elevator2 = new WPI_VictorSPX(8);//Victor for the elevator
  SpeedControllerGroup elevator = new SpeedControllerGroup(elevator1, elevator2);//Group the elevator controllers together

  WPI_VictorSPX liftDriver = new WPI_VictorSPX(5);//Victor for the motor that drives the robot for the climb

  DoubleSolenoid shifter = new DoubleSolenoid(0, 1);
  DoubleSolenoid frontLift = new DoubleSolenoid(2, 3);
  DoubleSolenoid backLift = new DoubleSolenoid(4, 5);
  Solenoid puncher = new Solenoid(6);
  
  Joystick driver = new Joystick(0);//Joystick for the driver
  Joystick manip = new Joystick(1);
  AHRS navx = new AHRS(Port.kMXP);//NavX
  Limelight limelight = new Limelight();//Limelight object to handle getting the data
  CTREEncoder leftEnc = new CTREEncoder(left1);
  CTREEncoder rightEnc = new CTREEncoder(right1);
  CTREEncoder elevatorEnc = new CTREEncoder(elevator1);
  PressureSensor pressureSensor = new PressureSensor(3);
  Encoder wristEnc = new Encoder(0, 1, false, EncodingType.k2X);

  double goalHeight = 10;//Used in vision processing
  double goalThreshold = 1;
  double distanceKP = 0.2;
  double rotationKP = 0.04;
  double liftKP = 0;
  double wristKP = 0;
  double stillKP = 0;
  double liftSetPoint = 0;
  double wristSetPoint = 0;

  double hatch1 = 0;//TODO
  double hatch2 = 0;
  double hatch3 = 0;
  double port1 = 0;
  double port2 = 0;
  double port3 = 0;
  double vertical = 0;
  double wristFloor = 0;
  double liftFloor = 0;
  double ballPick = 0;
  boolean track = false;

  Preferences tuner = Preferences.getInstance();

  @Override
  public void robotInit() {
    mainDrive.setSafetyEnabled(false);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    read();
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
    liftKP = tuner.getDouble("LiftKP", 0.0);
    wristKP = tuner.getDouble("WristKP", 0.0);
    stillKP = tuner.getDouble("StillKP", 0.0);

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

    double liftSpeed = manip.getRawAxis(1);
    if(Math.abs(liftSpeed) >= 0.05) {
      elevator.set(liftSpeed);
      liftSetPoint = elevatorEnc.get();
    } else {
      liftSpeed = (liftSetPoint - elevatorEnc.get()) * liftKP;
      elevator.set(liftSpeed);
    }
    if(manip.getRawButton(1)) { 
      liftSetPoint = hatch1;
      wristSetPoint = vertical;
    }
    if(manip.getRawButton(2)) {
      liftSetPoint = hatch2;
      wristSetPoint = vertical;
    }
    if(manip.getRawButton(3)) {
      liftSetPoint = hatch3;
      wristSetPoint = vertical;
    }
    if(manip.getRawButton(4)) {
      liftSetPoint = port1;
    }
    if(manip.getRawButton(5)) {
      liftSetPoint = port2;
    }
    if(manip.getRawButton(6)) {
      liftSetPoint = port3;
    }

    double wristSpeed = manip.getRawAxis(5);
    if(Math.abs(wristSpeed) >= 0.05) {
      wrist.set(wristSpeed);
      wristSetPoint = wristEnc.get();
    } else {
      double position = wristEnc.get();
      wristSpeed = (wristSetPoint - position) * wristKP + Math.cos(position) * stillKP;
    }
    if(manip.getRawButton(7)) {
      wristSetPoint = vertical;
    }
    if(manip.getRawButton(8)) {
      wristSetPoint = wristFloor;
      liftSetPoint = liftFloor;
    }
    if(manip.getRawButton(9)) {
      wristSetPoint = ballPick;
    }

    if(manip.getRawButton(1) && manip.getRawButton(10)) {
      liftSetPoint = hatch1;
      wristSetPoint = vertical;
    }

    read();//Read from sensors
  }

  private void read() {
    SmartDashboard.putNumber("Center-x", limelight.getX());
    SmartDashboard.putNumber("NavX", navx.getAngle());
    SmartDashboard.putNumber("Center-y", limelight.getY());
    SmartDashboard.putNumber("Height", limelight.getHeight());
    SmartDashboard.putNumber("Elevator Height", elevatorEnc.get());
    SmartDashboard.putNumber("Wrist Angle", wristEnc.get());
    SmartDashboard.putNumber("Left Encoder", leftEnc.get());
    SmartDashboard.putNumber("Right Encoder", rightEnc.get());
    SmartDashboard.putNumber("Read wristKP", wristKP);
    SmartDashboard.putNumber("Read liftKP", liftKP);
    SmartDashboard.putNumber("Read stillKP", stillKP);
    SmartDashboard.putNumber("Pressure", pressureSensor.get());
  }

  @Override
  public void testPeriodic() {
  }
}