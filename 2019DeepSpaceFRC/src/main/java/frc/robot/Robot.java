package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
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
  WPI_TalonSRX right1 = new WPI_TalonSRX(10);
  WPI_VictorSPX right2 = new WPI_VictorSPX(11);
  WPI_VictorSPX right3 = new WPI_VictorSPX(12);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2, left3);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2, right3);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);

  WPI_VictorSPX intake = new WPI_VictorSPX(8);//Victor for intake motor
  WPI_VictorSPX wrist = new WPI_VictorSPX(6);//Victor for wrist motor
  WPI_VictorSPX wrist2 = new WPI_VictorSPX(7);
  WPI_TalonSRX elevator1 = new WPI_TalonSRX(4);//Talon for the elevator
  WPI_VictorSPX elevator2 = new WPI_VictorSPX(9);//Victor for the elevator
  SpeedControllerGroup elevator = new SpeedControllerGroup(elevator1, elevator2);
  WPI_VictorSPX liftDriver = new WPI_VictorSPX(5);

  DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse;
  DoubleSolenoid.Value out = DoubleSolenoid.Value.kForward;
  DoubleSolenoid shifter = new DoubleSolenoid(3, 4);
  DoubleSolenoid frontLift = new DoubleSolenoid(2, 5);
  DoubleSolenoid backLift = new DoubleSolenoid(1, 6);
  Solenoid puncher = new Solenoid(7);

  Servo leftServo = new Servo(0);
  Servo rightServo = new Servo(1);
  
  Joystick driver = new Joystick(0);//Joystick for the driver
  Joystick manip = new Joystick(1);
  AHRS navx = new AHRS(Port.kMXP);//NavX
  Limelight limelight = new Limelight();//Limelight object to handle getting the data
  CTREEncoder rightEnc = new CTREEncoder(right1, false);
  CTREEncoder elevatorEnc = new CTREEncoder(elevator1, false);
  PressureSensor pressureSensor = new PressureSensor(3);
  Encoder wristEnc = new Encoder(0, 1, false, EncodingType.k2X);

  double liftSetPoint = 0;
  double wristSetPoint = 94;

  double liftKP = -0.00025;
  double wristKP = -0.04;
  double turnKP = -0.019;//0.05
  double straightKP = 0.03;//0.004

  final double hatch1 = 0;//TO DO
  final double hatch2 = 0;
  final double hatch3 = 0;
  final double port1 = 0;
  final double port2 = 0;
  final double port3 = 0;
  final double vertical = 90;
  final double wristFloor = 0;
  final double liftFloor = 0;
  final double ballPick = 0;

  long servoTime = Long.MAX_VALUE;
  int lastPOVState = -1;
  boolean down = false;
  boolean updateSD = true;
  boolean point = false;

  @Override
  public void robotInit() {
    mainDrive.setSafetyEnabled(false);
    elevator1.setInverted(true);
    wrist2.setInverted(true);
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
    wristSetPoint = (-94.0/91.0)*wristEnc.get()+94;
    elevatorEnc.reset();
    liftSetPoint = elevatorEnc.get();
    rightServo.set(0);
    leftServo.set(1);
  }

  @Override
  public void teleopPeriodic() {

    double y = Math.pow(driver.getRawAxis(1), 1);
    double rot = Math.pow(-driver.getRawAxis(2), 1);

    if(Math.abs(y) >= 0.15 || Math.abs(rot) >= 0.15) {
      mainDrive.arcadeDrive(y, rot);
    } else if(driver.getRawButton(1) && limelight.hasValidTarget()){
      double rSpeed = limelight.getX()*turnKP;
      double fSpeed = (limelight.getHeight()-64)*straightKP;
      mainDrive.arcadeDrive(fSpeed , rSpeed);
    } else {
      mainDrive.arcadeDrive(0,0);
    }

    if(driver.getRawButton(12)) {
      liftSetPoint = hatch1;
    }
    if(driver.getRawButton(10)) {
      liftSetPoint = hatch2;
    }
    if(driver.getRawButton(8)) {
      liftSetPoint = hatch3;
    }

    shifter.set(out);

    double liftSpeed = manip.getRawAxis(1);
    if(liftSpeed < 0) {
      liftSpeed*=0.5;
    } else {
      liftSpeed *= 0.15;
    }
    if(Math.abs(liftSpeed) >= 0.05) {
      //elevator.set(liftSpeed);
      liftSetPoint = elevatorEnc.get();
    } else {
      liftSpeed = (liftSetPoint - elevatorEnc.get()) * liftKP;
      if(liftSpeed > 0.5) {
        liftSpeed = 0.5;
      } else if(liftSpeed < -0.5) {
        liftSpeed = -0.15;
      }
      //elevator.set(liftSpeed);
    }
    elevator.set(0);

    double wristSpeed = manip.getRawAxis(5);
    wristSpeed = wristSpeed > 0 ? wristSpeed*=0.2 : wristSpeed;
    double wristPosition = (-94.0/91.0)*wristEnc.get()+94;
    if(Math.abs(wristSpeed) >= 0.05) {
      wrist.set(wristSpeed);
      wrist2.set(wristSpeed);
      wristSetPoint = wristPosition;
    } else {
      wristSpeed = (wristSetPoint - wristPosition) * wristKP;
      wrist.set(wristSpeed);
      wrist2.set(wristSpeed);
      SmartDashboard.putNumber("Wrist Speed", wristSpeed);
    }

    int currentPOVState = manip.getPOV();
    if(currentPOVState > lastPOVState) {
      if(currentPOVState == 0) {
        servoTime = System.currentTimeMillis();
        down = true;
        rightServo.set(0);
      }
      if(currentPOVState == 180) {
        servoTime = System.currentTimeMillis();
        down = false;
        rightServo.set(1);
      }
    }
    if(!down && System.currentTimeMillis() - servoTime >= 300) {
      leftServo.set(0);
    }
    if(!down && System.currentTimeMillis() - servoTime >= 600 && System.currentTimeMillis() - servoTime < 1000) {
      puncher.set(true);
    }
    if(!down && System.currentTimeMillis() - servoTime >= 1000) {
      puncher.set(false);
    }
    if(down && System.currentTimeMillis() - servoTime >= 300) {
      leftServo.set(1);
    }
    lastPOVState = currentPOVState;

    if (manip.getRawAxis(2) >= 0.5){
      intake.set(0.6); 
    } else if (manip.getRawAxis(3) >= 0.5){
      intake.set(-0.6); 
    } else {
      intake.set(0);
    }

    if(manip.getRawButton(10)){
      point = !point; // if button 10 is pressed, then point turns true/false.
      if (point){ // if point = true, then set it to the floor
        wristSetPoint = wristFloor;
      } else { // if point = false. then set it to the KP (Constant Proportional) vertical
        wristSetPoint = vertical;
      }
    }
    read();//Read from sensors
  }

  private void read() {
    if(updateSD) {
      SmartDashboard.putNumber("Center-x", limelight.getX());
      SmartDashboard.putNumber("NavX", navx.getAngle());
      SmartDashboard.putNumber("Center-y", limelight.getY());
      SmartDashboard.putNumber("Height", limelight.getHeight());
      SmartDashboard.putNumber("Pressure", pressureSensor.get());
      SmartDashboard.putNumber("Elevator Speed 2", elevator.get());
      SmartDashboard.putNumber("Elevator Height", elevatorEnc.get());
    }
    updateSD = !updateSD;
  }

  @Override
  public void testPeriodic() {
  }
}