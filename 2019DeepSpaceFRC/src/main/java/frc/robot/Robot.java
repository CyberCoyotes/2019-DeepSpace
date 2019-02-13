package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
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
  WPI_TalonSRX right1 = new WPI_TalonSRX(9);
  WPI_VictorSPX right2 = new WPI_VictorSPX(10);
  WPI_VictorSPX right3 = new WPI_VictorSPX(11);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2, left3);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2, right3);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);

  WPI_VictorSPX intake = new WPI_VictorSPX(7);//Victor for intake motor
  WPI_VictorSPX wrist = new WPI_VictorSPX(6);//Victor for wrist motor
  WPI_TalonSRX elevator = new WPI_TalonSRX(4);//Talon for the elevator
  WPI_VictorSPX elevator2 = new WPI_VictorSPX(8);//Victor for the elevator
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
  CTREEncoder leftEnc = new CTREEncoder(left1, false);
  CTREEncoder rightEnc = new CTREEncoder(right1, false);
  CTREEncoder elevatorEnc = new CTREEncoder(elevator, false);
  PressureSensor pressureSensor = new PressureSensor(3);
  Encoder wristEnc = new Encoder(0, 1, false, EncodingType.k2X);

  double liftSetPoint = 0;
  double wristSetPoint = 94;

  double liftKP = 0;
  double wristKP = -0.1;
  double stillKP = -0.3;

  final double hatch1 = 0;//TODO
  final double hatch2 = 0;
  final double hatch3 = 0;
  final double port1 = 0;
  final double port2 = 0;
  final double port3 = 0;
  final double vertical = 90;
  final double wristFloor = 0;
  final double liftFloor = 0;
  final double ballPick = 0;

  long servoTime;
  boolean closed = false;
  boolean updateSD = true;

  Preferences tuner = Preferences.getInstance();

  @Override
  public void robotInit() {
    mainDrive.setSafetyEnabled(false);
    elevator.setInverted(true);
    elevator2.set(ControlMode.Follower, 4);
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

    double y = driver.getRawAxis(1);
    double rot = driver.getRawAxis(4)*-1;

    if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05) {
      mainDrive.arcadeDrive(y, rot);
    } else {
      mainDrive.arcadeDrive(0, 0);//Stop
    }

    shifter.set(driver.getRawButton(1) ? out : in);

    double liftSpeed = manip.getRawAxis(5);
    if(Math.abs(liftSpeed) >= 0.15) {
      elevator.set(liftSpeed);
      liftSetPoint = elevatorEnc.get();
    } else {
      //liftSpeed = (liftSetPoint - elevatorEnc.get()) * liftKP;
      //elevator.set(liftSpeed);
      elevator.set(0);
    }
    
    double wristSpeed = manip.getRawAxis(1);
    double wristPosition = (-94.0/77.0)*wristEnc.get()+94;
    if(Math.abs(wristSpeed) >= 0.15) {
      wrist.set(wristSpeed);
      wristSetPoint = wristPosition;
    } else {
      //wristSpeed = Math.cos(wristSetPoint) * stillKP;
      wristSpeed = (wristSetPoint - wristPosition) * wristKP + Math.cos(wristPosition) * stillKP;
      //wristSpeed = stillKP;
      //wrist.set(wristSpeed);
      SmartDashboard.putNumber("Wrist Speed", wristSpeed);
    }
    
    if(manip.getRawButton(3)) {
      intake.set(0.3);
    } else if (manip.getRawButton(4)) {
      intake.set(-0.3);
    } else {
      intake.set(0);
    }

    if(manip.getRawButtonPressed(5)) {
      closed = true;
      servoTime = System.currentTimeMillis();//Record the start time
      rightServo.set(0.965);//Bring the first servo down
    }
    if(closed && System.currentTimeMillis() - servoTime > 250) {//If it has been 250 milliseconds...
      leftServo.set(0);//Bring the other servo down
    }
    if(closed && System.currentTimeMillis() - servoTime > 1000) {//If it has been 500 milliseconds...
      puncher.set(true);//Punch out the panel
    }
    if(closed && System.currentTimeMillis() - servoTime > 2000) {//If it has been 2 seconds...
      closed = false;//Reset the servos and piston
    }
    if(!manip.getRawButton(5)) {
      leftServo.set(0.4);
      rightServo.set(0.4);
      closed = false;
      puncher.set(false);
    }
    read();//Read from sensors
  }

  private void read() {
    if(updateSD) {
      liftKP = tuner.getDouble("LiftKP", 0.0);
      wristKP = tuner.getDouble("WristKP", 0.0);
      stillKP = tuner.getDouble("StillKP", 0.0);
      SmartDashboard.putNumber("Center-x", limelight.getX());
      SmartDashboard.putNumber("NavX", navx.getAngle());
      SmartDashboard.putNumber("Center-y", limelight.getY());
      SmartDashboard.putNumber("Height", limelight.getHeight());
      SmartDashboard.putNumber("Elevator Height", elevatorEnc.get());
      SmartDashboard.putNumber("Wrist Angle", (-94.0/77.0)*wristEnc.get()+94);
      SmartDashboard.putNumber("Left Encoder", leftEnc.get());
      SmartDashboard.putNumber("Right Encoder", rightEnc.get());
      SmartDashboard.putNumber("Read wristKP", wristKP);
      SmartDashboard.putNumber("Read liftKP", liftKP);
      SmartDashboard.putNumber("Read stillKP", stillKP);
      SmartDashboard.putNumber("Pressure", pressureSensor.get());
      SmartDashboard.putNumber("Left Elevator Voltage", elevator.getMotorOutputVoltage());
      SmartDashboard.putNumber("Manip Axis 5", manip.getRawAxis(5));
      SmartDashboard.putNumber("Driver Axis 5", driver.getRawAxis(5));
    }
    updateSD = !updateSD;
  }

  @Override
  public void testPeriodic() {
  }
}