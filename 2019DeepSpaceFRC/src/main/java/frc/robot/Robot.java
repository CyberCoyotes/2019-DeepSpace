package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
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
  DoubleSolenoid frontLift = new DoubleSolenoid(1, 6);
  DoubleSolenoid backLift = new DoubleSolenoid(5, 2);
  Solenoid puncher = new Solenoid(7);

  Servo leftServo = new Servo(0);
  Servo rightServo = new Servo(1);
  
  Joystick driver = new Joystick(0);//Joystick for the driver
  Joystick manip = new Joystick(1);
  AHRS navx = new AHRS(Port.kMXP);//NavX
  Limelight limelight = new Limelight();//Limelight object to handle getting the data
  //CameraServer camera = CameraServer.getInstance();
  CTREEncoder rightEnc = new CTREEncoder(right1, false);
  CTREEncoder elevatorEnc = new CTREEncoder(elevator1, false);
  PressureSensor pressureSensor = new PressureSensor(3);
  Encoder wristEnc = new Encoder(0, 1, false, EncodingType.k2X);

  double liftSetPoint = 0;
  double wristSetPoint = 94;

  double liftKP = -0.00025;
  double wristKP = -0.04;
  double turnKP = -0.019;//0.05

  final double liftHatchOffset = 16000;
  final double liftOffset = 2500;
  double previous = 0;
  final double hatch1 = 0;//TO DO
  final double hatch2 = 26896;
  final double hatch3 = 53792;
  final double port1 = 13000;
  final double port2 = 30000;
  final double port3 = 60000;

  long servoTime = Long.MAX_VALUE;
  int lastPOVState = -1;
  boolean down = false;
  boolean wristDown = true;
  boolean lift20Toggle = false;
  boolean wasAuton = false;

  @Override
  public void robotInit() {
    mainDrive.setSafetyEnabled(false);
    elevator1.setInverted(true);
    wrist2.setInverted(true);
    elevatorEnc.reset();
    //camera.startAutomaticCapture("cam0", 0);
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
    teleopInit();
    wasAuton = true;
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    if(!wasAuton) {
      wristSetPoint = (-94.0/91.0)*wristEnc.get()+94;
      wristSetPoint = 85;
      elevatorEnc.reset();
      liftSetPoint = elevatorEnc.get();
      rightServo.set(0);
      leftServo.set(1);
      wasAuton = false;
    }
  }

  @Override
  public void teleopPeriodic() {

    double y = Math.pow(driver.getRawAxis(1), 1);
    double rot = Math.pow(-driver.getRawAxis(2), 1);

    if(driver.getRawButtonPressed(1)) {
      previous = liftSetPoint;
      liftSetPoint = hatch2;
    }
    if(Math.abs(y) >= 0.15 || Math.abs(rot) >= 0.15) {
      mainDrive.arcadeDrive(y, rot);
    } else if(driver.getRawButton(1) && limelight.hasValidTarget()) {
      double rSpeed = limelight.getX()*turnKP;
      mainDrive.arcadeDrive(y, rSpeed);
    } else if(driver.getRawButton(2) && limelight.hasValidTarget()) {
      double rSpeed = limelight.getX()*turnKP;
      mainDrive.arcadeDrive(y, rSpeed);
    } else {
      mainDrive.arcadeDrive(0, 0);
    }
    if(driver.getRawButtonReleased(1)) {
      liftSetPoint = previous;
    }
    

    shifter.set(out);


    double liftSpeed = manip.getRawAxis(1); //Get the manual lift speed
    if(liftSpeed < 0) { //If the manual speed is negative...
      liftSpeed*=0.9; //Limit the up speed
    } else {  //Else...
      liftSpeed*=0.5; //Limit the down speed
    }
    if(driver.getRawButtonPressed(12)) {  //If the driver pressed button 12...
      liftSetPoint = hatch1;  //Set the base height to hatch1
      if(lift20Toggle) {  //If the hook offset is true...
        liftSetPoint+=liftHatchOffset;  //Add the offset
      }
    }
    if(driver.getRawButtonPressed(11)) {  //If the driver pressed button 12...
      liftSetPoint = port1;  //Set the base height to port1
      lift20Toggle = false;
    }
    if(driver.getRawButtonPressed(10)) {  //If the driver presses button 10...
      liftSetPoint = hatch2;  //Set the base height to hatch2
      if(lift20Toggle) {  //If the hook offset is true...
        liftSetPoint+=liftHatchOffset;  //Add the offsett
      }
    }
    if(driver.getRawButtonPressed(9)) {  //If the driver presses button 10...
      liftSetPoint = port2;  //Set the base height to port2
      lift20Toggle = false;
    }
    if(driver.getRawButtonPressed(8)) { //If the driver presses button 8...
      liftSetPoint = hatch3;  //Set the base height to hatch3
      if(lift20Toggle) {  //If the hook offset is true...
        liftSetPoint+=liftHatchOffset;  //Add the offset
      }
    }
    
    if(driver.getRawButtonPressed(7)) { //If the driver presses button 8...
      liftSetPoint = port3;  //Set the base height to hatch3
      lift20Toggle = false;
    }
    if(Math.abs(liftSpeed) >= 0.05 && !(elevatorEnc.get() < 2000 && liftSpeed >= 0)) { //If the manip wants to manual control the elevator...
      elevator.set(liftSpeed);  //Set the elevator speed
      liftSetPoint = elevatorEnc.get(); //Set the PID setpoint to the current reading
    } else {  //Else (if the PID is activated)
      if(liftSetPoint < 2000) {
        liftSetPoint = 2000;
      }
      liftSpeed = (liftSetPoint - elevatorEnc.get()) * liftKP;  //Calculate the lift speed based on error
      if(liftSpeed > 0.4) { //Limit the down speed
        liftSpeed = 0.4;
      } else if(liftSpeed < -0.75) {  //Limit the up speed
        liftSpeed = -0.75;
      }
      elevator.set(liftSpeed);  //Set the elevator
    }
    if(driver.getRawButtonPressed(5)) { //If button 5 is pressed...
      lift20Toggle=!lift20Toggle; //Switch the lift hook toggle
      if(lift20Toggle) {  //If the toggle is on...
        liftSetPoint+=liftHatchOffset;  //Add the toggle
      } else {  //If the toggle is off...
        liftSetPoint-=liftHatchOffset;  //Remove the toggle
      }
    }
    if(driver.getRawButtonPressed(3)) { //If button 3 is pressed...
      liftSetPoint+=liftOffset; //Add 2 inches
    }
    if(driver.getRawButtonReleased(3)) {  //If button 3 is released...
      liftSetPoint-=liftOffset; //Remove 2 inches
    }
    if(driver.getRawButtonPressed(4)) { //If button 4 is pressed...
      liftSetPoint-=liftOffset; //Remove 2 inches
    }
    if(driver.getRawButtonReleased(4)) {  //If button 4 is released
      liftSetPoint+=liftOffset; //Add 2 inches
    }

    double wristSpeed = manip.getRawAxis(5);  //Get the manual wrist speed
    wristSpeed = wristSpeed > 0 ? wristSpeed*=0.25 : wristSpeed; //Limit the down speed
    double wristPosition = (-94.0/91.0)*wristEnc.get()+94;  //Calculate the current angle
    if(Math.abs(wristSpeed) >= 0.05) {  //If the manip wants to manually move the wrist...
      wrist.set(wristSpeed);  //Set wrist 1
      wrist2.set(wristSpeed); //Set wrist 2
      wristSetPoint = wristPosition;  //Get the current position
    } else {  //Else (if PID is activated)...
      wristSpeed = (wristSetPoint - wristPosition) * wristKP; //Calculate the speed based off of the error
      wrist.set(wristSpeed);  //Set wrist1
      wrist2.set(wristSpeed); //Set wrist2
      SmartDashboard.putNumber("Wrist Speed", wristSpeed);  //Publish the speed
    }

    int currentPOVState = manip.getPOV();//Get the manip POV
    if(currentPOVState > lastPOVState) {//If the current POV angle is different than the old one...
      if(currentPOVState == 0) {//if the POV is up...
        servoTime = System.currentTimeMillis();//Record the time
        down = true;//Active the servo down boolean
        rightServo.set(0);//Set the right servo to up
      } else if(currentPOVState == 180) {//if the POV is down...
        servoTime = System.currentTimeMillis();//Record the time
        down = false;//Deactivate the servo down boolean
        rightServo.set(1);//Set the right servo to down
      }
    }
    if(!down && System.currentTimeMillis() - servoTime >= 300) {//If it has been 300ms since the POV was pushed down...
      leftServo.set(0);//Set the left servo to down
    }
    if(!down && System.currentTimeMillis() - servoTime >= 600 && System.currentTimeMillis() - servoTime < 1000) {//If it has been 600ms since the POV was pushed down...
      puncher.set(true);//Activate the punchers
    }
    if(!down && System.currentTimeMillis() - servoTime >= 1000) {//If it has been 1000ms since the POV was pushed down...
      puncher.set(false);//Deactivate the punchers
    }
    if(down && System.currentTimeMillis() - servoTime >= 300) {//If it has been 300ms since the POV was pushed up...
      leftServo.set(1);//Set the left servo to up
    }
    lastPOVState = currentPOVState;//Reset the last POV value

    if (manip.getRawAxis(2) >= 0.25) {//If left trigger is pressed...
      intake.set(-0.6);//Move the intake out
    } else if (manip.getRawAxis(3) >= 0.25) {//If right trigger is pressed...
      intake.set(0.6); //Move the intake motor in.
    } else {//If nothing is pressed...
      intake.set(0);//Set the intake to stop
    }

    if(manip.getRawButton(6)) {
      backLift.set(out);
    } else {
      backLift.set(in);
    }
    if(manip.getRawButton(5)) {
      frontLift.set(out);
    } else {
      frontLift.set(in);
    }

    read();//Read from sensors
  }

  private void read() {
    SmartDashboard.putNumber("Center-x", limelight.getX());
    SmartDashboard.putNumber("NavX", navx.getAngle());
    SmartDashboard.putNumber("Center-y", limelight.getY());
    SmartDashboard.putNumber("Height", limelight.getHeight());
    SmartDashboard.putNumber("Pressure", pressureSensor.get());
    SmartDashboard.putNumber("Elevator Speed 2", elevator.get());
    SmartDashboard.putNumber("Elevator Height", elevatorEnc.get());
    SmartDashboard.putNumber("WristAngle", (-94.0/91.0)*wristEnc.get()+94);
    SmartDashboard.putNumber("Lift Set Point", liftSetPoint);
  }

  @Override
  public void testPeriodic() {
  }
}