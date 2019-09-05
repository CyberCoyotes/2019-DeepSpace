package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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
  DoubleSolenoid leftFlipper = new DoubleSolenoid(5, 2);
  DoubleSolenoid rightFlipper = new DoubleSolenoid (6, 1);
  Solenoid puncher = new Solenoid(0);

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
  double turnKP = -0.09;//0.05

  final double liftHatchOffset = 14346;
  final double liftOffset = 3000;
  double previous = 0;
  final double hatch1 = 0;//TO DO
  final double hatch2 = 26896;
  final double port1 = 5464;
  final double port2 = 30000;

  long servoTime = Long.MAX_VALUE;
  int lastPOVState = -1;
  boolean down = true;
  boolean wristDown = true;
  boolean wasAuton = false;
  boolean servoToggle = false;

  @Override
  public void robotInit() {
    mainDrive.setSafetyEnabled(false);
    elevator1.setInverted(true);
    wrist2.setInverted(false);
    wrist.setInverted(true);
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
      limelight.setPipeline(0);
      wristSetPoint = (-94.0/91.0)*wristEnc.get()+94;
      wristSetPoint = 85;
      elevatorEnc.reset();
      liftSetPoint = elevatorEnc.get();
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
      limelight.setPipeline(1);
    }
    if((Math.abs(y) >= 0.15 || Math.abs(rot) >= 0.15) && !driver.getRawButton(1)) {
      mainDrive.arcadeDrive(y, rot);
    } else if(driver.getRawButton(1) && limelight.hasValidTarget() && elevatorEnc.get() > 23000) {
      double rSpeed = limelight.getX()*turnKP;
      mainDrive.arcadeDrive(y, rSpeed);
    } else {
      mainDrive.arcadeDrive(0, 0);
    }
    if(driver.getRawButtonReleased(1)) {
      limelight.setPipeline(0);
      liftSetPoint = previous;
    }

    if(driver.getRawButton(2)) {
      shifter.set(out);
    } else {
      shifter.set(in);
    }

    double liftSpeed = manip.getRawAxis(1); //Get the manual lift speed
    if(liftSpeed < 0) { //If the manual speed is negative...
      liftSpeed*=0.9; //Limit the up speed
    } else {  //Else...
      liftSpeed*=0.5; //Limit the down speed
    }
    if(driver.getRawButtonPressed(12)) {  //If the driver pressed button 12...
      liftSetPoint = hatch1;  //Set the base height to hatch1
    }
    if(driver.getRawButtonPressed(11)) {  //If the driver pressed button 12...
      liftSetPoint = port1;  //Set the base height to port1
      }
    if(driver.getRawButtonPressed(10)) {  //If the driver presses button 10...
      liftSetPoint = hatch2;  //Set the base height to hatch2
    }
    if(driver.getRawButtonPressed(9)) {  //If the driver presses button 10...
      liftSetPoint = port2;  //Set the base height to port2
    }
    if(Math.abs(liftSpeed) >= 0.05 && elevatorEnc.get() > 250) { //If the manip wants to manual control the elevator...
      elevator.set(liftSpeed);  //Set the elevator speed
      liftSetPoint = elevatorEnc.get(); //Set the PID setpoint to the current reading
    } else {  //Else (if the PID is activated)
      if(elevatorEnc.get() < 250) {
        liftSetPoint = 750;
      }
      liftSpeed = (liftSetPoint - elevatorEnc.get()) * liftKP;  //Calculate the lift speed based on error
      if(liftSpeed > 0.4) { //Limit the down speed
        liftSpeed = 0.4;
      } else if(liftSpeed < -0.75) {  //Limit the up speed
        liftSpeed = -0.75;
      }
      elevator.set(liftSpeed);  //Set the elevator
    }
    if(driver.getRawButtonPressed(5)) { //If button 3 is pressed...
      liftSetPoint+=liftOffset; //Add 2 inches
    }
    if(driver.getRawButtonReleased(5)) {  //If button 3 is released...
      liftSetPoint-=liftOffset; //Remove 2 inches
    }
    if(driver.getRawButtonPressed(6)) { //If button 4 is pressed...
      liftSetPoint-=liftOffset; //Remove 2 inches
    }
    if(driver.getRawButtonReleased(6)) {  //If button 4 is released
      liftSetPoint+=liftOffset; //Add 2 inches
    }
    if(driver.getRawButton(3)) {
      leftFlipper.set(out);
    } else {
      leftFlipper.set(in); 
    }
    if(driver.getRawButton(4)) {
      rightFlipper.set(out);
    } else {
      rightFlipper.set(in);
    }

    double wristSpeed = manip.getRawAxis(5);  //Get the manual wrist speed
    wristSpeed = wristSpeed > 0 ? wristSpeed*=0.35 : wristSpeed; //Limit the down speed
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
    if(manip.getRawButtonPressed(2)) {
      wristSetPoint = 30;//Was 30
    }

    if(manip.getRawButton(1)) {
      wristSetPoint = 90;
    }

    /*****
    int currentPOVState = manip.getPOV();//Get the manip POV
    if(currentPOVState > lastPOVState) {//If the current POV angle is different than the old one...
      if(currentPOVState == 180) {//if the POV is down...
        servoTime = System.currentTimeMillis();//Record the time
        down = false;//Deactivate the servo down boolean
        //wristSetPoint = 55;
      }
    }
    if(!down && System.currentTimeMillis() - servoTime > 750) {
      intake.set(-0.2);
    }
    if(!down && System.currentTimeMillis() - servoTime >= 1500) {//If it has been 300ms since the POV was pushed down...
      //intake.set(0);
      //wristSetPoint-=5;/////
      puncher.set(true);
    }
    if(!down && System.currentTimeMillis() - servoTime >= 2000) {//If it has been 600ms since the POV was pushed down...
      puncher.set(false);
      intake.set(0);
      down = true;
    }
    lastPOVState = currentPOVState;//Reset the last POV value
*/


/*
int currentPOVState = manip.getPOV();//Get the manip POV
    if(currentPOVState > lastPOVState) {//If the current POV angle is different than the old one...
      if(currentPOVState == 180) {//if the POV is down...
        servoTime = System.currentTimeMillis();//Record the time
        down = false;//Deactivate the servo down boolean
        intake.set(-0.25);
      }
    }
    if(!down && System.currentTimeMillis() - servoTime >= 500) {//If it has been 300ms since the POV was pushed down...
      //intake.set(0);
      //wristSetPoint-=5;/////
      puncher.set(true);
    }
    if(!down && System.currentTimeMillis() - servoTime >= 1250) {//If it has been 600ms since the POV was pushed down...
      puncher.set(false);
      intake.set(0);
      down = true;
    }
    lastPOVState = currentPOVState;//Reset the last POV value*/


    
    if (manip.getRawAxis(2) >= 0.25 && down) {//If left trigger is pressed...
      intake.set(-0.6);//Move the intake out
    } else if (manip.getRawAxis(3) >= 0.25 && down) {//If right trigger is pressed...
      intake.set(0.6); //Move the intake motor in.
    } else if(manip.getRawButton(5)) {
      intake.set(-0.3);
    } else {
      intake.set(0);
    }     
    
    puncher.set(manip.getRawButton(6));
      //else if(down) {//If nothing is pressed...
      //intake.set(0);//Set the intake to stop
    //}
    
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