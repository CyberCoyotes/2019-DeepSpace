/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable limelight;//Table for the limelight
    NetworkTableEntry tx;//Table for the x-coordinate
    NetworkTableEntry ty;//Table for the y-coordinate
    NetworkTableEntry ta;//Table for the area
    NetworkTableEntry ts;//Table for the skew
    NetworkTableEntry tv;//Table to see if there are valid targets
    NetworkTableEntry tl;//Table for latency
    NetworkTableEntry ledMode;//Table to set blinking leds
    NetworkTableEntry camMode;//Table to set camera mode
    NetworkTableEntry pipeline;//Table to switch pipelines

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");//Instantiate the tables
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        ts = limelight.getEntry("ts");
        tv = limelight.getEntry("tv");
        tl = limelight.getEntry("tl");
        ledMode = limelight.getEntry("ledMode");
        camMode = limelight.getEntry("camMode");
        pipeline = limelight.getEntry("pipeline");
    }

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public double getSkew() {
        return ts.getDouble(0.0);
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0) == 1.0;
    }

    public double getLatency() {
        return tl.getDouble(0.0);
    }

    public void setPipeline(int id) {
        pipeline.setNumber(id);
    }

    public void setLedMode(int mode) {
        ledMode.setNumber(mode);
    }

    public void setCamMode(int mode) {
        camMode.setNumber(mode);
    }
}
