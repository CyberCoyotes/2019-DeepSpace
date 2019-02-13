package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CTREEncoder implements PIDSource {

    WPI_TalonSRX talon;
    boolean invert;

    public CTREEncoder(int id, boolean invert) {
        talon = new WPI_TalonSRX(id);
        talon.getSensorCollection();
        this.invert = invert;
    }

    public CTREEncoder(WPI_TalonSRX _talon, boolean invert) {
        talon = _talon;
        talon.getSensorCollection();
        this.invert = invert;
    }

    public int get() {
        return invert ? -talon.getSelectedSensorPosition() : talon.getSelectedSensorPosition();
    }

    public void setPosition(int pos) {
        talon.setSelectedSensorPosition(pos);
    }

    @Override
    public double pidGet() {
        return get();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        
    }
}