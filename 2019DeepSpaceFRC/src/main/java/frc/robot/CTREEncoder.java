package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CTREEncoder implements PIDSource {

    WPI_TalonSRX talon;
    public CTREEncoder(int id) {
        talon = new WPI_TalonSRX(id);
    }

    public int get() {
        return talon.getSelectedSensorPosition();
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