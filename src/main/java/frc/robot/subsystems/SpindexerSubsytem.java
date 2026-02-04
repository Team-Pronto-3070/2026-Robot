package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsytem extends SubsystemBase{

    private final TalonFX spindexerMotor = new TalonFX(16);

    public SpindexerSubsytem(){

    }

    @Override
    public void periodic(){

    }

    public void spin() {
        spindexerMotor.set(0.1);
    }

    public void stop() {
        spindexerMotor.set(0);
    }

    @Override
    public void simulationPeriodic(){
        
    }
}
