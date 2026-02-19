package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsytem extends SubsystemBase{

    private final TalonFX spindexerMotor = new TalonFX(Constants.Spindexer.spindexerMotorID);
    private final TalonFX indexerMotor = new TalonFX(Constants.Spindexer.indexerMotorID);

    public SpindexerSubsytem(){

    }

    @Override
    public void periodic(){

    }

    public void spin() {
        spindexerMotor.set(0.15);
        indexerMotor.set(0.3);
    }
    
    public void outtake() {
        spindexerMotor.set(-0.15);
        indexerMotor.set(-0.3);
    }

    public void stop() {
        spindexerMotor.set(0);
        indexerMotor.set(0);
    }

    @Override
    public void simulationPeriodic(){
        
    }
}
