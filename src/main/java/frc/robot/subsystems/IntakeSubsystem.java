package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);
    private final TalonFX liftingMotor = new TalonFX(Constants.Intake.liftingMotorID);

    public IntakeSubsystem(){

    }

    @Override
    public void periodic(){

    }

    public void intake() {
        intakeMotor.set(-0.4);
    }

    public void stop() {
        intakeMotor.set(0);
    }
}
