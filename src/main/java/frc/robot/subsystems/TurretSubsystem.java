package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX mainShooterMotor = new TalonFX(Constants.Turret.mainShooterMotorID);
    private final TalonFX hoodShooterMotor = new TalonFX(Constants.Turret.hoodShooterMotorID);

    private final TalonFX turretMotor = new TalonFX(Constants.Turret.turretMotorID);

    public TurretSubsystem() {

    }

    @Override
    public void periodic() {

    }

    // Give the turret a new robot position to calculate speeds for
    public void update(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
                : DriverStation.Alliance.Blue;

        Translation3d target = alliance.equals(Alliance.Blue) ? Constants.Turret.blueHub : Constants.Turret.redHub;

        double robotAngle = pose.getRotation().getRadians();
        double angleToTarget = Math.atan2(pose.getY() - target.getY(), pose.getY() - target.getY());

        double targetShooterAngle = angleToTarget - robotAngle;

        setShooterHeading(targetShooterAngle);
    }

    // Set the shooter to a target heading in radians relative to the robot (0rad is straight, + is clockwise) 
    public void setShooterHeading(double heading) {
        //TODO: Implement shooter rotation logic (Motion Magic?)
    }

    public void setShooterSpeed(double speed) {
        mainShooterMotor.set(-speed);
        hoodShooterMotor.set(-speed * Constants.Turret.shooterRatio);
    }
}
