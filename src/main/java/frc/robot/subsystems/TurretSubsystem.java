package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX mainShooterMotor = new TalonFX(Constants.Turret.mainShooterMotorID);
    private final TalonFX hoodShooterMotor = new TalonFX(Constants.Turret.hoodShooterMotorID);

    private final TalonFX turretMotor = new TalonFX(Constants.Turret.turretMotorID);

    private Pose2d turretPose = new Pose2d();

    private final Field2d field;

    public TurretSubsystem(Field2d field) {
        this.field = field;
    }

    @Override
    public void periodic() {

    }

    private final double[] m_poseArray = new double[3];

    // Give the turret a new robot position to calculate speeds for
    public void update(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
                : DriverStation.Alliance.Blue;

        Translation3d target = alliance.equals(Alliance.Blue) ? Constants.Turret.blueHub : Constants.Turret.redHub;

        Translation2d turret = new Translation2d(
                        Constants.Turret.turretToRobot.getX(),
                        Constants.Turret.turretToRobot.getY())
                        .rotateBy(pose.getRotation()).plus(pose.getTranslation());
        
        double angleToTarget = Math.atan2(turret.getY() - target.getY(), turret.getX() - target.getX());
        double distanceToTarget = Math.hypot(turret.getX() - target.getX(), turret.getY() - target.getY());
        
        double robotAngle = pose.getRotation().getRadians();
        double targetShooterAngle = angleToTarget - robotAngle;

        setShooterHeading(targetShooterAngle);

        turretPose = new Pose2d(
                new Translation2d(
                        Constants.Turret.turretToRobot.getX(),
                        Constants.Turret.turretToRobot.getY())
                        .rotateBy(pose.getRotation()).plus(pose.getTranslation()),
                pose.getRotation().plus(new Rotation2d(targetShooterAngle + Math.PI)));

        field.getObject("shooter").setPose(turretPose);

        field.getObject("target").setPose(new Pose2d(target.toTranslation2d(), new Rotation2d()));
    }

    // Set the shooter to a target heading in radians relative to the robot (0rad is
    // straight, + is clockwise)
    public void setShooterHeading(double heading) {
        // TODO: Implement shooter rotation logic (Motion Magic?)
    }

    public void setShooterSpeed(double speed) {
        mainShooterMotor.set(-speed);
        hoodShooterMotor.set(-speed * Constants.Turret.shooterRatio);
    }
}
