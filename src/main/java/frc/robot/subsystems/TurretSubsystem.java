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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX mainShooterMotor = new TalonFX(Constants.Turret.mainShooterMotorID);
    private final TalonFX hoodShooterMotor = new TalonFX(Constants.Turret.hoodShooterMotorID);

    private final TalonFX turretMotor = new TalonFX(Constants.Turret.turretMotorID);

    private Pose2d turretPose = new Pose2d();

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable driveStateTable = inst.getTable("TurretState");
    private final StructPublisher<Pose2d> turretPoseState = driveStateTable.getStructTopic("Pose", Pose2d.struct)
            .publish();

    /* Turret pose for field image */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("TurretState").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    public TurretSubsystem() {

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

        double robotAngle = pose.getRotation().getRadians();

        double angleToTarget = Math.atan2(pose.getY() - target.getY(), pose.getX() - target.getX());
        double distanceToTarget = Math.hypot(pose.getX() - target.getX(), pose.getY() - target.getY());

        double targetShooterAngle = angleToTarget - robotAngle;

        setShooterHeading(targetShooterAngle);

        turretPose = new Pose2d(
                new Translation2d(
                        Constants.Turret.turretToRobot.getX(),
                        Constants.Turret.turretToRobot.getY())
                        .rotateBy(pose.getRotation()).plus(pose.getTranslation()),
                pose.getRotation().plus(new Rotation2d(targetShooterAngle + Math.PI)));

        turretPoseState.set(turretPose);

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = turretPose.getX();
        m_poseArray[1] = turretPose.getY();
        m_poseArray[2] = turretPose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);
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
