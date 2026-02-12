package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.lang.annotation.Target;
import java.sql.Driver;
import java.util.List;
import java.util.TreeMap;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
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

    private Translation2d lastTurretTranslation = new Translation2d();
    private long lastUpdateTime = 0;

    private final Field2d field;

    public TurretSubsystem(Field2d field) {
        this.field = field;
    }

    @Override
    public void periodic() {

    }

    private double calculateSpeedForDistance(double targetDistance) {
        TreeMap<Double, Double> treeMap = Constants.Turret.speedTreeMap;

        // Handle exact matches
        if (treeMap.containsKey(targetDistance)) {
            return treeMap.get(targetDistance);
        }

        // Get the distances immediately below and above target
        Double lowerDistance = treeMap.floorKey(targetDistance);
        Double upperDistance = treeMap.ceilingKey(targetDistance);

        // Handle out-of-range cases
        if (lowerDistance == null) {
            return treeMap.get(upperDistance); // Use closest
        }
        if (upperDistance == null) {
            return treeMap.get(lowerDistance); // Use closest
        }

        // Linear interpolation
        double lowerSpeed = treeMap.get(lowerDistance);
        double upperSpeed = treeMap.get(upperDistance);

        double ratio = (targetDistance - lowerDistance) / (upperDistance - lowerDistance);
        return lowerSpeed + ratio * (upperSpeed - lowerSpeed);
    }

    // Give the turret a new robot position to calculate speeds for
    public void update(Pose2d pose) {

        // Calculate delta time in seconds
        double deltaTime = (System.nanoTime() - lastUpdateTime) / 1e9;
        lastUpdateTime = System.nanoTime();

        /*
         * Calculate the shooter's target to aim at based off of current position and
         * alliance
         */
        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
                : DriverStation.Alliance.Blue;

        Distance lineX = alliance.equals(Alliance.Blue) ? Constants.Turret.blueLineX : Constants.Turret.redLineX;

        Translation3d target = alliance.equals(Alliance.Blue) ? Constants.Turret.blueHub : Constants.Turret.redHub;

        if (alliance.equals(Alliance.Blue)) {
            if (pose.getTranslation().getX() > lineX.in(Meters)) {
                if (pose.getTranslation().getY() < Constants.fieldHeight.in(Meters) / 2)
                    target = Constants.Turret.blueFerryRight;
                else
                    target = Constants.Turret.blueFerryLeft;
            }
        } else {
            if (pose.getTranslation().getX() < lineX.in(Meters)) {
                if (pose.getTranslation().getY() < Constants.fieldHeight.in(Meters) / 2)
                    target = Constants.Turret.redFerryLeft;
                else
                    target = Constants.Turret.redFerryRight;
            }
        }

        /*
         * Calculate the current position of the turret on the field using the robot
         * position. Also calculate the delta position to determine field relative
         * velocity
         */
        Translation2d turret = new Translation2d(
                Constants.Turret.turretToRobot.getX(),
                Constants.Turret.turretToRobot.getY())
                .rotateBy(pose.getRotation()).plus(pose.getTranslation());

        Translation2d deltaTranslation = turret.minus(lastTurretTranslation).div(deltaTime);
        lastTurretTranslation = turret;

        target = target
                .minus(new Translation3d(deltaTranslation.getMeasureX(), deltaTranslation.getMeasureY(), Inches.of(0))
                        .times(Constants.Turret.shootOnTheMoveScale));

        /*
         * Calculate the actual angles for the shooter
         */
        double angleToTarget = Math.atan2(turret.getY() - target.getY(), turret.getX() - target.getX());

        double robotAngle = pose.getRotation().getRadians();
        double targetShooterAngle = angleToTarget - robotAngle;

        turretPose = new Pose2d(
                turret,
                pose.getRotation().plus(new Rotation2d(targetShooterAngle + Math.PI)));

        targetShooterAngle += Math.PI; // subtract 180˚ so that 0˚ is directly forwards relative to the robot

        /*
         * Calculate desired shooter speed based on distance to target by interpolating
         * through calibrated values
         */

        double distanceToTarget = Math.hypot(turret.getX() - target.getX(), turret.getY() - target.getY());

        double targetShooterSpeed = calculateSpeedForDistance(distanceToTarget);

        setShooterHeading(targetShooterAngle);
        // setShooterSpeed(targetShooterSpeed);

        SmartDashboard.putNumber("Shooter Target Heading (˚)", targetShooterAngle * 180 / Math.PI);
        SmartDashboard.putNumber("Shooter Target Distance (m)", distanceToTarget);
        SmartDashboard.putNumber("Shooter Target Velocity (rpm)", targetShooterSpeed);

        field.getObject("shooter").setPose(turretPose);

        field.getObject("target").setPose(new Pose2d(target.toTranslation2d(), new Rotation2d()));

        Trajectory autoLine = TrajectoryGenerator.generateTrajectory(
                new Pose2d(lineX.in(Meters), 0, Rotation2d.fromDegrees(90)),
                List.of(),
                new Pose2d(lineX.in(Meters), Constants.fieldHeight.in(Meters),
                        Rotation2d.fromDegrees(90)),
                new TrajectoryConfig(1, 1));

        field.getObject("line").setTrajectory(autoLine);
    }

    // Set the shooter to a target heading in radians relative to the robot (0rad is
    // straight, + is counter-clockwise)
    public void setShooterHeading(double heading) {
        // TODO: Implement shooter rotation logic (Motion Magic?)
    }

    public void setShooterSpeed(double speed) {
        mainShooterMotor.set(-speed);
        hoodShooterMotor.set(-speed * Constants.Turret.shooterRatio);
    }
}
