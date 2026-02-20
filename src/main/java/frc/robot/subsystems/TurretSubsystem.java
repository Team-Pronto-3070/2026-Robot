package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX mainShooterMotor = new TalonFX(Constants.Turret.mainShooterMotorID);
    private final TalonFX hoodShooterMotor = new TalonFX(Constants.Turret.hoodShooterMotorID);

    private final TalonFX turretMotor = new TalonFX(Constants.Turret.turretMotorID);

    private final TalonFXSimState turretMotorSimState = turretMotor.getSimState();

    // For a simple mechanism (e.g., an arm or elevator)
    private final DCMotorSim turretMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), // motor model
                    0.001, // moment of inertia (kg·m²) — tune this
                    1.0 // gear ratio
            ),
            DCMotor.getKrakenX60(1));

    private Pose2d turretPose = new Pose2d();

    private Translation2d lastTurretTranslation = new Translation2d();
    private long lastUpdateTime = 0;

    private final Field2d field;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable turretStateTable = inst.getTable("TurretState");
    private final StructPublisher<Pose3d> turretPosePublisher = turretStateTable
            .getStructTopic("Robot Relative Pose", Pose3d.struct).publish();
    // private final DoublePublisher turret =
    // driveStateTable.getDoubleTopic("Timestamp").publish();

    public TurretSubsystem(Field2d field) {
        this.field = field;

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // talonFXConfigs.MotorOutput.Inverted =
        // InvertedValue.CounterClockwise_Positive;

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity
        // of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of
        // 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        turretMotor.getConfigurator().apply(talonFXConfigs);

        // turretMotor.setPosition(0.5 * Constants.Turret.turretBeltRatio);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Motor Actual (deg)",
                turretMotor.getPosition().getValue().div(Constants.Turret.turretBeltRatio).in(Degrees));
        turretPosePublisher.set(new Pose3d(
                new Translation3d(Constants.Turret.turretToRobot.getX(), Constants.Turret.turretToRobot.getY(), 0),
                new Rotation3d(0, 0,
                        turretMotorSim.getAngularPositionRad() / Constants.Turret.turretBeltRatio)));
    }

    @Override
    public void simulationPeriodic() {
        // Supply battery voltage to the sim state
        turretMotorSim.setInputVoltage(RobotController.getBatteryVoltage());

        // Get the motor voltage output and feed it to the physics sim
        double motorVoltage = turretMotorSimState.getMotorVoltage();
        turretMotorSim.setInputVoltage(motorVoltage);

        // Step the physics simulation (20ms loop)
        turretMotorSim.update(0.020);

        // Push simulated position/velocity back to the TalonFX sim state
        turretMotorSimState.setRawRotorPosition(
                turretMotorSim.getAngularPositionRotations());
        turretMotorSimState.setRotorVelocity(
                turretMotorSim.getAngularVelocityRPM() / 60.0 // convert to rot/s
        );
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
                .minus(new Translation3d(deltaTranslation.getX(), deltaTranslation.getY(), 0)
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

        targetShooterAngle += Math.PI; // add 180˚ so that 0˚ is directly forwards relative to the robot
        // targetShooterAngle *= -1;

        // if (targetShooterAngle > Constants.Turret.clockwiseStop) {
        // targetShooterAngle = Math.copySign(Constants.Turret.maxShooterAngle,
        // targetShooterAngle);
        // }

        /*
         * Calculate desired shooter speed based on distance to target by interpolating
         * through calibrated values
         */

        double distanceToTarget = Math.hypot(turret.getX() - target.getX(), turret.getY() - target.getY());

        double targetShooterSpeed = calculateSpeedForDistance(distanceToTarget);

        setShooterHeading(targetShooterAngle);
        // setShooterSpeed(targetShooterSpeed);

        // SmartDashboard.putNumber("Shooter Target Heading (deg)", targetShooterAngle *
        // 180 / Math.PI);
        SmartDashboard.putNumber("Shooter Target Distance (m)", distanceToTarget);
        SmartDashboard.putNumber("Shooter Target Velocity (rpm)", targetShooterSpeed);

        field.getObject("Shooter").setPose(turretPose);

        field.getObject("Target").setPose(new Pose2d(target.toTranslation2d(), new Rotation2d()));

        // Trajectory autoLine = TrajectoryGenerator.generateTrajectory(
        // new Pose2d(lineX.in(Meters), 0, Rotation2d.fromDegrees(90)),
        // List.of(),
        // new Pose2d(lineX.in(Meters), Constants.fieldHeight.in(Meters),
        // Rotation2d.fromDegrees(90)),
        // new TrajectoryConfig(1, 1));

        // field.getObject("Neutral Zone Line").setTrajectory(autoLine);
    }

    public Command calibrateHeading() {
        // return this.run(() -> {
        // turretMotor.stopMotor();
        // turretMotor.set(0.05);
        // // System.out.println("Calibrating Turret");
        // }).until(() -> turretMotor.getTorqueCurrent().getValueAsDouble() > 22.0)
        // .andThen(this.runOnce(() -> {
        // // System.out.println("Turret Current Limit Reached");
        // turretMotor.stopMotor();
        // double heading = 170 * (Math.PI / 180);
        // // turretMotor.setPosition(((heading / (2 * Math.PI))) *
        // Constants.Turret.turretBeltRatio);
        // turretMotor.setPosition(0 * Constants.Turret.turretBeltRatio);
        // }));

        return this.runOnce(() -> {
            double heading = Constants.Turret.clockwiseStop.in(Radians);

            // Convert heading in radians into rotations, then get motor rotations
            turretMotor.setPosition(((heading / (2 * Math.PI))) *
                    Constants.Turret.turretBeltRatio);
        });
    }

    // Set the shooter to a target heading in radians relative to the robot (0rad is
    // straight, + is counter-clockwise)
    public void setShooterHeading(double heading) {
        if (heading > Math.PI)
            heading -= 2 * Math.PI;
        else if (heading < -Math.PI)
            heading += 2 * Math.PI;

        if (heading > Constants.Turret.clockwiseStop.in(Radians))
            heading = Constants.Turret.clockwiseStop.in(Radians);
        else if (heading < -Constants.Turret.counterclockwiseStop.in(Radians))
            heading = -Constants.Turret.counterclockwiseStop.in(Radians);

        final MotionMagicVoltage m_request = new MotionMagicVoltage(
                ((heading / (2 * Math.PI))) * Constants.Turret.turretBeltRatio);

        turretMotor.setControl(m_request);

        SmartDashboard.putNumber("Turret Target (deg)", heading * 180 / Math.PI);
    }

    public void setShooterSpeed(double speed) {
        mainShooterMotor.set(-speed);
        hoodShooterMotor.set(-speed * Constants.Turret.shooterRatio);
    }
}
