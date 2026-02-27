package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

public class AutonomousSubsystem extends SubsystemBase {

        private final CommandSwerveDrivetrain drivetrain;

        private SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

        private final Field2d field;

        private final SendableChooser<Command> autoChooser;

        private boolean selfDriving = false;

        public AutonomousSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field) {
                this.drivetrain = drivetrain;
                this.field = field;

                // Load the RobotConfig from the GUI settings. You should probably
                // store this in your Constants file
                RobotConfig config = null;
                try {
                        config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        // Handle exception as needed
                        e.printStackTrace();
                }

                // Configure AutoBuilder last
                AutoBuilder.configure(
                                () -> drivetrain.getState().Pose, // Robot pose supplier
                                (newPose) -> drivetrain.resetPose(newPose), // Method to reset odometry (will be called
                                                                            // if your
                                // auto has a starting pose)
                                () -> drivetrain.getKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates), // ChassisSpeeds
                                // supplier.
                                // MUST BE ROBOT
                                // RELATIVE
                                (speeds, feedforwards) -> drivetrain.setControl(drive.withSpeeds(speeds)), // Method
                                                                                                           // that will
                                // drive the robot given
                                // ROBOT
                                // RELATIVE ChassisSpeeds. Also optionally outputs
                                // individual module feedforwards
                                new PPHolonomicDriveController( // PPHolonomicController is the built in path following
                                                                // controller for
                                                                // holonomic drive trains
                                                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                                new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                                ),
                                config, // The robot configuration
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                                return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                },
                                this // Reference to this subsystem to set requirements
                );

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private String autoName, newAutoName;
        private boolean redAlliance = false, lastAlliance = false;
        private List<Pose2d> poses = new ArrayList<Pose2d>();

        @Override
        public void periodic() {

                var optional = DriverStation.getAlliance();
                if (optional.isPresent()) {
                        if (optional.get() == Alliance.Red) {
                                redAlliance = true;
                        } else {
                                redAlliance = false;
                        }
                }

                newAutoName = autoChooser.getSelected().getName();
                if (autoName != newAutoName || lastAlliance != redAlliance) {
                        autoName = newAutoName;
                        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                                // System.out.println("Displaying " + autoName);
                                try {
                                        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto
                                                        .getPathGroupFromAutoFile(autoName);
                                        poses.clear();
                                        // ** Rotate Path on red side **//
                                        for (PathPlannerPath path : pathPlannerPaths) {
                                                poses.addAll(path.getAllPathPoints().stream().map(
                                                                point -> redAlliance
                                                                                ? new Pose2d(Constants.fieldWidth.minus(
                                                                                                point.position.getMeasureX()),
                                                                                                Constants.fieldHeight
                                                                                                                .minus(point.position
                                                                                                                                .getMeasureY()),
                                                                                                new Rotation2d())
                                                                                : new Pose2d(point.position.getX(),
                                                                                                point.position.getY(),
                                                                                                new Rotation2d()))
                                                                .collect(Collectors.toList()));
                                        }

                                        field.getObject("Auto Path").setPoses(poses);
                                } catch (Exception e) {
                                        System.out.println("Auto Display Exception: " + e);
                                }
                        }
                }

                lastAlliance = redAlliance;
        }

        public ChassisSpeeds trenchInputAdjust(double inputX, double inputY, double rotationInput, boolean apply) {

                if (!apply) {
                        return new ChassisSpeeds(inputX, inputY, rotationInput);
                }

                Translation2d trench = Constants.Autonomous.redTrenchLeft;

                Pose2d pose = drivetrain.getState().Pose;

                double bumpHalfWidth = 0.5; // exact half of bump is 0.564

                if (pose.getMeasureX().lt(Constants.fieldWidth.div(2))) {
                        if (pose.getTranslation().getY() < Constants.fieldHeight.in(Meters) / 2)
                                trench = Constants.Autonomous.blueTrenchRight;
                        else
                                trench = Constants.Autonomous.blueTrenchLeft;
                } else {
                        if (pose.getTranslation().getY() < Constants.fieldHeight.in(Meters) / 2)
                                trench = Constants.Autonomous.redTrenchLeft;
                        else
                                trench = Constants.Autonomous.redTrenchRight;
                }

                double xDistance = Math.abs(pose.getX() - trench.getX());
                double yDistance = Math.abs(pose.getY() - trench.getY());

                double yProximity = Math.min(1.0, Math.max(Constants.Autonomous.minForce,
                                (yDistance - bumpHalfWidth) / Constants.Autonomous.yActivationRange));

                if (yDistance < Constants.Autonomous.yActivationRange
                                && xDistance < Constants.Autonomous.xActivationRange) {

                        boolean pauseX = false;

                        // Blue side facing trenches
                        if (pose.getX() > trench.getX() - bumpHalfWidth && inputX > 0) {

                                pauseX = (xDistance / Constants.Autonomous.pauseRatio) < yDistance
                                                && yDistance > Constants.Autonomous.tolerance
                                                && xDistance > bumpHalfWidth && inputX > 0 && xDistance < 1.75;

                                double force = pauseX && Math.abs(inputY) < 0.1 ? Constants.Autonomous.maxForce
                                                : Constants.Autonomous.maxForce * (1 - Math.pow(1 - yProximity,
                                                                Constants.Autonomous.alignmentExponent));

                                // System.out.printf("inputY: %.2f, inputX: %.2f, force: %.3f%n", inputY,
                                // inputX, force);
                                if (pose.getY() + Constants.Autonomous.tolerance < trench.getY()) {
                                        inputY += -inputX * force;
                                } else if (pose.getY() - Constants.Autonomous.tolerance > trench.getY()) {
                                        inputY += inputX * force;
                                }

                                Angle targetRotation = Radians
                                                .of(Math.round(pose.getRotation().getRadians() / (Math.PI)) * Math.PI);

                                if (pose.getRotation().getDegrees()
                                                + Constants.Autonomous.rotationTolerance < targetRotation.in(Degrees)) {
                                        rotationInput = 1;
                                        // rotationInput = 1 * Math.max(Math.pow(pose.getRotation().getDegrees() / 180,
                                        // 4), 0.5);
                                } else if (pose.getRotation().getDegrees()
                                                - Constants.Autonomous.rotationTolerance > targetRotation.in(Degrees)) {
                                        rotationInput = -1;
                                        // rotationInput = -1 *
                                        // Math.max(Math.pow((Math.abs(pose.getRotation().getDegrees()) / 180), 4),
                                        // 0.5);
                                }

                                // Red side facing trenches
                        } else if (trench.getX() + bumpHalfWidth > pose.getX() && inputX < 0) {

                                pauseX = (xDistance / Constants.Autonomous.pauseRatio) < yDistance
                                                && yDistance > Constants.Autonomous.tolerance
                                                && xDistance > bumpHalfWidth && inputX < 0 && xDistance < 1.75;

                                double force = pauseX && Math.abs(inputY) < 0.1 ? Constants.Autonomous.maxForce
                                                : Constants.Autonomous.maxForce * (1 - Math.pow(1 - yProximity,
                                                                Constants.Autonomous.alignmentExponent));

                                if (pose.getY() + Constants.Autonomous.tolerance < trench.getY()) {
                                        inputY += inputX * force;
                                } else if (pose.getY() - Constants.Autonomous.tolerance > trench.getY()) {
                                        inputY += -inputX * force;

                                }
                                Angle targetRotation = Radians
                                                .of(Math.round(pose.getRotation().getRadians() / (Math.PI)) * Math.PI);

                                if (pose.getRotation().getDegrees()
                                                + Constants.Autonomous.rotationTolerance < targetRotation.in(Degrees)) {
                                        rotationInput = 1;
                                        // rotationInput = 1 * Math.max(Math.pow(pose.getRotation().getDegrees() / 180,
                                        // 4), 0.5);
                                } else if (pose.getRotation().getDegrees()
                                                - Constants.Autonomous.rotationTolerance > targetRotation.in(Degrees)) {
                                        rotationInput = -1;
                                        // rotationInput = -1 *
                                        // Math.max(Math.pow((Math.abs(pose.getRotation().getDegrees()) / 180), 4),
                                        // 0.5);
                                }
                        }

                        if (pauseX) {
                                inputX = 0;
                        }

                        inputY = MathUtil.clamp(inputY, -1.0, 1.0);

                }

                return new ChassisSpeeds(inputX, inputY, rotationInput);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public boolean isSelfDriving() {
                return selfDriving;
        }
}