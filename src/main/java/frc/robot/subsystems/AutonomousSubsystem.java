package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

public class AutonomousSubsystem extends SubsystemBase {

        private final CommandSwerveDrivetrain drivetrain;

        private final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withHeadingPID(4, 0, 0); /* tune this for your robot! */

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private static final APConstraints kConstraints = new APConstraints()
                        .withAcceleration(15.0)
                        .withJerk(5.0);

        private static final APProfile kProfile = new APProfile(kConstraints)
                        .withErrorXY(Centimeters.of(5))
                        .withErrorTheta(Degrees.of(0.5))
                        .withBeelineRadius(Centimeters.of(8));

        public static final Autopilot kAutopilot = new Autopilot(kProfile);

        private APTarget target = new APTarget(new Pose2d(Constants.Autonomous.redLeftTrench, new Rotation2d()))
                        .withEntryAngle(Rotation2d.kZero);

        private final Field2d field;

        private boolean selfDriving = false;

        public AutonomousSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field) {
                this.drivetrain = drivetrain;
                this.field = field;
        }

        @Override
        public void periodic() {
                Pose2d pose = drivetrain.getState().Pose;

                target = new APTarget(new Pose2d(Constants.Autonomous.redLeftTrench, new Rotation2d(Math.round(pose.getRotation().getRadians() / (Math.PI / 2)) * (Math.PI / 2))))
                                .withEntryAngle(pose.getX() > Constants.Autonomous.redLeftTrench.getX()
                                                ? Rotation2d.k180deg
                                                : Rotation2d.kZero).withVelocity(5.0);
        }

        @Override
        public void simulationPeriodic() {

        }

        public boolean isSelfDriving() {
                return selfDriving;
        }

        public Command trench() {
                return drivetrain.run(() -> {
                        selfDriving = true;

                        ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;
                        Pose2d pose = drivetrain.getState().Pose;

                        APResult output = kAutopilot.calculate(pose, robotRelativeSpeeds, target);

                        /* these speeds are field relative */
                        LinearVelocity veloX = output.vx();
                        LinearVelocity veloY = output.vy();
                        Rotation2d headingReference = output.targetAngle();

                        drivetrain.setControl(request
                                        .withVelocityX(veloX)
                                        .withVelocityY(veloY)
                                        .withTargetDirection(headingReference));
                })
                                .until(() -> kAutopilot.atTarget(drivetrain.getState().Pose, target))
                                // .andThen(() -> {
                                //         drivetrain.applyRequest(() -> drive
                                //                         .withVelocityX(3.0)
                                //                         .withVelocityY(0.0)
                                //                         .withRotationalRate(0.0))
                                //                         .withTimeout(5.0);
                                // }, drivetrain)
                                .finallyDo(() -> {
                                        selfDriving = false;
                                });
        }
}
