// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SpindexerSubsytem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
        // kSpeedAt12Volts desired top speed
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10%
                                                                                                     // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        public final Telemetry logger = new Telemetry(MaxSpeed);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final AutonomousSubsystem autonomousSubsystem = new AutonomousSubsystem(drivetrain, logger.field);

        public final OI oi = new OI();

        public final CameraSubsystem frontCamera = new CameraSubsystem(Constants.Vision.FrontCamera.name,
                        Constants.Vision.FrontCamera.transform);

        public final SpindexerSubsytem spindexerSubsytem = new SpindexerSubsytem();

        public final TurretSubsystem turretSubsystem = new TurretSubsystem(logger.field);

        public RobotContainer() {
                if (!RobotBase.isReal()) { // if in the simulator
                        DriverStation.silenceJoystickConnectionWarning(true);
                }

                SmartDashboard.putData("Field", logger.field);

                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() ->
                                // Drive forward with negative y (forward)
                                drive.withVelocityX(oi.processed_drive_x.getAsDouble() * MaxSpeed)

                                                // Drive left with negative X (left)
                                                .withVelocityY(oi.processed_drive_y.getAsDouble() * MaxSpeed)
                                                // Drive counterclockwise with negative X(left)
                                                .withRotationalRate(oi.processed_drive_rot.getAsDouble()
                                                                * MaxAngularRate))
                                                .onlyIf(() -> !autonomousSubsystem.isSelfDriving()));

                frontCamera.setDefaultCommand(frontCamera.runOnce(() -> {
                        drivetrain.addVisionMeasurement(frontCamera.getEstimatedPose(),
                                        frontCamera.getEstimatedTimestamp());
                }).ignoringDisable(true));

                turretSubsystem.setDefaultCommand(
                                turretSubsystem.runOnce(() -> turretSubsystem.update(drivetrain.getState().Pose)));

                // Reset the field-centric heading on left bumper press.
                oi.gyroReset.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                oi.index.onTrue(spindexerSubsytem.runOnce(() -> spindexerSubsytem.spin()));
                oi.index.onFalse(spindexerSubsytem.runOnce(() -> spindexerSubsytem.stop()));

                oi.shoot.onTrue(turretSubsystem
                                .runOnce(() -> turretSubsystem.setShooterSpeed(1.0 / Constants.Turret.shooterRatio)));
                oi.shoot.onFalse(turretSubsystem.runOnce(() -> turretSubsystem.setShooterSpeed(0.0)));

                oi.trench.whileTrue(autonomousSubsystem.trench());

                drivetrain.registerTelemetry(logger::telemeterize);

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Then slowly drive forward (away from us) for 5 seconds.
                                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                // Finally idle for the rest of auton
                                drivetrain.applyRequest(() -> idle));
        }
}