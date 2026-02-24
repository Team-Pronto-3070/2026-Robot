// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
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

        public final CameraSubsystem leftCamera = new CameraSubsystem(Constants.Vision.leftParams);
        public final CameraSubsystem frontLeftCamera = new CameraSubsystem(Constants.Vision.frontLeftParams);
        public final CameraSubsystem frontRightCamera = new CameraSubsystem(Constants.Vision.frontRightParams);
        public final CameraSubsystem rightCamera = new CameraSubsystem(Constants.Vision.rightParams);

        public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
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
                drivetrain.setDefaultCommand(drivetrain.run(() -> {
                        double driveX = oi.processed_drive_x.getAsDouble() * MaxSpeed;
                        double driveY = oi.processed_drive_y.getAsDouble() * MaxSpeed;
                        double rotationalRate = oi.processed_drive_rot.getAsDouble() * MaxAngularRate;

                        drivetrain.applyRequest(() -> drive
                                        .withVelocityX(driveX)
                                        .withVelocityY(driveY)
                                        .withRotationalRate(rotationalRate))
                                        .execute();
                }));

                leftCamera.setDefaultCommand(leftCamera.run(() -> {
                        leftCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds,
                                                leftCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                frontLeftCamera.setDefaultCommand(frontLeftCamera.run(() -> {
                        frontLeftCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds,
                                                frontLeftCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                frontRightCamera.setDefaultCommand(frontRightCamera.run(() -> {
                        frontRightCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds,
                                                frontRightCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                rightCamera.setDefaultCommand(rightCamera.run(() -> {
                        rightCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds,
                                                rightCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                turretSubsystem.setDefaultCommand(
                                turretSubsystem.runOnce(() -> turretSubsystem.update(drivetrain.getState().Pose))
                                                .ignoringDisable(true));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Reset the field-centric heading on left bumper press.
                oi.gyroReset.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                oi.index.onTrue(spindexerSubsytem.runOnce(() -> spindexerSubsytem.spin()));
                oi.index.onFalse(spindexerSubsytem.runOnce(() -> spindexerSubsytem.stop()));

                oi.outtake.onTrue(spindexerSubsytem.runOnce(() -> spindexerSubsytem.outtake()));
                oi.outtake.onFalse(spindexerSubsytem.runOnce(() -> spindexerSubsytem.stop()));

                // oi.intake.onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.intake()));
                // oi.intake.onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));

                // oi.intake.whileTrue(turretSubsystem.run(() ->
                // turretSubsystem.update(drivetrain.getState().Pose)));

                oi.shoot.onTrue(turretSubsystem
                                .runOnce(() -> turretSubsystem.setShooterSpeed(1.0 /
                                                Constants.Turret.shooterRatio)));
                oi.shoot.onFalse(turretSubsystem.runOnce(() -> turretSubsystem.setShooterSpeed(0.0)));

                oi.calibrateShooter.onTrue(turretSubsystem.calibrateHeading()
                                .andThen(turretSubsystem.runOnce(() -> turretSubsystem.setShooterHeading(0.0))));
                // oi.calibrateShooter.onTrue(turretSubsystem.calibrateHeading());

                // oi.shoot.onTrue(turretSubsystem
                // .runOnce(() -> turretSubsystem.setShooterHeading(Math.PI / 2)));
                // oi.shoot.onFalse(turretSubsystem.runOnce(() ->
                // turretSubsystem.setShooterHeading(Math.PI)));

                oi.trench.whileTrue(autonomousSubsystem.trench());

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return autonomousSubsystem.getAutonomousCommand();
        }
}