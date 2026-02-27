// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

                SmartDashboard.putBoolean("On Field", false);

                new Trigger(() -> DriverStation.isFMSAttached()).onChange(Commands
                                .run(() -> SmartDashboard.putBoolean("On Field", DriverStation.isFMSAttached())));

                configureBindings();
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(drivetrain.run(() -> {
                        double driveX = oi.processed_drive_x.getAsDouble();
                        double driveY = oi.processed_drive_y.getAsDouble();
                        double rotationalRate = oi.processed_drive_rot.getAsDouble();

                        ChassisSpeeds adjustedInput = autonomousSubsystem.trenchInputAdjust(driveX, driveY,
                                        rotationalRate, SmartDashboard.getBoolean("On Field", false));

                        drivetrain.applyRequest(() -> drive
                                        .withVelocityX(adjustedInput.vxMetersPerSecond * MaxSpeed)
                                        .withVelocityY(adjustedInput.vyMetersPerSecond * MaxSpeed)
                                        .withRotationalRate(adjustedInput.omegaRadiansPerSecond * MaxAngularRate))
                                        .execute();
                }));

                leftCamera.setDefaultCommand(leftCamera.run(() -> {
                        leftCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds);
                                // leftCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                frontLeftCamera.setDefaultCommand(frontLeftCamera.run(() -> {
                        frontLeftCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds);
                                // frontLeftCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                frontRightCamera.setDefaultCommand(frontRightCamera.run(() -> {
                        frontRightCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds);
                                // frontRightCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                rightCamera.setDefaultCommand(rightCamera.run(() -> {
                        rightCamera.getLatestEstimation().ifPresent(est -> {
                                drivetrain.addVisionMeasurement(
                                                est.estimatedPose.toPose2d(),
                                                est.timestampSeconds);
                                // rightCamera.getEstimationStdDevs());
                        });
                }).ignoringDisable(true));

                turretSubsystem.setDefaultCommand(
                                turretSubsystem.runOnce(() -> {
                                        turretSubsystem.update(drivetrain.getState().Pose);

                                        if (SmartDashboard.getBoolean("On Field", false)) {
                                                if (oi.turret.getAsBoolean()) {
                                                        turretSubsystem.stopAiming();
                                                } else {
                                                        turretSubsystem.startAiming();
                                                }
                                        } else {
                                                if (oi.turret.getAsBoolean()) {
                                                        turretSubsystem.startAiming();
                                                } else {
                                                        turretSubsystem.stopAiming();
                                                }

                                        }
                                })
                                                .ignoringDisable(true));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Reset the field-centric heading on left bumper press.
                oi.gyroReset.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                oi.outtake.onTrue(spindexerSubsytem.runOnce(() -> spindexerSubsytem.outtake()));
                oi.outtake.onFalse(spindexerSubsytem.runOnce(() -> spindexerSubsytem.stop()));

                oi.intake.onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.intake()));
                oi.intake.onFalse(intakeSubsystem.runOnce(() -> intakeSubsystem.stop()));

                oi.shoot.onTrue(turretSubsystem.runOnce(() -> {
                        turretSubsystem.startShooter();
                        spindexerSubsytem.spin();
                }));
                oi.shoot.onFalse(turretSubsystem.runOnce(() -> {
                        turretSubsystem.stopShooter();
                        spindexerSubsytem.stop();
                }));

                oi.turret.onTrue(turretSubsystem.runOnce(() -> turretSubsystem.startAiming()));
                oi.turret.onFalse(turretSubsystem.runOnce(() -> turretSubsystem.stopAiming()));

                oi.calibrateShooter.onTrue(turretSubsystem.calibrateHeading());

                drivetrain.registerTelemetry(logger::telemeterize);
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