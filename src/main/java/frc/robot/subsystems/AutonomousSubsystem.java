package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutonomousSubsystem extends SubsystemBase {

        private final CommandSwerveDrivetrain drivetrain;

        private final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withHeadingPID(5, 0, 0); /* tune this for your robot! */

        private final Field2d field;

        private boolean selfDriving = false;

        public AutonomousSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field) {
                this.drivetrain = drivetrain;
                this.field = field;
        }

        public ChassisSpeeds trenchInputAdjust(double inputX, double inputY, double rotationInput) {

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
                                                + Constants.Autonomous.rotationTolerance < targetRotation
                                                                .in(Degrees)) {
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
                                                + Constants.Autonomous.rotationTolerance < targetRotation
                                                                .in(Degrees)) {
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

        @Override
        public void periodic() {
                
        }

        @Override
        public void simulationPeriodic() {

        }

        public boolean isSelfDriving() {
                return selfDriving;
        }
}