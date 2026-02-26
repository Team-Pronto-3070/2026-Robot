package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
//import com.therekrab.autopilot.APConstraints;
//import com.therekrab.autopilot.APProfile;
//import com.therekrab.autopilot.APTarget;
//import com.therekrab.autopilot.Autopilot;
//import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

        /*private static final APConstraints kConstraints = new APConstraints()
                        .withAcceleration(15.0)
                        .withJerk(8.0);

        private static final APProfile kProfile = new APProfile(kConstraints)
                        .withErrorXY(Centimeters.of(4))
                        .withErrorTheta(Degrees.of(1))
                        .withBeelineRadius(Centimeters.of(8));

        public static final Autopilot kAutopilot = new Autopilot(kProfile);

        private APTarget target = new APTarget(new Pose2d(new Translation2d(), new Rotation2d()))
                        .withEntryAngle(Rotation2d.kZero);*/

        private final Field2d field;

        private boolean selfDriving = false;

        public AutonomousSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field) {
                this.drivetrain = drivetrain;
                this.field = field;
        }
        
        public ChassisSpeeds trenchInputAdjust(double inputX, double inputY, double rotationInput) {

                Translation2d trench = Constants.Autonomous.redTrenchLeft;

                Pose2d pose = drivetrain.getState().Pose;

                double bumpHalfWidth = 0.5; //exact half of bump is 0.564

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

                double yProximity = Math.min(1.0, Math.max(Constants.Autonomous.minForce, (yDistance - bumpHalfWidth) / Constants.Autonomous.yActivationRange));
                
                if (yDistance < Constants.Autonomous.yActivationRange && xDistance < Constants.Autonomous.xActivationRange){

                        boolean pauseX = false;

                        //Blue side facing trenches 
                        if (pose.getX() > trench.getX() - bumpHalfWidth && inputX > 0){

                                pauseX = (xDistance / Constants.Autonomous.pauseRatio) < yDistance && yDistance > Constants.Autonomous.tolerance && xDistance > bumpHalfWidth && inputX > 0 && xDistance < 1.75;

                                double force = pauseX && Math.abs(inputY) < 0.1 ? Constants.Autonomous.maxForce : Constants.Autonomous.maxForce * (1 - Math.pow(1 - yProximity,  Constants.Autonomous.alignmentExponent));

                                //System.out.printf("inputY: %.2f, inputX: %.2f, force: %.3f%n", inputY, inputX, force);
                                if (pose.getY() + Constants.Autonomous.tolerance < trench.getY()) {
                                        inputY += -inputX * force;
                                } else if (pose.getY() - Constants.Autonomous.tolerance > trench.getY()) {
                                        inputY += inputX * force;
                                }

                                if (pose.getRotation().getDegrees() > 0){
                                        if (pose.getRotation().getDegrees() < 180 - Constants.Autonomous.rotationTolerance){
                                                rotationInput = 1;
                                        }
                                } else {
                                        if (pose.getRotation().getDegrees() > -180 + Constants.Autonomous.rotationTolerance){
                                                rotationInput = -1;
                                        }
                                }
                                
                        
                        //Red side facing trenches 
                        } else if (trench.getX() + bumpHalfWidth > pose.getX() && inputX < 0){

                                pauseX = (xDistance / Constants.Autonomous.pauseRatio) < yDistance && yDistance > Constants.Autonomous.tolerance && xDistance > bumpHalfWidth && inputX < 0 && xDistance < 1.75;

                                double force = pauseX && Math.abs(inputY) < 0.1 ? Constants.Autonomous.maxForce : Constants.Autonomous.maxForce * (1 - Math.pow(1 - yProximity, Constants.Autonomous.alignmentExponent));

                                if (pose.getY() + Constants.Autonomous.tolerance < trench.getY()) {
                                        inputY += inputX * force;
                                } else if (pose.getY() - Constants.Autonomous.tolerance > trench.getY()) {
                                        inputY += -inputX * force;

                                }

                                if (pose.getRotation().getDegrees() + Constants.Autonomous.rotationTolerance < 0){
                                        rotationInput = 1;
                                        //rotationInput = 1 * Math.max(Math.pow(pose.getRotation().getDegrees() / 180, 4), 0.5);
                                } else if (pose.getRotation().getDegrees() - Constants.Autonomous.rotationTolerance > 0) {
                                        rotationInput = -1;
                                        //rotationInput = -1 * Math.max(Math.pow((Math.abs(pose.getRotation().getDegrees()) / 180), 4), 0.5);
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
                // Do not update target while moving towards it
                if (!selfDriving) {
                        Pose2d pose = drivetrain.getState().Pose;

                        Translation2d trench = new Translation2d();

                        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
                                        : DriverStation.Alliance.Blue;

                        Distance cutoff = Constants.fieldWidth.div(2);

                        switch (Constants.Autonomous.trenchMethod) {
                                default: // NEAREST
                                        if (pose.getMeasureX().lt(cutoff)) {
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
                                        break;

                                case NEAREST_BIASED:
                                        cutoff = Constants.fieldWidth.div(2)
                                                        .plus(alliance.equals(Alliance.Blue)
                                                                        ? Constants.Autonomous.allianceBias
                                                                        : Constants.Autonomous.allianceBias
                                                                                        .unaryMinus());

                                        if (pose.getMeasureX().lt(cutoff)) {
                                                if (pose.getTranslation()
                                                                .getY() < Constants.fieldHeight.in(Meters) / 2)
                                                        trench = Constants.Autonomous.blueTrenchRight;
                                                else
                                                        trench = Constants.Autonomous.blueTrenchLeft;
                                        } else {
                                                if (pose.getTranslation()
                                                                .getY() < Constants.fieldHeight.in(Meters) / 2)
                                                        trench = Constants.Autonomous.redTrenchLeft;
                                                else
                                                        trench = Constants.Autonomous.redTrenchRight;
                                        }
                                        break;

                                case VELOCITY:
                                        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

                                        double vx = speeds.vxMetersPerSecond;
                                        double vy = speeds.vyMetersPerSecond;

                                        double db = Constants.Autonomous.velocityDeadband.in(MetersPerSecond);

                                        if (pose.getX() < Constants.Autonomous.blueTrenchLeft.getX()) {
                                                if (vy > db) {
                                                        trench = Constants.Autonomous.blueTrenchLeft;
                                                } else if (vy < -db) {
                                                        trench = Constants.Autonomous.blueTrenchRight;
                                                } else {
                                                        if (Constants.fieldHeight.div(2).gt(pose.getMeasureY())) {
                                                                trench = Constants.Autonomous.blueTrenchRight;
                                                        } else {
                                                                trench = Constants.Autonomous.blueTrenchLeft;
                                                        }
                                                }
                                        } else if (pose.getX() > Constants.Autonomous.redTrenchLeft.getX()) {
                                                if (vy > db) {
                                                        trench = Constants.Autonomous.redTrenchRight;
                                                } else if (vy < -db) {
                                                        trench = Constants.Autonomous.redTrenchLeft;
                                                } else {
                                                        if (Constants.fieldHeight.div(2).gt(pose.getMeasureY())) {
                                                                trench = Constants.Autonomous.redTrenchLeft;
                                                        } else {
                                                                trench = Constants.Autonomous.redTrenchRight;
                                                        }
                                                }
                                        } else {
                                                boolean movingUp = false;
                                                boolean movingDown = false;
                                                boolean movingLeft = false;
                                                boolean movingRight = false;

                                                boolean topHalf = false;
                                                boolean leftHalf = false;

                                                
                                                if (vx > db)
                                                        movingRight = true;
                                                else if (vx < -db)
                                                        movingLeft = true;
                                                
                                                if (vy > db)
                                                        movingUp = true;
                                                else if (vy < -db)
                                                        movingDown = true;
                                                
                                                if (pose.getMeasureX().lt(Constants.fieldWidth.div(2)))
                                                        leftHalf = true;
                                                
                                                if (pose.getMeasureY().gt(Constants.fieldHeight.div(2)))
                                                        topHalf = true;

                                                // System.out.println("vx: " + vx + ", vy: " + vy);
                                                // System.out.println("Moving Up: " + movingUp + ", Moving Down: " + movingDown);

                                                if (movingUp & movingLeft)
                                                        trench = Constants.Autonomous.blueTrenchLeft;
                                                else if (movingDown & movingLeft)
                                                        trench = Constants.Autonomous.blueTrenchRight;
                                                else if (movingUp & movingRight)
                                                        trench = Constants.Autonomous.redTrenchRight;
                                                else if (movingDown & movingRight)
                                                        trench = Constants.Autonomous.redTrenchLeft;

                                                else if (movingLeft & topHalf)
                                                        trench = Constants.Autonomous.blueTrenchLeft;
                                                else if (movingLeft & !topHalf)
                                                        trench = Constants.Autonomous.blueTrenchRight;
                                                else if (movingRight & topHalf)
                                                        trench = Constants.Autonomous.redTrenchRight;
                                                else if (movingRight & !topHalf)
                                                        trench = Constants.Autonomous.redTrenchLeft;

                                                else if (movingUp & leftHalf)
                                                        trench = Constants.Autonomous.blueTrenchLeft;
                                                else if (movingDown & leftHalf)
                                                        trench = Constants.Autonomous.blueTrenchRight;
                                                else if (movingUp & !leftHalf)
                                                        trench = Constants.Autonomous.redTrenchRight;
                                                else if (movingDown & !leftHalf)
                                                        trench = Constants.Autonomous.redTrenchLeft;
                                                
                                                else if (topHalf & leftHalf)
                                                        trench = Constants.Autonomous.blueTrenchLeft;
                                                else if (!topHalf & leftHalf)
                                                        trench = Constants.Autonomous.blueTrenchRight;
                                                else if (topHalf & !leftHalf)
                                                        trench = Constants.Autonomous.redTrenchRight;
                                                else if (!topHalf & !leftHalf)
                                                        trench = Constants.Autonomous.redTrenchLeft;
                                        }
                                        break;
                        }

                        /*target = new APTarget(new Pose2d(trench,
                                        new Rotation2d(Math.round(pose.getRotation().getRadians() / (Math.PI))
                                                        * (Math.PI))))
                                        .withEntryAngle(pose.getX() > trench.getX()
                                                        ? Rotation2d.k180deg
                                                        : Rotation2d.kZero)
                                        .withVelocity(5.0); */
                }
        }

        @Override
        public void simulationPeriodic() {

        }

        public boolean isSelfDriving() {
                return selfDriving;
        }

        public Command trench() {
                return drivetrain.run(() -> {
                        /*selfDriving = true;

                        ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;
                        Pose2d pose = drivetrain.getState().Pose;

                        APResult output = kAutopilot.calculate(pose, robotRelativeSpeeds, target);

                        /* these speeds are field relative */ /* 
                        LinearVelocity veloX = output.vx();
                        LinearVelocity veloY = output.vy();
                        Rotation2d headingReference = output.targetAngle();

                        drivetrain.setControl(request
                                        .withVelocityX(veloX)
                                        .withVelocityY(veloY)
                                        .withTargetDirection(headingReference));
                })
                                .until(() -> kAutopilot.atTarget(drivetrain.getState().Pose, target))
                                .finallyDo(() -> {
                                        selfDriving = false; */
                                });
        }

        public enum TrenchMethod {
                NEAREST,
                NEAREST_BIASED,
                VELOCITY
        }
}