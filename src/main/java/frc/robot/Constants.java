package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.AutonomousSubsystem.TrenchMethod;

public class Constants {

        public static final Distance fieldHeight = Inches.of(317.7);
        public static final Distance fieldWidth = Inches.of(651.2);

        public static final class OI {
                public static final int driverPort = 0;
                public static final int operatorPort = 1;
        }

        public static final class Vision {
                public static final class CameraParams {
                        public final String name;
                        public final Transform3d transform;
                        public final Matrix<N3, N1> singleTagStdDevs;
                        public final Matrix<N3, N1> multiTagStdDevs;

                        CameraParams(String name, Transform3d transform, Matrix<N3, N1> singleTagStdDevs,
                                        Matrix<N3, N1> multiTagStdDevs) {
                                this.name = name;
                                this.transform = transform;
                                this.singleTagStdDevs = singleTagStdDevs;
                                this.multiTagStdDevs = multiTagStdDevs;
                        }
                }

                // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

                private static final Distance groundDistance = Inches.of(20.5); // distance off ground

                // distance between two front cameras
                private static final Distance frontWidth = Inches.of(21 + (11 / 16));

                // distance from center to front cameras
                private static final Distance frontDistance = Inches.of((33.5 / 2) - (3 + (7 / 8)) - (6 / 8));

                // distance between two side cameras
                private static final Distance sideWidth = Inches.of(25 + (5 / 16));

                // distance from center to side cameras
                private static final Distance sideDistance = Inches.of((33.5 / 2) - (3 + (7 / 8)) - (3 + (6 / 16)));

                public static final CameraParams leftParams = new CameraParams(
                                "Left",
                                new Transform3d(
                                                new Translation3d(
                                                                sideDistance,
                                                                sideWidth.div(2),
                                                                groundDistance),
                                                new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(90))),
                                // The standard deviations of our vision estimated poses, which affect
                                // correction rate
                                // TODO: Experiment and determine estimation noise on an actual robot
                                VecBuilder.fill(4, 4, 8),
                                VecBuilder.fill(0.5, 0.5, 1));

                public static final CameraParams frontLeftParams = new CameraParams(
                                "Front Left",
                                new Transform3d(
                                                new Translation3d(
                                                                frontDistance,
                                                                frontWidth.div(2),
                                                                groundDistance),
                                                new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(20))),
                                // The standard deviations of our vision estimated poses, which affect
                                // correction rate
                                // TODO: Experiment and determine estimation noise on an actual robot
                                VecBuilder.fill(4, 4, 8),
                                VecBuilder.fill(0.5, 0.5, 1));

                public static final CameraParams frontRightParams = new CameraParams(
                                "Front Right",
                                new Transform3d(
                                                new Translation3d(
                                                                frontDistance,
                                                                frontWidth.div(-2),
                                                                groundDistance),
                                                new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-20))),
                                // The standard deviations of our vision estimated poses, which affect
                                // correction rate
                                // TODO: Experiment and determine estimation noise on an actual robot
                                VecBuilder.fill(4, 4, 8),
                                VecBuilder.fill(0.5, 0.5, 1));

                public static final CameraParams rightParams = new CameraParams(
                                "Right",
                                new Transform3d(
                                                new Translation3d(
                                                                sideDistance,
                                                                sideWidth.div(-2),
                                                                groundDistance),
                                                new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-90))),
                                // The standard deviations of our vision estimated poses, which affect
                                // correction rate
                                // TODO: Experiment and determine estimation noise on an actual robot
                                VecBuilder.fill(4, 4, 8),
                                VecBuilder.fill(0.5, 0.5, 1));
        }

        public static final class Turret {

                private static final Distance allianceZoneWidth = Inches.of(158.6);
                private static final Distance neutralZoneWidth = Inches.of(283.0);

                public static final Translation3d blueHub = new Translation3d(
                                Inches.of(allianceZoneWidth.in(Inches) + (47 / 2)),
                                Inches.of(fieldHeight.in(Inches) / 2),
                                Inches.of(72));

                public static final Translation3d redHub = new Translation3d(
                                Inches.of(fieldWidth.in(Inches) - (allianceZoneWidth.in(Inches) + (47 / 2))),
                                Inches.of(fieldHeight.in(Inches) / 2),
                                Inches.of(72));

                private static final double ferryOffset = 48;

                public static final Translation3d blueFerryLeft = new Translation3d(
                                Inches.of(ferryOffset),
                                Inches.of(fieldHeight.in(Inches) - ferryOffset),
                                Inches.of(0));

                public static final Translation3d blueFerryRight = new Translation3d(
                                Inches.of(ferryOffset),
                                Inches.of(ferryOffset),
                                Inches.of(0));

                public static final Translation3d redFerryLeft = new Translation3d(
                                Inches.of(fieldWidth.in(Inches) - ferryOffset),
                                Inches.of(ferryOffset),
                                Inches.of(0));

                public static final Translation3d redFerryRight = new Translation3d(
                                Inches.of(fieldWidth.in(Inches) - ferryOffset),
                                Inches.of(fieldHeight.in(Inches) - ferryOffset),
                                Inches.of(0));

                public static final Translation2d turretToRobot = new Translation2d(
                                Inches.of(5.596),
                                Inches.of(0));

                public static final Distance blueLineX = Inches
                                .of(((fieldWidth.in(Inches) - neutralZoneWidth.in(Inches)) / 2));
                public static final Distance redLineX = Inches.of(fieldWidth.in(Inches) - blueLineX.in(Inches));

                // // Look up table of Shooter speeds and hub distances. Format is (Distance,
                // [Shooter main speed, Shooter hood speed])
                // public static final Map<Double, Double[]> speedTable = Map.of(
                // 1.0, new Double[]{1.0, 1.0},
                // 2.0, new Double[]{2.0, 2.0},
                // 3.0, new Double[]{3.0, 3.0}); // TODO: Test

                // Look up table of Shooter speeds and hub distances. Format is (Distance,
                // Shooter speed)
                public static final Map<Double, Double> speedTable = Map.of(
                                2.2, 2150.0,
                                2.8, 2400.0,
                                3.2, 2550.0,
                                4.2, 2830.0,
                                3.65, 2750.0,
                                4.8, 3000.0);

                public static final TreeMap<Double, Double> speedTreeMap = new TreeMap<>(speedTable);

                /*
                 * When shooting on the move, we subtract the robot velocity from the target
                 * position to adjust the trajectory. I think this should technically be based
                 * off of the time it will take for the projectile to reach the target, but we
                 * will use a constant for now and change later if it is not good enough.
                 */
                public static final double shootOnTheMoveScale = 1.0;

                // greater than one is forward spin
                public static final double shooterRatio = 0.9;

                public static final int mainShooterMotorID = 20;
                public static final int hoodShooterMotorID = 21;
                public static final int turretMotorID = 19;

                public static final double turretBeltRatio = 126.0 / 18.0;

                public static final Angle clockwiseStop = Degrees.of(160);


                public static final Angle counterclockwiseStop = Degrees.of(180);

                public static final Current hardStopCurrent = Amps.of(26.0);

        }

        public static final class Spindexer {
                public static final int spindexerMotorID = 16;
                public static final int indexerMotorID = 17;
        }

        public static final class Intake {
                public static final int intakeMotorID = 22;
                public static final int liftingMotorID = 23;
        }

        public static final class Autonomous {
                public static final Translation2d blueTrenchLeft = new Translation2d(Inches.of(180),
                                fieldHeight.minus(Inches.of(25)));
                public static final Translation2d blueTrenchRight = new Translation2d(Inches.of(180),
                                Inches.of(25));

                public static final Translation2d redTrenchLeft = new Translation2d(
                                fieldWidth.minus(Inches.of(180)),
                                Inches.of(25));
                public static final Translation2d redTrenchRight = new Translation2d(
                                fieldWidth.minus(Inches.of(180)),
                                fieldHeight.minus(Inches.of(25)));

                /*
                 * What method to use to decide which trench to drive under
                 * 
                 * NEAREST: Drive under the closest trench
                 * NEAREST_BIASED: Drive under the closest trench but with a bias towards the
                 * current alliance side
                 * VELOCITY: Use the current robot velocity to pick the trench we are driving
                 * towards
                 */
                public static final TrenchMethod trenchMethod = TrenchMethod.VELOCITY;

                /*
                 * We usually want to drive back under our trench, but sometimes we want to
                 * drive under our opponent's trench, so we move the cutoff line away from our
                 * alliance wall to allow for a larger area where it will align under our
                 * trench.
                 */
                public static final Distance allianceBias = Inches.of(80);

                public static final LinearVelocity velocityDeadband = MetersPerSecond.of(0.15);
        }
}