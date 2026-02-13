package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class Constants {

    public static final Distance fieldHeight = Inches.of(317.7);
    public static final Distance fieldWidth = Inches.of(651.2);

    public static final class OI {
        public static final int driverPort = 0;
        public static final int operatorPort = 1;
    }

    public static final class Vision {
        public static final class FrontCamera {

            public static final String name = "Front";

            public static final Transform3d transform = new Transform3d(
                    new Translation3d(-0.02, 0.19, 0),
                    new Rotation3d(0, 0.52, 0));
        }

        public static final class FrontCamera2 {
            public static final String name = "Front2";

            public static final Transform3d transform = new Transform3d(
                    new Translation3d(0.02, 0.19, 0),
                    new Rotation3d(0, 0.52, 0));
        }

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

        private static final double ferryOffset = 24;

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

        public static final Distance blueLineX = Inches.of(((fieldWidth.in(Inches) - neutralZoneWidth.in(Inches)) / 2));
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
                0.0, 0.0,
                1.0, 1000.0,
                2.0, 2000.0,
                3.0, 3000.0,
                6.0, 6000.0
            ); // TODO: Test

        public static final TreeMap<Double, Double> speedTreeMap = new TreeMap<>(speedTable);

        /*
         * When shooting on the move, we subtract the robot velocity from the target
         * position to adjust the trajectory. I think this should technically be based
         * off of the time it will take for the projectile to reach the target, but we
         * will use a constant for now and change later if it is not good enough.
         */
        public static final double shootOnTheMoveScale = 0.1;

        public static final double shooterRatio = 1.2;

        public static final int mainShooterMotorID = 20; // TODO: fix
        public static final int hoodShooterMotorID = 21; // TODO: fix
        public static final int turretMotorID = 97; // TODO: fix

    }

    public static final class Autonomous {
        public static final Translation2d redLeftTrench = new Translation2d(fieldWidth.minus(Inches.of(180)), Inches.of(25));
    }
}
