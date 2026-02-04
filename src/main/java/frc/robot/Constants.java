package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
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
        public static final Translation3d redHub = new Translation3d(
                Inches.of(158.6 + (47 / 2)),
                Inches.of(317.7 / 2),
                Inches.of(72));

        public static final Translation3d blueHub = new Translation3d(
                Inches.of(651.2 - (158.6 + (47 / 2))),
                Inches.of(158.85),
                Inches.of(72));

        // Look up table of Shooter speeds and hub distances
        public static final Map<Double, Double> speedTable = Map.of(
            1.0, 1.0,
            2.0, 2.0,
            3.0, 3.0
        ); //TODO: Test

        public static final double shooterRatio = 1.2;

        public static final int mainShooterMotorID = 20; // TODO: fix
        public static final int hoodShooterMotorID = 21; // TODO: fix
        public static final int turretMotorID = 97; // TODO: fix

    }
}
