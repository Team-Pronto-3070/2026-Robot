package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
    public static final class OI {
        public static final int driverPort = 0;
        public static final int operatorPort = 0;
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
                new Rotation3d(0, 0.52, 0)
            );
        }

    }

    

    public static final class Shooter {

    }

    public static final class Swerve {

    }

}
