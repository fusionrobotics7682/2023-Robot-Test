package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class DriveConstants{
        public static final int LEFT_MOTOR_MASTER = 1;
        public static final int LEFT_MOTOR_SLAVE = 2;
        public static final int RIGHT_MOTOR_MASTER = 3;
        public static final int RIGHT_MOTOR_SLAVE = 4;

        public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.23);
        public static final double WHEEL_RADIUS_METER = Units.inchesToMeters(3);
        public static final double GEAR_RATIO = 10.72;

    }

    public static class VisionConstants{
        
    }

    public static class ShoulderConstants{
        public static final int SHOULDER_MOTOR = 7;
        // TODO: Find the actual values
        public static final double REDUCTOR_RATIO = 1;
        public static final double GEAR_RATIO = 1;
        public static final double GENERAL_RATIO = REDUCTOR_RATIO * GEAR_RATIO;
        public static final double DEGREES_2_POSITION = GENERAL_RATIO / 360;
        public static final double POSITION_2_DEGREES = 360 / GENERAL_RATIO;
        public static final double DEFAULT_ARM_ANGLE = -30;
    }

    public static class TurretConstants{
        public static final int TURRET_MOTOR = 5;
        public static final double GEAR_RATIO = 12;
        public static final double DEGREES_2_POSITION = GEAR_RATIO / 360;
        public static final double POSITION_2_DEGREES = 360 / GEAR_RATIO;
    }

    public static class ExtensibleConstants{
        public static final int EXTENSIBLE_MOTOR = 6;
        public static final double DEFAULT_ARM_LENGTH = 1.2;
        public static final double WINDER_PERIMETER = 0.070022;
        public static final double WINDER_GEAR_RATIO = 100;
        public static final double POSITION_TO_METER = WINDER_PERIMETER / WINDER_GEAR_RATIO;
        
    }
}
