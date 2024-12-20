package frc.robot.subsystems.intake;

import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class IntakeConstants {
    public class IntakeRollerConstants {
        public static final int motorId = 21;
        public static final String CANbus = "*";
        public static final double reduction = 1;

        public static final double statorCurrentLimit = 40;
        public static final double supplyCurrentLimit = 20;

        public static final boolean inverted = false;
    }

    public class IntakePivotConstants {
        public static final int motorId = 20;
        public static final String CANbus = "*";
        public static final double reduction = 2.375;

        public static final double statorCurrentLimit = 40;
        public static final double supplyCurrentLimit = 20;

        public static final boolean inverted = false;

        public static final Gains gains = new Gains(20.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        public static final TrapezoidalConstraints constraints = new TrapezoidalConstraints(20.0, 60.0);
    }
}
