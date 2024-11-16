package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class ElevatorConstants {
    public static final Transform3d STAGE_ONE_ZEROED_POSE = new Transform3d(-0.223, 0.0, 0.1368, new Rotation3d());
    public static final Transform3d STAGE_TWO_ZEROED_POSE = new Transform3d(-0.1971, 0.0, 0.15, new Rotation3d());

    public static final int leaderMotorId = 30;
    public static final int followerMotorId = 31;
    public static final String CANbus = "*";

    public static final double reduction = 5.6;

    public static final double statorCurrentLimit = 60;
    public static final double supplyCurrentLimit = 40;

    public static final boolean inverted = false;

    public static final Gains gains = new Gains(20.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public static final TrapezoidalConstraints constraints = new TrapezoidalConstraints(20.0, 60.0);

}
