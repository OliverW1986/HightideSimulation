package frc.robot.subsystems.endeffector;

import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class EndEffectorConstants {
    public static final int kPivotMotorOnePort = 30;
    public static final int kPivotMotorTwoPort = 31;
    public static final int kRollerMotorPort = 32;

    public static final double kPivotOneGearRatio = 2.62;
    public static final double kPivotTwoGearRatio = 2.62;

    public static final boolean kPivotOneInverted = false;
    public static final boolean kPivotTwoInverted = false;
    public static final boolean kRollerInverted = false;

    public static final int kPivotMotorStatorCurrentLimit = 40;
    public static final int kPivotMotorSupplyCurrentLimit = 30;

    public static final int kRollerMotorStatorCurrentLimit = 30;
    public static final int kRollerMotorSupplyCurrentLimit = 20;

    public static final Gains kPivotOneGains = new Gains(0.1, 0.0, 0.0, 0.0, 0, 0.0);
    public static final Gains kPivotTwoGains = new Gains(0.1, 0.0, 0.0, 0.0, 0, 0.0);

    public static final TrapezoidalConstraints kPivotOneConstraints = new TrapezoidalConstraints(2.0, 2.0);
    public static final TrapezoidalConstraints kPivotTwoConstraints = new TrapezoidalConstraints(2.0, 2.0);
}
