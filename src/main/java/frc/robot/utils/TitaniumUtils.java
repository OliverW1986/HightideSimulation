package frc.robot.utils;

public class TitaniumUtils {
    public enum GamePieceState {
        CONE,
        CUBE
    }

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kG) {
    }

    public record TrapezoidalConstraints(double maxVelocity, double maxAcceleration) {
    }
}
