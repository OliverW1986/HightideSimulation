package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;

import static edu.wpi.first.units.Units.*;

public class EndEffector extends SubsystemBase {
    private final TalonFX pivotOneMotor, pivotTwoMotor, rollerMotor;

    private final TalonFXConfiguration pivotOneConfig, pivotTwoConfig, rollerConfig;

    private final MotionMagicVoltage pivotOneMotionMagicVoltage = new MotionMagicVoltage(0);
    private final MotionMagicVoltage pivotTwoMotionMagicVoltage = new MotionMagicVoltage(0);
    private final VoltageOut rollerVoltageOut = new VoltageOut(0);

    private State state = State.STOW;

    private final SingleJointedArmSim pivotOneSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 2.62, 0.0309841,
            Units.inchesToMeters(20), Units.degreesToRadians(-15), Math.PI, false, 0);

    private final SingleJointedArmSim pivotTwoSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 2.62, 0.02069841,
            Units.inchesToMeters(7.5), -Math.PI, Math.PI, false, Units.degreesToRadians(170));

    private final DCMotorSim rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.012, 1), DCMotor.getKrakenX60(1));

    public enum State {
        STOW,
        INTAKE_SUBSTATION,
        INTAKE_GROUND,
        PLACE,
    }

    public EndEffector() {
        pivotOneMotor = new TalonFX(EndEffectorConstants.kPivotMotorOnePort);
        pivotTwoMotor = new TalonFX(EndEffectorConstants.kPivotMotorTwoPort);
        rollerMotor = new TalonFX(EndEffectorConstants.kRollerMotorPort);

        pivotOneConfig = new TalonFXConfiguration();
        pivotTwoConfig = new TalonFXConfiguration();
        rollerConfig = new TalonFXConfiguration();

        // Pivot One Config
        pivotOneConfig.Slot0.kP = EndEffectorConstants.kPivotOneGains.kP();
        pivotOneConfig.Slot0.kI = EndEffectorConstants.kPivotOneGains.kI();
        pivotOneConfig.Slot0.kD = EndEffectorConstants.kPivotOneGains.kD();
        pivotOneConfig.Slot0.kS = EndEffectorConstants.kPivotOneGains.kS();
        pivotOneConfig.Slot0.kV = EndEffectorConstants.kPivotOneGains.kV();
        pivotOneConfig.Slot0.kG = EndEffectorConstants.kPivotOneGains.kG();

        pivotOneConfig.MotionMagic.MotionMagicAcceleration = EndEffectorConstants.kPivotOneConstraints
                .maxAcceleration();
        pivotOneConfig.MotionMagic.MotionMagicCruiseVelocity = EndEffectorConstants.kPivotOneConstraints.maxVelocity();

        pivotOneConfig.CurrentLimits.StatorCurrentLimit = EndEffectorConstants.kPivotMotorStatorCurrentLimit;
        pivotOneConfig.CurrentLimits.SupplyCurrentLowerLimit = EndEffectorConstants.kPivotMotorSupplyCurrentLimit;

        pivotOneConfig.MotorOutput.Inverted = EndEffectorConstants.kPivotOneInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        pivotOneConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotOneConfig.Feedback.SensorToMechanismRatio = EndEffectorConstants.kPivotOneGearRatio;

        // Pivot Two Config
        pivotTwoConfig.Slot0.kP = EndEffectorConstants.kPivotTwoGains.kP();
        pivotTwoConfig.Slot0.kI = EndEffectorConstants.kPivotTwoGains.kI();
        pivotTwoConfig.Slot0.kD = EndEffectorConstants.kPivotTwoGains.kD();
        pivotTwoConfig.Slot0.kS = EndEffectorConstants.kPivotTwoGains.kS();
        pivotTwoConfig.Slot0.kV = EndEffectorConstants.kPivotTwoGains.kV();
        pivotTwoConfig.Slot0.kG = EndEffectorConstants.kPivotTwoGains.kG();

        pivotTwoConfig.MotionMagic.MotionMagicAcceleration = EndEffectorConstants.kPivotTwoConstraints
                .maxAcceleration();
        pivotTwoConfig.MotionMagic.MotionMagicCruiseVelocity = EndEffectorConstants.kPivotTwoConstraints.maxVelocity();

        pivotTwoConfig.CurrentLimits.StatorCurrentLimit = EndEffectorConstants.kPivotMotorStatorCurrentLimit;
        pivotTwoConfig.CurrentLimits.SupplyCurrentLowerLimit = EndEffectorConstants.kPivotMotorSupplyCurrentLimit;

        pivotTwoConfig.MotorOutput.Inverted = EndEffectorConstants.kPivotTwoInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        pivotTwoConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotTwoConfig.Feedback.SensorToMechanismRatio = EndEffectorConstants.kPivotTwoGearRatio;

        // Roller Motor Config
        rollerConfig.CurrentLimits.StatorCurrentLimit = EndEffectorConstants.kRollerMotorStatorCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = EndEffectorConstants.kRollerMotorSupplyCurrentLimit;

        rollerConfig.MotorOutput.Inverted = EndEffectorConstants.kRollerInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        pivotOneMotor.getConfigurator().apply(pivotOneConfig);
        pivotTwoMotor.getConfigurator().apply(pivotTwoConfig);
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    @Override
    public void periodic() {
        switch (state) {
            case STOW -> {
                pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(0));
                pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(0));
                rollerMotor.setControl(rollerVoltageOut.withOutput(0));
            }
            case INTAKE_SUBSTATION -> {
                switch (RobotState.gamePieceState) {
                    case CUBE:
                        pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(90));
                        pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(90));
                        rollerMotor.setControl(rollerVoltageOut.withOutput(12.0));
                        break;
                    case CONE:
                        pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(180));
                        pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(180));
                        rollerMotor.setControl(rollerVoltageOut.withOutput(12.0));
                        break;
                }
            }
            case INTAKE_GROUND -> {
                switch (RobotState.gamePieceState) {
                    case CUBE:
                        pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(90));
                        pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(90));
                        rollerMotor.setControl(rollerVoltageOut.withOutput(12.0));
                        break;
                    case CONE:
                        pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(180));
                        pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(180));
                        rollerMotor.setControl(rollerVoltageOut.withOutput(12.0));
                        break;
                }
            }
            case PLACE -> {
                switch (RobotState.gamePieceState) {
                    case CUBE:
                        pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(90));
                        pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(90));
                        rollerMotor.setControl(rollerVoltageOut.withOutput(12.0));
                        break;
                    case CONE:
                        pivotOneMotor.setControl(pivotOneMotionMagicVoltage.withPosition(180));
                        pivotTwoMotor.setControl(pivotTwoMotionMagicVoltage.withPosition(180));
                        rollerMotor.setControl(rollerVoltageOut.withOutput(12.0));
                        break;
                }
            }
        }

        displayInfo(true);
    }

    public Command setState(State state) {
        return runEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    private void displayInfo(boolean debug) {
        if (!debug)
            return;

        DogLog.log("EndEffector/State", state.toString());

        DogLog.log("EndEffector/PivotOne/Position", pivotOneMotor.getRotorPosition().getValue().in(Degrees));
        DogLog.log("EndEffector/PivotOne/Velocity", pivotOneMotor.getRotorVelocity().getValue().in(DegreesPerSecond));
        DogLog.log("EndEffector/PivotOne/StatorCurrent", pivotOneMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log("EndEffector/PivotOne/SupplyCurrent", pivotOneMotor.getSupplyCurrent().getValue().in(Amps));
        DogLog.log("EndEffector/PivotOne/MotorVoltage", pivotOneMotor.getMotorVoltage().getValue().in(Volts));
        DogLog.log("EndEffector/PivotOne/SupplyVoltage", pivotOneMotor.getSupplyVoltage().getValue().in(Volts));

        DogLog.log("EndEffector/PivotTwo/Position", pivotTwoMotor.getRotorPosition().getValue().in(Degrees));
        DogLog.log("EndEffector/PivotTwo/Velocity", pivotTwoMotor.getRotorVelocity().getValue().in(DegreesPerSecond));
        DogLog.log("EndEffector/PivotTwo/StatorCurrent", pivotTwoMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log("EndEffector/PivotTwo/SupplyCurrent", pivotTwoMotor.getSupplyCurrent().getValue().in(Amps));
        DogLog.log("EndEffector/PivotTwo/MotorVoltage", pivotTwoMotor.getMotorVoltage().getValue().in(Volts));
        DogLog.log("EndEffector/PivotTwo/SupplyVoltage", pivotTwoMotor.getSupplyVoltage().getValue().in(Volts));

        DogLog.log("EndEffector/Roller/Position", rollerMotor.getRotorPosition().getValue().in(Degrees));
        DogLog.log("EndEffector/Roller/Velocity", rollerMotor.getRotorVelocity().getValue().in(DegreesPerSecond));
        DogLog.log("EndEffector/Roller/StatorCurrent", rollerMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log("EndEffector/Roller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValue().in(Amps));
        DogLog.log("EndEffector/Roller/MotorVoltage", rollerMotor.getMotorVoltage().getValue().in(Volts));
        DogLog.log("EndEffector/Roller/SupplyVoltage", rollerMotor.getSupplyVoltage().getValue().in(Volts));
    }

    @Override
    public void simulationPeriodic() {
        var pivotOneMotorSim = pivotOneMotor.getSimState();
        var pivotTwoMotorSim = pivotTwoMotor.getSimState();
        var rollerMotorSim = rollerMotor.getSimState();

        pivotOneMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        pivotTwoMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rollerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        pivotOneSim.setInput(pivotOneMotorSim.getMotorVoltage());
        pivotOneSim.update(0.02);

        pivotTwoSim.setInput(pivotTwoMotorSim.getMotorVoltage());
        pivotTwoSim.update(0.02);

        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(0.02);

        pivotOneMotorSim.setRawRotorPosition(Radians.of(pivotOneSim.getAngleRads()));
        pivotOneMotorSim.setRotorVelocity(RadiansPerSecond.of(pivotOneSim.getVelocityRadPerSec()));

        pivotTwoMotorSim.setRawRotorPosition(Radians.of(pivotTwoSim.getAngleRads()));
        pivotTwoMotorSim.setRotorVelocity(RadiansPerSecond.of(pivotTwoSim.getVelocityRadPerSec()));

        rollerMotorSim.setRawRotorPosition(rollerSim.getAngularPosition());
        rollerMotorSim.setRotorVelocity(rollerSim.getAngularVelocity());
    }
}
