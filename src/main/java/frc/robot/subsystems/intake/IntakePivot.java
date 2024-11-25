package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants.*;

import java.util.function.Supplier;

public class IntakePivot extends SubsystemBase {
    private final TalonFX pivotMotor;

    private final TalonFXConfiguration config;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(Degrees.of(0));
    private final VoltageOut voltageOut = new VoltageOut(Volts.of(0));
    private final NeutralOut neutralOut = new NeutralOut();

    private State state = State.STOW;

    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), reduction,
            0.47820244 / 4.0, Units.inchesToMeters(9.708526), 0, Units.degreesToRadians(110), true, 0.0);

    public enum State {
        STOW(() -> Degrees.of(0)),
        HOMIMG(() -> Degrees.of(0)),
        INTAKE(() -> Degrees.of(90));

        private final Supplier<Angle> positionSetpoint;

        private State(Supplier<Angle> positionSetpoint) {
            this.positionSetpoint = positionSetpoint;
        }

        @Override
        public String toString() {
            return this.name() + " Setpoint Degrees: " + positionSetpoint.get().in(Degrees);
        }
    }

    public IntakePivot() {
        pivotMotor = new TalonFX(motorId, CANbus);

        config = new TalonFXConfiguration();

        config.Slot0.kP = gains.kP();
        config.Slot0.kI = gains.kI();
        config.Slot0.kD = gains.kD();
        config.Slot0.kS = gains.kS();
        config.Slot0.kV = gains.kV();
        config.Slot0.kG = gains.kG();
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
        config.MotionMagic.MotionMagicCruiseVelocity = constraints.maxAcceleration();

        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;

        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = reduction;

        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (state == State.STOW && atGoal().getAsBoolean()) {
            pivotMotor.setControl(neutralOut);
        } else if (state == State.HOMIMG) {
            pivotMotor.setControl(voltageOut.withOutput(Volts.of(2.0)));
            if (currentTrigger().getAsBoolean()) {
                pivotMotor.setPosition(Degrees.of(0));
                Commands.print("Homing complete");
                state = State.STOW;
            }
        } else {
            pivotMotor.setControl(motionMagicVoltage.withPosition(state.positionSetpoint.get()));
        }

        displayInfo(true);
    }

    public Command setState(State state) {
        return Commands.runEnd(() -> this.state = state, () -> this.state = State.STOW, this);
    }

    public Trigger atGoal() {
        return new Trigger(
                () -> pivotMotor.getPosition().getValue().isNear(state.positionSetpoint.get(), Degrees.of(1)));
    }

    private Trigger currentTrigger() {
        return new Trigger(() -> pivotMotor.getSupplyCurrent().getValue().gte(Amps.of(20))).debounce(0.15);
    }

    private void displayInfo(boolean debug) {
        if (!debug)
            return;

        DogLog.log("IntakePivot/State", state.toString());
        DogLog.log("IntakePivot/AtGoal", atGoal().getAsBoolean());

        DogLog.log("IntakePivot/Position", pivotMotor.getPosition().getValue().in(Degrees));
        DogLog.log("IntakePivot/Velocity",
                pivotMotor.getVelocity().getValue().in(DegreesPerSecond));
        DogLog.log("IntakePivot/StatorCurrent",
                pivotMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log("IntakePivot/SupplyCurrent",
                pivotMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log("IntakePivot/MotorVoltage",
                pivotMotor.getMotorVoltage().getValue().in(Volts));
        DogLog.log("IntakePivot/SupplyVoltage",
                pivotMotor.getSupplyVoltage().getValue().in(Volts));
    }

    @Override
    public void simulationPeriodic() {
        var pivotMotorSim = pivotMotor.getSimState();

        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(pivotMotorSim.getMotorVoltage());
        sim.update(0.02);

        pivotMotorSim.setRawRotorPosition(Units.radiansToRotations(sim.getAngleRads()) * reduction);
        pivotMotorSim.setRotorVelocity(Units.radiansToRotations(sim.getVelocityRadPerSec()) * reduction);

        DogLog.log("IntakePivot/Sim/SimPosition", Units.radiansToDegrees(sim.getAngleRads()));
        DogLog.log("IntakePivot/Sim/SimVelocityRPS", Units.radiansToRotations(sim.getVelocityRadPerSec()));
        DogLog.log("IntakePivot/Sim/IntakePose", new Pose3d(new Translation3d(0.2925, 0, 0.2),
                new Rotation3d(0, Units.degreesToRadians(-90) + sim.getAngleRads(), 0)));
    }
}
