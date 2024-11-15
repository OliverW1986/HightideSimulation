package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final TalonFXSimState pivotMotorSim;

    private final TalonFXConfiguration config;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(Degrees.of(0));
    private final VoltageOut voltageOut = new VoltageOut(Volts.of(0));
    private final NeutralOut neutralOut = new NeutralOut();

    private State state = State.STOW;

    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1), reduction,
            0.0322933, Units.inchesToMeters(9.708526), 0, Units.degreesToRadians(110), false,
            0.0);

    public enum State {
        STOW(() -> Degrees.of(0)),
        HOMIMG(() -> Degrees.of(0)),
        INTAKE(() -> Degrees.of(90)),
        EXTENDED(() -> Degrees.of(180));

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
        pivotMotorSim = pivotMotor.getSimState();

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
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = reduction;

        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (state == State.STOW && atGoal()) {
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
            // pivotMotor.set(-0.1);
        }

        displayInfo(true);
    }

    public Command setState(State state) {
        return Commands.runEnd(() -> this.state = state, () -> this.state = State.STOW, this);
    }

    public boolean atGoal() {
        return pivotMotor.getPosition().getValue().isNear(state.positionSetpoint.get(), Degrees.of(1));
    }

    private Trigger currentTrigger() {
        return new Trigger(() -> pivotMotor.getSupplyCurrent().getValue().gte(Amps.of(20))).debounce(0.15);
    }

    private void displayInfo(boolean debug) {
        if (!debug)
            return;

        DogLog.log(this.getClass().getSimpleName() + "/State", state.toString());
        DogLog.log(this.getClass().getSimpleName() + "/StateSetpoint", state.positionSetpoint.get().in(Degrees));
        DogLog.log(this.getClass().getSimpleName() + "/Position", pivotMotor.getPosition().getValue().in(Degrees));
        DogLog.log(this.getClass().getSimpleName() + "/Velocity",
                pivotMotor.getVelocity().getValue().in(DegreesPerSecond));
        DogLog.log(this.getClass().getSimpleName() + "/StatorCurrent",
                pivotMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log(this.getClass().getSimpleName() + "/SupplyCurrent",
                pivotMotor.getSupplyCurrent().getValue().in(Amps));
        DogLog.log(this.getClass().getSimpleName() + "/Voltage", pivotMotor.getMotorVoltage().getValue().in(Volts));
        DogLog.log(this.getClass().getSimpleName() + "/AtGoal", atGoal());
        DogLog.log(this.getClass().getSimpleName() + "/UpperLimit", sim.hasHitUpperLimit());
        DogLog.log(this.getClass().getSimpleName() + "/LowerLimit", sim.hasHitLowerLimit());
    }

    @Override
    public void simulationPeriodic() {
        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(pivotMotorSim.getMotorVoltage());
        sim.update(0.02);

        pivotMotorSim.setRawRotorPosition(Units.radiansToRotations(sim.getAngleRads()) * reduction);
        pivotMotorSim.setRotorVelocity(Units.radiansToRotations(sim.getVelocityRadPerSec()) * reduction);
    }
}
