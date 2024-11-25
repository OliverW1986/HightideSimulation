package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants.*;

import java.util.function.Supplier;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX intakeMotor;

    private final TalonFXConfiguration config;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut neutralOut = new NeutralOut();

    private State state = State.IDLE;

    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 2.9556605e-5, 1), DCMotor.getFalcon500(1));

    public enum State {
        IDLE(() -> 0.0),
        INTAKING(() -> 0.5),
        EJECTING(() -> -0.5);

        private final Supplier<Double> percentOutput;

        private State(Supplier<Double> percentOutput) {
            this.percentOutput = percentOutput;
        }

        @Override
        public String toString() {
            return this.name() + " Percent Output: " + percentOutput.get();
        }
    }

    public IntakeRollers() {
        intakeMotor = new TalonFX(motorId, CANbus);

        config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLimit;

        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        intakeMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (state == State.IDLE) {
            intakeMotor.setControl(neutralOut);
        } else {
            intakeMotor.setControl(dutyCycleOut.withOutput(state.percentOutput.get()));
        }

        displayInfo(true);
    }

    public Command setState(State state) {
        return runEnd(() -> this.state = state, () -> this.state = State.IDLE);
    }

    public Trigger hasCube() {
        return new Trigger(() -> intakeMotor.getSupplyCurrent().getValue().gte(Amps.of(20)));
    }

    private void displayInfo(boolean debug) {
        if (!debug)
            return;

        DogLog.log(this.getClass().getSimpleName() + "/State", state.toString());
        DogLog.log(this.getClass().getSimpleName() + "/HasCube", hasCube().getAsBoolean());

        DogLog.log(this.getClass().getSimpleName() + "/Position", intakeMotor.getPosition().getValue().in(Degrees));
        DogLog.log(this.getClass().getSimpleName() + "/Velocity",
                intakeMotor.getVelocity().getValue().in(DegreesPerSecond));
        DogLog.log(this.getClass().getSimpleName() + "/StatorCurrent",
                intakeMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log(this.getClass().getSimpleName() + "/SupplyCurrent",
                intakeMotor.getStatorCurrent().getValue().in(Amps));
        DogLog.log(this.getClass().getSimpleName() + "/MotorVoltage",
                intakeMotor.getMotorVoltage().getValue().in(Volts));
        DogLog.log(this.getClass().getSimpleName() + "/SupplyVoltage",
                intakeMotor.getSupplyVoltage().getValue().in(Volts));
    }

    @Override
    public void simulationPeriodic() {
        var intakeMotorSim = intakeMotor.getSimState();

        intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(intakeMotorSim.getMotorVoltage());
        sim.update(0.02);

        intakeMotorSim.setRawRotorPosition(sim.getAngularPosition());
        intakeMotorSim.setRotorVelocity(sim.getAngularVelocity());
    }
}
