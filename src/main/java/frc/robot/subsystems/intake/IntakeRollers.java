package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants.*;

import java.util.function.Supplier;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final TalonFXSimState pivotMotorSim;

    private final TalonFXConfiguration config;

    private final VoltageOut voltageOut = new VoltageOut(Volts.of(0));
    private final NeutralOut neutralOut = new NeutralOut();

    public enum State {
        STOW(() -> RotationsPerSecond.of(0)),
        HOMIMG(() -> RotationsPerSecond.of(0)),
        INTAKE(() -> RotationsPerSecond.of(90)),
        EXTENDED(() -> RotationsPerSecond.of(180));

        private final Supplier<AngularVelocity> positionSetpoint;

        private State(Supplier<AngularVelocity> positionSetpoint) {
            this.positionSetpoint = positionSetpoint;
        }

        @Override
        public String toString() {
            return this.name() + " Setpoint Degrees: " + positionSetpoint.get().in(RotationsPerSecond);
        }
    }

    public IntakeRollers() {
        pivotMotor = new TalonFX(motorId, CANbus);
        pivotMotorSim = pivotMotor.getSimState();

        config = new TalonFXConfiguration();
    }
}
