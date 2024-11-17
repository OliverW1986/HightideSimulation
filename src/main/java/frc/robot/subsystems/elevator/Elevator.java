package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final TalonFXConfiguration config;

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  private State state = State.STOW;

  private final ElevatorSim sim = new ElevatorSim(DCMotor.getKrakenX60(2), ElevatorConstants.reduction, 11.25139919829,
      Units.inchesToMeters(0.75), 0, Units.inchesToMeters(62), false, 0);

  public enum State {
    STOW(() -> Inches.of(0.0)),
    HOMIMG(() -> Inches.of(0.0)),
    MIDDLE_NODE(() -> Inches.of(32.0)),
    TOP_NODE(() -> Inches.of(62.0)),
    DOUBLE_SUBSTATION(() -> Inches.of(48.0));

    private final Supplier<Distance> positionSetpoint;

    private State(Supplier<Distance> positionSetpoint) {
      this.positionSetpoint = positionSetpoint;
    }

    private Angle getRotationSetpoint() {
      return Rotations
          .of(positionSetpoint.get().divide(Meters.of(2 * Math.PI * Units.inchesToMeters(0.75))).magnitude());
    }

    @Override
    public String toString() {
      return this.name() + " Setpoint Rotations: " + getRotationSetpoint().in(Rotations);
    }
  }

  public Elevator() {
    leaderMotor = new TalonFX(ElevatorConstants.leaderMotorId, ElevatorConstants.CANbus);
    followerMotor = new TalonFX(ElevatorConstants.followerMotorId, ElevatorConstants.CANbus);
    followerMotor.setControl(new Follower(ElevatorConstants.leaderMotorId, true));

    config = new TalonFXConfiguration();

    config.Slot0.kP = ElevatorConstants.gains.kP();
    config.Slot0.kI = ElevatorConstants.gains.kI();
    config.Slot0.kD = ElevatorConstants.gains.kD();
    config.Slot0.kS = ElevatorConstants.gains.kS();
    config.Slot0.kV = ElevatorConstants.gains.kV();
    config.Slot0.kG = ElevatorConstants.gains.kG();

    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.constraints.maxAcceleration();
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.constraints.maxVelocity();

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimit;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.reduction;

    config.MotorOutput.Inverted = ElevatorConstants.inverted ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leaderMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    if (state == State.STOW && atGoal().getAsBoolean()) {
      leaderMotor.setControl(neutralOut);
    } else if (state == State.HOMIMG) {
      leaderMotor.setControl(voltageOut.withOutput(Volts.of(-0.5)));
      if (currentTrigger().getAsBoolean()) {
        leaderMotor.setPosition(Rotations.of(0));
        Commands.print("Homing complete");
        state = State.STOW;
      }
    } else {
      leaderMotor.setControl(motionMagicVoltage.withPosition(state.getRotationSetpoint()));
      // leaderMotor.setControl(voltageOut.withOutput(Volts.of(6.0)));
    }

    displayInfo(true);
  }

  public Command setState(State state) {
    return Commands.runEnd(() -> this.state = state, () -> this.state = State.STOW, this);
  }

  public Trigger atGoal() {
    return new Trigger(
        () -> leaderMotor.getPosition().getValue().isNear(state.getRotationSetpoint(), Rotations.of(0.1)));
  }

  private Trigger currentTrigger() {
    return new Trigger(() -> leaderMotor.getSupplyCurrent().getValue().gte(Amps.of(20)));
  }

  private void displayInfo(boolean debug) {
    if (!debug)
      return;

    DogLog.log("Elevator/State", state.toString());
    DogLog.log("Elevator/Position", leaderMotor.getPosition().getValueAsDouble());
    DogLog.log("Elevator/Velocity", leaderMotor.getVelocity().getValueAsDouble());
    DogLog.log("Elevator/StatorCurrent", leaderMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Elevator/SupplyCurrent", leaderMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Elevator/Voltage", leaderMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/AtGoal", atGoal().getAsBoolean());
    DogLog.log("Elevator/UpperLimit", sim.hasHitUpperLimit());
    DogLog.log("Elevator/LowerLimit", sim.hasHitLowerLimit());

    // Pose3d updatedStageTwoPose = STAGE_TWO_ZEROED_POSE;
    // final double stageTwoDistance = 0.5;

    // final double xPos = stageTwoDistance * Math.cos(Units.degreesToRadians(35));
    // final double zPos = stageTwoDistance * Math.sin(Units.degreesToRadians(35));

    // updatedStageTwoPose = updatedStageTwoPose.plus(new Transform3d(xPos, 0, zPos,
    // new Rotation3d()));

    // DogLog.log("Elevator/Stage2Pose", updatedStageTwoPose);
  }

  @Override
  public void simulationPeriodic() {
    var elevatorMotorSim = leaderMotor.getSimState();
    elevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    sim.setInputVoltage(elevatorMotorSim.getMotorVoltage());
    sim.update(0.02);

    elevatorMotorSim.setRawRotorPosition(sim.getPositionMeters() / (2 * Math.PI * Units.inchesToMeters(0.75)));
    elevatorMotorSim.setRotorVelocity(sim.getVelocityMetersPerSecond() / (2 * Math.PI * Units.inchesToMeters(0.75)));
  }
}
