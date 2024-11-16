package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.leaderMotorId, ElevatorConstants.CANbus);
  private final TalonFX followerMotor = new TalonFX(ElevatorConstants.followerMotorId);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final ElevatorSim sim = new ElevatorSim(DCMotor.getKrakenX60(2), ElevatorConstants.reduction, 11.25139919829,
      Units.inchesToMeters(0.75), 0, Units.inchesToMeters(62), false, 0);

  public enum State {
    STOW(() -> 0.0),
    HOMIMG(() -> 0.0),
    INTAKE(() -> 0.5);

    private final Supplier<Double> positionSetpoint;

    private State(Supplier<Double> positionSetpoint) {
      this.positionSetpoint = positionSetpoint;
    }
  }

  public Elevator() {
  }

  @Override
  public void periodic() {
    // Pose3d updatedStageTwoPose = STAGE_TWO_ZEROED_POSE;
    // final double stageTwoDistance = 0.5;

    // final double xPos = stageTwoDistance * Math.cos(Units.degreesToRadians(35));
    // final double zPos = stageTwoDistance * Math.sin(Units.degreesToRadians(35));

    // updatedStageTwoPose = updatedStageTwoPose.plus(new Transform3d(xPos, 0, zPos,
    // new Rotation3d()));

    // DogLog.log("Elevator/Stage2Pose", updatedStageTwoPose);
  }
}
