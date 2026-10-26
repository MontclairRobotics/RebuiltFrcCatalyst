package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.IndexerConstants;

public class Shooter {
  private final FlywheelMechanism flywheel = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
      .name("Flywheel")
      .motor(FlywheelConstants.LEFT_CAN_ID)
      .primaryFollower(FlywheelConstants.RIGHT_CAN_ID)
      .motorType(MotorType.KRAKEN_X60_FOC)
      .gearRatio(FlywheelConstants.GEARING)
      .moi(FlywheelConstants.MOMENT_OF_INERTIA)
      .currentLimit(FlywheelConstants.SUPPLY_CURRENT_LIMIT)   //supply current limit
      .statorCurrentLimit(FlywheelConstants.STATOR_CURRENT_LIMIT)
      .pid(FlywheelConstants.kP, 0, FlywheelConstants.kD)
      .feedforward(FlywheelConstants.kS, FlywheelConstants.kV)
      .velocityTolerance(FlywheelConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond))
      .build()
    );

  private final FlywheelMechanism indexer = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
    .name("Indexer")
    .motor(IndexerConstants.CAN_ID)
    .gearRatio(IndexerConstants.GEARING)
    .motorType(MotorType.KRAKEN_X60_FOC)
    .moi(IndexerConstants.MOMENT_OF_INERTIA)
    .pid(IndexerConstants.kP, 0, IndexerConstants.kD)
    .feedforward(IndexerConstants.kS, IndexerConstants.kV)
    .currentLimit(IndexerConstants.SUPPLY_CURRENT_LIMIT)
    .statorCurrentLimit(IndexerConstants.STATOR_CURRENT_LIMIT)
    .velocityTolerance(IndexerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond))
    .build()
  );

  private final FlywheelMechanism spindexer = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
    .name("Spindexer")
    .motor(20)
    .gearRatio(20)
    .motorType(MotorType.KRAKEN_X60_FOC)
    .moi(0.02)
    .pid(15, 0, 0)
    .feedforward(8, 0.67)
    .currentLimit(20)
    .statorCurrentLimit(60)
    .velocityTolerance(0.1)
    .build()
  );

  private final TurretMechanism turret = new TurretMechanism(
    TurretMechanism.Config.builder()
    .name("Turret")
    .motor(30)
    .cancoder(29, 3.0)
    .gearRatio(3.0 * 128.0/18)
    .range(-0.51, 0.51)

    .build()
  );

}
