package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.SerializerConstants;
import frc.robot.constants.TurretConstants;

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
      .pid(FlywheelConstants.kP, 0.0, FlywheelConstants.kD)
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
    .pid(IndexerConstants.kP, 0.0, IndexerConstants.kD)
    .feedforward(IndexerConstants.kS, IndexerConstants.kV)
    .currentLimit(IndexerConstants.SUPPLY_CURRENT_LIMIT)
    .statorCurrentLimit(IndexerConstants.STATOR_CURRENT_LIMIT)
    .velocityTolerance(IndexerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond))
    .build()
  );

  private final FlywheelMechanism serializer = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
    .name("Serializer")
    .motor(SerializerConstants.CAN_ID)
    .gearRatio(SerializerConstants.GEARING)
    .motorType(MotorType.KRAKEN_X60_FOC)
    .moi(SerializerConstants.MOMENT_OF_INERTIA)
    .pid(SerializerConstants.kP, 0.0, SerializerConstants.kD)
    .feedforward(SerializerConstants.kS, SerializerConstants.kV)
    .currentLimit(SerializerConstants.SUPPLY_CURRENT_LIMIT)
    .statorCurrentLimit(SerializerConstants.STATOR_CURRENT_LIMIT)
    .velocityTolerance(SerializerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond))
    .build()
  );

  private final TurretMechanism turret = new TurretMechanism(
    TurretMechanism.Config.builder()
    .name("Turret")
    .motor(TurretConstants.CAN_ID)
    .motorType(MotorType.KRAKEN_X60_FOC)
    .cancoder(TurretConstants.ENCODER_ID, TurretConstants.ROTOR_TO_SENSOR_RATIO)
    .gearRatio(TurretConstants.GEARING)
    .range(TurretConstants.MIN_ANGLE.in(Degrees), TurretConstants.MIN_ANGLE.in(Degrees))
    .pid(TurretConstants.kP, 0.0, TurretConstants.kD)
    .feedforward(TurretConstants.kS, TurretConstants.kV)
    //no jerk in constants - have to convert all to doubles
    .motionMagic(TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY.magnitude(), TurretConstants.MOTION_MAGIC_ACCELERATION.magnitude(), 0.0)
    .tolerance(TurretConstants.ANGLE_TOLERANCE.in(Degrees))
    .build()
  );

  private final RotationalMechanism hood = new RotationalMechanism(
    RotationalMechanism.Config.builder()
    .name("Hood")
    .motor(HoodConstants.CAN_ID)
    .motorType(MotorType.KRAKEN_X60_FOC)
    .gearRatio(HoodConstants.GEARING)
    .length(HoodConstants.HOOD_LENGTH.in(Meters))
    .range(HoodConstants.MIN_ANGLE.in(Degrees), HoodConstants.MAX_ANGLE.in(Degrees))
    .currentLimit(HoodConstants.SUPPLY_CURRENT_LIMIT)
    .statorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT)
    .pid(HoodConstants.kP, 0.0, HoodConstants.kD)
    .gravityGain(HoodConstants.kG)
    .motionMagic(
      HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY.in(RotationsPerSecond), 
      HoodConstants.MOTION_MAGIC_ACCELERATION.in(RotationsPerSecondPerSecond), 
      HoodConstants.MOTION_MAGIC_JERK
    )
    .build()
  );
}
