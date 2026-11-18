package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.util.AimingSolver;
import frc.robot.RobotContainer;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.SerializerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ShooterCoordinator;
import frc.robot.subsystems.ShooterCoordinator.ShooterGoal;
import frc.robot.subsystems.shooter.aiming.Aiming;
import frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShootingParameters;

public class Shooter {
  private static final FlywheelMechanism flywheel = new FlywheelMechanism(
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

  private static final FlywheelMechanism indexer = new FlywheelMechanism(
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

  private static final FlywheelMechanism serializer = new FlywheelMechanism(
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


  private static final TurretMechanism turret = new TurretMechanism(
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

    private static final RotationalMechanism hood = new RotationalMechanism(
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
  
  public static Command movingAimCommand(TargetLocation target){
    Supplier<AimingSolver.Solution> sol = Aiming.calculateShootingParameters(target);

    return Commands.parallel(
      turret.track(
        sol,
        () -> RobotContainer.drivetrain.getWrappedHeading().getDegrees(),
        () -> Math.toDegrees(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond)
      ),
    
      flywheel.track(() -> sol.get().shooterRpm() / 60.0),
    
      hood.track(() -> sol.get().hoodDegrees())
    );
	}

    public static TargetLocation getTargetFromGoal(ShooterGoal goal) {
        TargetLocation target = null;

        switch (goal.mode()) {
            case SCORING:
                target = TargetLocation.HUB;
                break;
            case FERRYING_LEFT:
                target = TargetLocation.FERRY_LEFT;
                break;
            case FERRYING_RIGHT:
                target = TargetLocation.FERRY_RIGHT;
                break;
            case IDLE:
                break;
        };

        return target;
    }

  public static Command shoot(Supplier <ShooterGoal> goalSupplier){
    return Commands.run(() -> {

      ShooterGoal goal = goalSupplier.get();
      TargetLocation targetLocation = getTargetFromGoal(goal);
      Supplier<AimingSolver.Solution> sol = Aiming.calculateShootingParameters(targetLocation);

      if(goal.intent().isToUseTurretAngle()) {
        turret.track(
          sol,
          () -> RobotContainer.drivetrain.getWrappedHeading().getDegrees(),
          () -> Math.toDegrees(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond)
        );
      } 
      else {
          turret.holdAngle();
      }

      if(goal.intent().isToUseHoodAngle()) {
        hood.track(() -> sol.get().hoodDegrees());
      } else {
        hood.goToAndWait(HoodConstants.MIN_ANGLE.in(Degrees), HoodConstants.TOLERANCE.in(Degrees));
      }

      flywheel.track(() -> sol.get().shooterRpm() / 60.0);

      boolean ready = turret.atSetpoint()
        && flywheel.atSpeed() 
        && hood.atSetpoint(HoodConstants.MIN_ANGLE.in(Degrees));

      if(ready) {
        flywheel.spinUp(sol.get().shooterRpm() / 60.0);
      } else {
          // nothing needed
      }
    }
    );
  }

	public Translation2d getTurretFieldRelativePosition() {
		Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
		Rotation2d robotHeading = robotPose.getRotation();
		Translation2d fieldRelativeOffset = TurretConstants.TURRET_OFFSET.rotateBy(robotHeading);
		return robotPose.getTranslation().plus(fieldRelativeOffset);
	}

  public FlywheelMechanism getFlywheel(){
    return flywheel;
  }

  public FlywheelMechanism getIndexer(){
    return indexer;
  }
  
  public FlywheelMechanism getSerializer(){
    return serializer;
  }

  public TurretMechanism getTurret(){
    return turret;
  }

  public RotationalMechanism getHood(){
    return hood;
  }

  public Command startShootingInAuto() {
      return Commands.runOnce(() -> {
          RobotContainer.shouldShootAuto = true;
      });
  }

  public Command stopShootingInAuto() {
      return Commands.runOnce(() -> {
          RobotContainer.shouldShootAuto = false;
      });
  }

  public Command shoot(){

  }
}
