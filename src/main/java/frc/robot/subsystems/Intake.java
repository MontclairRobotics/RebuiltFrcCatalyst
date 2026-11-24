package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.RollersConstants;

public class Intake {

    private static final RollerMechanism rollers = new RollerMechanism(RollerMechanism.Config.builder()
    .name("Rollers")
    .motor(RollersConstants.CAN_ID)
    .intakeSpeed(1) // need to set
    .ejectSpeed(1)
    .currentLimit(RollersConstants.SUPPLY_CURRENT_LIMIT)
    .statorCurrentLimit(RollersConstants.STATOR_CURRENT_LIMIT)
    .build());

    private static final RotationalMechanism pivot = new RotationalMechanism(RotationalMechanism.Config.builder()
    .name("Pivot")
    .motor(PivotConstants.CAN_ID)
    .gearRatio(PivotConstants.GEARING)
    .length(PivotConstants.ARM_LENGTH.in(Meters))
    .range(PivotConstants.MIN_ANGLE.in(Degrees), PivotConstants.MAX_ANGLE.in(Degrees))
    .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
    .currentLimit(PivotConstants.SUPPLY_CURRENT_LIMIT)
    .motionMagic(PivotConstants.MOTION_MAGIC_CRUISE_VELOCITY.in(RotationsPerSecond), PivotConstants.MOTION_MAGIC_ACCELERATION.in(RotationsPerSecondPerSecond), PivotConstants.MOTION_MAGIC_JERK)
    .build()
    );

    public Command intakeCommand() {
        return Commands.parallel(
            pivot.goTo("MIN"),
            rollers.intake()
        );
    }

    public Command jostleCommand() {
        return Commands.parallel(
            rollers.intake(),
            Commands.repeatingSequence(
                pivot.goTo("JIGGLE"),
                pivot.goTo("MIN")
            )
        );
    }

    public RollerMechanism getRollers(){
        return rollers;
    }

    public RotationalMechanism getPivot(){
        return pivot;
    }

}
