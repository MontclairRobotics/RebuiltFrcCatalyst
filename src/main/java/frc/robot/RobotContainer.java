// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.subsystems.ShooterCoordinator;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.PoseUtils;
import frc.robot.util.TunerConstants;

public class RobotContainer {
	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

	public static Trigger operatorWantsToFireTrigger = operatorController.R1();
	public static Trigger operatorWantsToTrackHubTrigger = operatorController.circle();
	public static Trigger operatorWantsToTrackFerryPointTrigger = operatorController.triangle();
	public static boolean shouldShootAuto = false;

	public static Shooter shooter;
	public static CommandSwerveDrivetrain drivetrain;

	public static boolean VISION_DEBUG = true;
	public static boolean TURRET_DEBUG = false;
	public static boolean FLYWHEEL_DEBUG = false;
	public static boolean HOOD_DEBUG  = false;
	public static boolean INDEXER_DEBUG = false;
	public static boolean SERIALIZER_DEBUG = false;
	public static boolean ROLLERS_DEBUG = false;
	public static boolean PIVOT_DEBUG = false;
	public static boolean DRIVETRAIN_DEBUG = false;
	public static boolean SUPERSTRUCTURE_DEBUG = false;

	public static boolean isUsingMegaTag1 = false;
	public static boolean isUsingMegaTag2 = false;

	public static ShooterCoordinator shooterCoordinator;

	public static Trigger precisionTrigger = driverController.R2();

	public RobotContainer(){
		drivetrain = TunerConstants.createDrivetrain();

		shooter = new Shooter();

		shooterCoordinator = new ShooterCoordinator(
			operatorWantsToFireTrigger,
			operatorWantsToTrackHubTrigger,
			operatorWantsToTrackFerryPointTrigger,
			() -> shouldShootAuto
		);
	}

	private void configureBindings(){
		driverController.povRight().whileTrue(new WheelRadiusCharacterization(Direction.CLOCKWISE, drivetrain));
		driverController.povLeft().whileTrue(new WheelRadiusCharacterization(Direction.COUNTER_CLOCKWISE, drivetrain));

		// driver
		drivetrain.setDefaultCommand(new JoystickDriveCommand());
		//driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
		//driverController.PS().onTrue(drivetrain.resetPoseCommand(new Pose2d(3.6, 4.035, new Rotation2d())));

		precisionTrigger
			.onTrue(drivetrain.setMaxSpeedsCommand(MetersPerSecond.of(1.25), RotationsPerSecond.of(0.25)))
			.onFalse(drivetrain.setMaxSpeedsCommand(TunerConstants.kSpeedAt12Volts, RotationsPerSecond.of(1.624)));

		driverController.triangle()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
		driverController.square()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(90)), false));
		driverController.cross()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
		driverController.circle()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(-90)), false));

	}
}
