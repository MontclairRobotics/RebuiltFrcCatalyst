// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.sim.SimDashboard;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterCoordinator;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.PoseUtils;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import frc.robot.util.sim.FuelSim;

public class RobotContainer {
	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

	public static Trigger operatorWantsToFireTrigger = operatorController.R1();
	public static Trigger operatorWantsToTrackHubTrigger = operatorController.circle();
	public static Trigger operatorWantsToTrackFerryPointTrigger = operatorController.triangle();
	public static boolean shouldShootAuto = false;

	public static CommandSwerveDrivetrain drivetrain;
	public static Intake intake;
	public static Shooter shooter;

	public static ShooterCoordinator shooterCoordinator;

	private SwerveDriveSimulation driveSimulation;
	//private SimGamePieces fuel;
	private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
	public static FuelSim fuelSim = new FuelSim("fuel");

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

	public static Trigger precisionTrigger = driverController.R2();
	public static Trigger robotRelativeTrigger = driverController.L1();
	public static Trigger xModeTrigger = driverController.R1();
	public static Trigger turboTrigger = driverController.L2();

	final SimDashboard dash = new SimDashboard();

	public RobotContainer(){

		drivetrain = TunerConstants.createDrivetrain();
		intake = new Intake();

		shooter = new Shooter();

		shooterCoordinator = new ShooterCoordinator(
			operatorWantsToFireTrigger,
			operatorWantsToTrackHubTrigger,
			operatorWantsToTrackFerryPointTrigger,
			() -> shouldShootAuto
		);

		if(!RobotBase.isReal()){
			dash.add(shooter.getFlywheel());
			dash.add(shooter.getIndexer());
			dash.add(shooter.getSerializer());
			dash.add(shooter.getTurret());
			dash.add(shooter.getHood());

			dash.add(intake.getRollers());
			dash.add(intake.getPivot());

			dash.start();

			driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;
			//fuel = new SimGamePieces("Fuel");
			fuelSim.enableAirResistance();
			fuelSim.start();

			fuelSim.registerRobot(
				Constants.BUMPER_WIDTH,
				Constants.BUMPER_WIDTH,
				Inches.of(6),
				() -> drivetrain.getRobotPose(),
				() -> drivetrain.getFieldRelativeSpeeds()
			);

			fuelSim.spawnStartingFuel();
		}

		configureBindings();

    drivetrain.registerTelemetry(logger::telemeterize);
	}

	public void update(){
		dash.update();
	}

	public void simulationPeriodic(){
		SimulatedArena.getInstance().simulationPeriodic();

    // 3. Stream game pieces for AdvantageScope.
    //fuel.clear();
    //for (var piece : SimulatedArena.getInstance().getGamePiecesByType("Fuel")) {
    //    fuel.set(piece, piece.getPose3d());
    //}
    //fuel.publish();   // -> /Catalyst/Sim/Fu
	}

	private void configureBindings(){
		driverController.povRight().whileTrue(new WheelRadiusCharacterization(Direction.CLOCKWISE, drivetrain));
		driverController.povLeft().whileTrue(new WheelRadiusCharacterization(Direction.COUNTER_CLOCKWISE, drivetrain));

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
