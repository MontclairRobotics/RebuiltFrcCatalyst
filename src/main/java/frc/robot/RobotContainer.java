// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TunerConstants;

public class RobotContainer {

	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);

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

	public RobotContainer(){
		shooter = new Shooter();
		drivetrain = TunerConstants.createDrivetrain();

	}
}
