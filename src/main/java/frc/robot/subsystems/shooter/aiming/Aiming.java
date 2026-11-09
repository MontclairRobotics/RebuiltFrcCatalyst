package frc.robot.subsystems.shooter.aiming;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.util.AimingSolver;
import frc.lib.catalyst.util.InterpolatingTable;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseUtils;

public class Aiming {
	public static void initializeTargetLocations() {
		TargetLocation.HUB.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.Hub.HUB_LOCATION));
		TargetLocation.FERRY_LEFT.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.FerryWaypoints.LEFT_FERRYING_POINT));
		TargetLocation.FERRY_RIGHT.setLocation(PoseUtils.flipTranslationAlliance(FieldConstants.FerryWaypoints.RIGHT_FERRYING_POINT));
	}

	public static Supplier<AimingSolver.Solution> calculateShootingParameters(TargetLocation target){
		Translation2d targetLocation = target.getLocation();

		InterpolatingTable tofMap;
		InterpolatingTable rpmMap;
		InterpolatingTable angleMap;

		switch(target) {
			case HUB:
				tofMap = AimingConstants.REAL_TOF_MAP;
				rpmMap = AimingConstants.REAL_RPM_MAP;
				angleMap = AimingConstants.REAL_HOOD_ANGLE_MAP;
				break;
			case FERRY_LEFT:
			case FERRY_RIGHT:
				tofMap = AimingConstants.REAL_FERRY_TOF_MAP;
				rpmMap = AimingConstants.REAL_FERRY_RPM_MAP;
				angleMap = AimingConstants.REAL_FERRY_HOOD_ANGLE_MAP;
				break;
			default:
				tofMap = AimingConstants.REAL_TOF_MAP;
				rpmMap = AimingConstants.REAL_RPM_MAP;
				angleMap = AimingConstants.REAL_HOOD_ANGLE_MAP;
		}

		AimingSolver solver = AimingSolver.builder()
			.target(targetLocation)
			.shotTime(tofMap)
			.shooterRpm(rpmMap)
			.hoodAngle(angleMap)
			.build();

		Supplier<AimingSolver.Solution> sol = 
			() -> solver.solve(RobotContainer.drivetrain.getRobotPose(), RobotContainer.drivetrain.getFieldRelativeSpeeds());

		return sol;
	}

	public static Command aimShooter(FlywheelMechanism flywheel, TurretMechanism turret, RotationalMechanism hood, TargetLocation target){
		Supplier<AimingSolver.Solution> sol = calculateShootingParameters(target);

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

	public enum TargetLocation {
		HUB(FieldConstants.Hub.HUB_LOCATION),
		FERRY_LEFT(FieldConstants.FerryWaypoints.LEFT_FERRYING_POINT),
		FERRY_RIGHT(FieldConstants.FerryWaypoints.RIGHT_FERRYING_POINT);

		private Translation2d location;

		TargetLocation(Translation2d location) {
			this.location = location;
		}

		public Translation2d getLocation() {
			return location;
		}

		public void setLocation(Translation2d location) {
			this.location = location;
		}
	}
}