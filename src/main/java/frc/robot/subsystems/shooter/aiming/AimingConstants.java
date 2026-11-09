package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.catalyst.util.InterpolatingTable;
import frc.robot.util.tunables.Tunable;

public class AimingConstants {

	private static double latency = 0.08; // seconds it takes to reach desired state once state is set
	public static double getLatency() { return latency; }
	public static Tunable latencyTunable = new Tunable("latency", latency, (input) -> latency = input);

	public record ShotSettings(Angle angle, AngularVelocity flywheelVelocity) implements Interpolatable<ShotSettings> {
		@Override
		public ShotSettings interpolate(ShotSettings endValue, double t) {
			return new ShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				RotationsPerSecond.of(MathUtil.interpolate(this.flywheelVelocity.in(RotationsPerSecond), endValue.flywheelVelocity.in(RotationsPerSecond), t))
			);
		}
    }

	public record SimShotSettings(Angle angle, LinearVelocity exitVelocity) implements Interpolatable<SimShotSettings> {
		@Override
		public SimShotSettings interpolate(SimShotSettings endValue, double t) {
			return new SimShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				MetersPerSecond.of(MathUtil.interpolate(this.exitVelocity.in(MetersPerSecond), endValue.exitVelocity.in(MetersPerSecond), t))
			);
		}
    }

	// RPM maps
	public static final InterpolatingTable REAL_RPM_MAP = new InterpolatingTable();
	public static final InterpolatingTable REAL_FERRY_RPM_MAP = new InterpolatingTable();
	public static final InterpolatingTable SIM_RPM_MAP = new InterpolatingTable();
	public static final InterpolatingTable SIM_FERRY_RPM_MAP = new InterpolatingTable();

	// hood angle maps
	public static final InterpolatingTable REAL_HOOD_ANGLE_MAP = new InterpolatingTable();
	public static final InterpolatingTable REAL_FERRY_HOOD_ANGLE_MAP = new InterpolatingTable();
	public static final InterpolatingTable SIM_HOOD_ANGLE_MAP = new InterpolatingTable();
	public static final InterpolatingTable SIM_FERRY_HOOD_ANGLE_MAP = new InterpolatingTable();


	// time of flight maps
	public static final InterpolatingTable REAL_TOF_MAP = new InterpolatingTable();
	public static final InterpolatingTable REAL_FERRY_TOF_MAP = new InterpolatingTable();
	public static final InterpolatingTable SIM_TOF_MAP = new InterpolatingTable();
	public static final InterpolatingTable SIM_FERRY_TOF_MAP = new InterpolatingTable();

	static {
		REAL_RPM_MAP.add(0.00, 19.5 * 60);
		REAL_RPM_MAP.add(1.50, 20.0 * 60);
		REAL_RPM_MAP.add(2.49, 22.0 * 60);
		REAL_RPM_MAP.add(3.49, 24.0 * 60);
		REAL_RPM_MAP.add(3.98, 25.0 * 60);
		REAL_RPM_MAP.add(4.28, 25.5 * 60);
		REAL_RPM_MAP.add(4.74, 26.3 * 60);
		REAL_RPM_MAP.add(5.07, 26.3 * 60);
		REAL_RPM_MAP.add(5.73, 28.5 * 60);
		REAL_RPM_MAP.add(7.00, 30.0 * 60);

		REAL_HOOD_ANGLE_MAP.add(0.00, 0);
		REAL_HOOD_ANGLE_MAP.add(1.50, 0);
		REAL_HOOD_ANGLE_MAP.add(2.49, 9.5);
		REAL_HOOD_ANGLE_MAP.add(3.49, 10);
		REAL_HOOD_ANGLE_MAP.add(3.98, 10.8);
		REAL_HOOD_ANGLE_MAP.add(4.28, 12.5);
		REAL_HOOD_ANGLE_MAP.add(4.74, 13);
		REAL_HOOD_ANGLE_MAP.add(5.07, 15);
		REAL_HOOD_ANGLE_MAP.add(5.73, 18);
		REAL_HOOD_ANGLE_MAP.add(7.00, 18);

		/*
		REAL_MAP.put(0.00, new ShotSettings(Degrees.of(0.0), RotationsPerSecond.of(19.5)));
		REAL_MAP.put(1.50, new ShotSettings(Degrees.of(0), RotationsPerSecond.of(20)));
		REAL_MAP.put(2.49, new ShotSettings(Degrees.of(9.5), RotationsPerSecond.of(22)));
		REAL_MAP.put(3.49, new ShotSettings(Degrees.of(10), RotationsPerSecond.of(24)));
		REAL_MAP.put(3.98, new ShotSettings(Degrees.of(10.8), RotationsPerSecond.of(25)));
		REAL_MAP.put(4.28, new ShotSettings(Degrees.of(12.5), RotationsPerSecond.of(25.5)));
		REAL_MAP.put(4.74, new ShotSettings(Degrees.of(13), RotationsPerSecond.of(26.3)));
		REAL_MAP.put(5.07, new ShotSettings(Degrees.of(15), RotationsPerSecond.of(26.3)));
		REAL_MAP.put(5.73, new ShotSettings(Degrees.of(18), RotationsPerSecond.of(28.5)));
		REAL_MAP.put(7.00, new ShotSettings(Degrees.of(18), RotationsPerSecond.of(30)));
		*/

		//double x = 0.0;

		// REAL_MAP.add(7.00, new ShotSettings(Degrees.of(20), RotationsPerSecond.of(29)));
		// REAL_MAP.add(5.56895, new ShotSettings(Degrees.of(20), RotationsPerSecond.of(27.84)));
		// REAL_MAP.add(5.08635, new ShotSettings(Degrees.of(20), RotationsPerSecond.of(26.37)));
		// REAL_MAP.add(4.70535, new ShotSettings(Degrees.of(19), RotationsPerSecond.of(25.8)));
		// REAL_MAP.add(4.2164, new ShotSettings(Degrees.of(18), RotationsPerSecond.of(24.99)));
		// REAL_MAP.add(3.58648, new ShotSettings(Degrees.of(17), RotationsPerSecond.of(24.36)));
		// REAL_MAP.add(2.92862, new ShotSettings(Degrees.of(11), RotationsPerSecond.of(23.15)));
		// REAL_MAP.add(2.49, new ShotSettings(Degrees.of(9.5), RotationsPerSecond.of(22)));
		// REAL_MAP.add(1.50, new ShotSettings(Degrees.of(0), RotationsPerSecond.of(20)));
		// REAL_MAP.add(0.00, new ShotSettings(Degrees.of(0.0), RotationsPerSecond.of(19.5)));

		REAL_TOF_MAP.add(0.00, 0.8);
		REAL_TOF_MAP.add(1.50, 0.81);
		REAL_TOF_MAP.add(2.50, 0.86);
		REAL_TOF_MAP.add(3.45, 0.98);
		REAL_TOF_MAP.add(4.52, 1.1);
		REAL_TOF_MAP.add(5.11, 1.14);
		REAL_TOF_MAP.add(7.00, 1.2);

		REAL_FERRY_RPM_MAP.add(20.0, 40.0 * 60);
		REAL_FERRY_RPM_MAP.add(9.59, 34.0 * 60);
		REAL_FERRY_RPM_MAP.add(8.67, 30.4 * 60);
		REAL_FERRY_RPM_MAP.add(6.63, 26.9 * 60);
		REAL_FERRY_RPM_MAP.add(5.15, 24.5 * 60);
		REAL_FERRY_RPM_MAP.add(3.67, 22.44 * 60);
		REAL_FERRY_RPM_MAP.add(0.0, 15.0 * 60);

		REAL_FERRY_HOOD_ANGLE_MAP.add(20.0, 22.5);
		REAL_FERRY_HOOD_ANGLE_MAP.add(9.59, 22.5);
		REAL_FERRY_HOOD_ANGLE_MAP.add(8.67, 22.5);
		REAL_FERRY_HOOD_ANGLE_MAP.add(6.63, 22.5);
		REAL_FERRY_HOOD_ANGLE_MAP.add(5.15, 22.5);
		REAL_FERRY_HOOD_ANGLE_MAP.add(3.67, 22.5);
		REAL_FERRY_HOOD_ANGLE_MAP.add(0.0, 22.5);

		/*
		REAL_FERRY_MAP.put(20.0, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(40)));
		REAL_FERRY_MAP.put(9.59, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(34)));
		REAL_FERRY_MAP.put(8.67, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(30.4)));
		REAL_FERRY_MAP.put(6.63, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(26.9)));
		REAL_FERRY_MAP.put(5.15, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(24.5)));
		REAL_FERRY_MAP.put(3.67, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(22.44)));
		REAL_FERRY_MAP.put(0.0, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(15)));
		*/

		REAL_FERRY_TOF_MAP.add(20.0, 3.0);
		REAL_FERRY_TOF_MAP.add(9.59, 1.8);
		REAL_FERRY_TOF_MAP.add(8.67, 1.78);
		REAL_FERRY_TOF_MAP.add(6.63, 1.38);
		REAL_FERRY_TOF_MAP.add(5.15, 1.22);
		REAL_FERRY_TOF_MAP.add(3.67, 1.06);
		REAL_FERRY_TOF_MAP.add(0.0, 1.0);

		SIM_RPM_MAP.add(0.99912, 7 * 60);
		SIM_RPM_MAP.add(2.005294, 7 * 60);
		SIM_RPM_MAP.add(3.000977, 7.4 * 60);
		SIM_RPM_MAP.add(3.993633, 8.2 * 60);
		SIM_RPM_MAP.add(4.998351, 8.6 * 60);
		SIM_RPM_MAP.add(5.714192, 9 * 60);

		SIM_HOOD_ANGLE_MAP.add(0.99912, 7);
		SIM_HOOD_ANGLE_MAP.add(2.005294, 16);
		SIM_HOOD_ANGLE_MAP.add(3.000977, 22);
		SIM_HOOD_ANGLE_MAP.add(3.993633, 26);
		SIM_HOOD_ANGLE_MAP.add(4.998351, 33);
		SIM_HOOD_ANGLE_MAP.add(5.714192, 37);

		/*
		SIM_MAP.put(0.99912, new SimShotSettings(Degrees.of(7), MetersPerSecond.of(7)));
		SIM_MAP.put(2.005294, new SimShotSettings(Degrees.of(16), MetersPerSecond.of(7)));
		SIM_MAP.put(3.000977, new SimShotSettings(Degrees.of(22), MetersPerSecond.of(7.4)));
		SIM_MAP.put(3.993633, new SimShotSettings(Degrees.of(26), MetersPerSecond.of(8.2)));
		SIM_MAP.put(4.998351, new SimShotSettings(Degrees.of(33), MetersPerSecond.of(8.6)));
		SIM_MAP.put(5.714192, new SimShotSettings(Degrees.of(37), MetersPerSecond.of(9)));
		*/

		SIM_TOF_MAP.add(0.99912, 1.12);
		SIM_TOF_MAP.add(2.005294, 1.12);
		SIM_TOF_MAP.add(3.000977, 1.0);
		SIM_TOF_MAP.add(3.993633, 1.16);
		SIM_TOF_MAP.add(4.998351, 1.1);
		SIM_TOF_MAP.add(5.714192, 1.14);

		SIM_FERRY_RPM_MAP.add(3.6669131055202966, 6.6 * 60);
		SIM_FERRY_RPM_MAP.add(5.147707889525594, 7.5 * 60);
		SIM_FERRY_RPM_MAP.add(6.628582671689323, 8.5 * 60);
		SIM_FERRY_RPM_MAP.add(8.666543502404746, 11 * 60);
		SIM_FERRY_RPM_MAP.add(9.59, 10.75 * 60);

		SIM_FERRY_HOOD_ANGLE_MAP.add(3.6669131055202966, 30);
		SIM_FERRY_HOOD_ANGLE_MAP.add(5.147707889525594, 40);
		SIM_FERRY_HOOD_ANGLE_MAP.add(6.628582671689323, 40);
		SIM_FERRY_HOOD_ANGLE_MAP.add(8.666543502404746, 30);
		SIM_FERRY_HOOD_ANGLE_MAP.add(9.59, 40);

		/*
		SIM_FERRY_MAP.put(9.59, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(10.75)));
		SIM_FERRY_MAP.put(3.6669131055202966, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(6.6)));//
		SIM_FERRY_MAP.put(8.666543502404746, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(11)));//
		SIM_FERRY_MAP.put(6.628582671689323, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(8.5)));//
		SIM_FERRY_MAP.put(5.147707889525594, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(7.5)));//
		*/
		
		SIM_FERRY_TOF_MAP.add(9.59, 1.8);
		SIM_FERRY_TOF_MAP.add(3.6669131055202966, 1.06);
		SIM_FERRY_TOF_MAP.add(8.666543502404746, 1.78);
		SIM_FERRY_TOF_MAP.add(6.628582671689323, 1.38);
		SIM_FERRY_TOF_MAP.add(5.147707889525594, 1.22);
	}

	public record ShootingParameters(
		Angle robotRelativeTurretAngle,
		Angle hoodAngle,
		AngularVelocity flywheelVelocity,
		double timeSecondsForSetpoint) {}

	public record SimShootingParameters(
		Angle robotRelativeTurretAngle,
		Angle hoodAngle,
		LinearVelocity exitVelocity) {}

}