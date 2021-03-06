/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.Accelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends SimpleRobot {

	Drivetrain drivetrain;
	Joystick joystick1;
	Joystick joystick2;
	OpticalSensor lWheel1, lWheel2;
	Gyro gyro;
	Accelerometer acc;
	Tracker tracker;
	Relay camLight, baseLights;
	public final static double JOYSTICK_SENSITIVITY = 1;
	public final static double TRACKING_ERROR = .05;
	public final static double MAX_DRIVE_SPEED = 1;
	public final static double MAX_TURN_SPEED = 1;
	public final static double TILT_RATE = .1;
	public final static double LAUNCH_RATE = .65;
	public static final double LIFTER_RATE = .98;
	public static final int TILT_BASE = 770; // 526
	private static final boolean DISABLE_AUTONOMOUS = false;
	private double shooterSpeed1, shooterSpeed2, launchRate;
	private boolean tracking;
	Station s;
	NetworkTable test;
	private int pic_num;
	private int timePressed;
	private Value lightState;
	private LightRunner lrunner;

	public void robotInit() {

		try {
			// s = new Station();
		} catch (Exception e) {
			e.printStackTrace();
		}

		// Relays
		camLight = new Relay(Sensors.SHOOTER_RELAY);
		camLight.setDirection(Direction.kForward);
		baseLights = new Relay(Sensors.BASE_RELAY);
		baseLights.setDirection(Direction.kForward);

		// Joysticks
		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);

		// DriverStation.getInstance().getTeamNumber()
		// NetworkTable.initialize();
		// test = NetworkTable.getTable("tester");
		// test.putInt("x", 9);

		gyro = new Gyro(Sensors.GYRO);
		drivetrain = new Drivetrain(this);

		shooterSpeed2 = -.75;
		shooterSpeed1 = .65;
		launchRate = LAUNCH_RATE;

		lWheel1 = new OpticalSensor(Sensors.SHOOTER_WHEEL_1_OPTICAL);
		lWheel2 = new OpticalSensor(Sensors.SHOOTER_WHEEL_2_OPTICAL);

		lWheel1.start();
		lWheel2.start();

		gyro.reset();

		// Tacking?
		tracking = false;
		tracker = new Tracker();
		camLight.set(Value.kOn);

		timePressed = 0;
		lightState = Value.kOn;
		baseLights.set(Value.kOn);

		// File picTmp = new File("TELEOP-" + pic_num + ".png");
		// while(){

		// }

		pic_num = 1;
	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {
			// System.out.println(drivetrain.tiltPot.getValue());
			// s.updateDashboard();
			// System.out.println(drivetrain.tiltPot.getValue());

			/*
			 * try { System.out.println(test.getInt("x")); } catch
			 * (NetworkTableKeyNotDefined e) { // TODO Auto-generated catch
			 * block e.printStackTrace(); }
			 */

			// Joystick 1

			dualJoystickControl();
			// singleJoystickControl();

			if (DriverStation.getInstance().getDigitalIn(1)) {

				if (lrunner == null) {

					if (timePressed > 0) {

						long time = 1000 / timePressed;
						lrunner = new LightRunner(time, baseLights);
						lrunner.start();
					} else {
						baseLights.set(Value.kOff);
					}

				} else if (!lrunner.isAlive()) {
					lrunner = null;
				}
			} else {
				baseLights.set(Value.kOn);
			}

			double forward = regression(joystick1.getRawAxis(2));
			double turn = regression(joystick1.getRawAxis(1));
			if (forward > MAX_DRIVE_SPEED)
				forward = MAX_DRIVE_SPEED;
			if (turn > MAX_TURN_SPEED)
				turn = MAX_TURN_SPEED;

			drivetrain.drive(forward, turn);

			Timer.delay(.01);
		}
	}

	private class LightRunner extends Thread {
		private long wait;
		private Relay relay;

		public LightRunner(long wait, Relay relay) {
			this.wait = wait;
			this.relay = relay;
		}

		public void run() {
			try {
				Thread.sleep(wait);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			Value next = lightState.equals(Value.kOn) ? Value.kOff : Value.kOn;
			lightState = next;
			relay.set(next);
		}
	}

	public void dualJoystickControl() {
		// Take Picture
		if (tracker != null) {
			if (joystick1.getRawButton(2)) {
				tracker.takePicThreaded("TELEOP-" + pic_num + ".png");
				System.out.println("Taking picture");
				pic_num++;
				Timer.delay(.5);
			}

			// Tracking
			if (joystick1.getRawButton(1)) {
				autoTrack();
			}
		}

		// Lifter
		if (joystick1.getRawButton(6)) {
			drivetrain.lift(LIFTER_RATE);
		} else if (joystick1.getRawButton(8)) {
			drivetrain.lift(-LIFTER_RATE);
		} else {
			drivetrain.lift(0);
		}
		// Joystick 2

		// Set Tilt to Normal
		/*
		 * if (joystick2.getRawButton(7)) { drivetrain.moveTilt(TILT_BASE);
		 * System.out.println("Setting tilt to normal"); Timer.delay(.5); }
		 */

		// Shooter
		if (joystick2.getRawButton(3)) {
			drivetrain.setShooter1(shooterSpeed1);
			drivetrain.setShooter2(shooterSpeed2);
			if (lrunner == null || !lrunner.isAlive())
				timePressed++;
		} else {
			drivetrain.setShooter1(0.0);
			drivetrain.setShooter2(0.0);
			timePressed = 0;
		}

		// Launcher
		if (joystick2.getRawButton(1)) {
			drivetrain.fire(launchRate, true);
		} else if (joystick2.getRawButton(2)) {
			drivetrain.fire(-launchRate, true);
		} else {
			drivetrain.fire(0, true);
		}

		// Launch speed increase
		if (joystick2.getRawButton(8)) {
			if (launchRate < 1.0) {
				launchRate += .05;
			}
			System.out.println("Launch speed: " + launchRate);
			Timer.delay(.5);
		}

		// Launch speed decrease
		if (joystick2.getRawButton(9)) {
			if (launchRate > 0) {
				launchRate -= .05;
			}
			System.out.println("Launch speed: " + launchRate);
			Timer.delay(.5);
		}

		// Change speed up
		if (joystick2.getRawButton(11)) {
			if (shooterSpeed1 < 1.0) {
				shooterSpeed1 += .05;
			}

			if (shooterSpeed2 > -1.0) {
				shooterSpeed2 -= .05;
			}

			System.out.println("Shooter speed 1: " + shooterSpeed1
					+ "  Shooter speed 2: " + shooterSpeed2);
			Timer.delay(.5);
		}

		// Change speed down
		if (joystick2.getRawButton(10)) {
			if (shooterSpeed1 > 0) {
				shooterSpeed1 -= .05;
			}

			if (shooterSpeed2 < 0) {
				shooterSpeed2 += .05;
			}

			System.out.println("Shooter speed 1: " + shooterSpeed1
					+ "  Shooter speed 2: " + shooterSpeed2);
			Timer.delay(.5);
		}

		// Tilt
		if (joystick2.getRawButton(4)) {
			drivetrain.tilt(TILT_RATE);
		} else if (joystick2.getRawButton(5)) {
			drivetrain.tilt(-TILT_RATE);
		} else {
			drivetrain.tilt(0);
		}
	}

	public void singleJoystickControl() {
		// Take Picture
		if (joystick1.getRawButton(2)) {
			tracker.takePicThreaded("TELEOP-" + pic_num + ".png");
			System.out.println("Taking picture");
			pic_num++;
			Timer.delay(.5);
		}

		// Tracking
		/*
		 * if (joystick1.getRawButton(1)) { autoTrack(); }
		 */

		// Lifter
		if (joystick1.getRawButton(6)) {
			drivetrain.lift(LIFTER_RATE);
		} else if (joystick1.getRawButton(7)) {
			drivetrain.lift(-LIFTER_RATE);
		} else {
			drivetrain.lift(0);
		}

		// Toggle AutoCam
		/*
		 * if (joystick1.getRawButton(10)) { tracker.toggleAutoPicture();
		 * Timer.delay(.5); }
		 */

		// Joystick 2

		// Set Tilt to Normal
		/*
		 * if (joystick2.getRawButton(7)) { drivetrain.moveTilt(TILT_BASE);
		 * System.out.println("Setting tilt to normal"); Timer.delay(.5); }
		 */

		// Shooter
		if (joystick1.getRawButton(3)) {
			drivetrain.setShooter1(shooterSpeed1);
			drivetrain.setShooter2(shooterSpeed2);
			if (lrunner == null || !lrunner.isAlive())
				timePressed++;
		} else {
			drivetrain.setShooter1(0.0);
			drivetrain.setShooter2(0.0);
			timePressed = 0;
		}

		// Launcher
		if (joystick1.getRawButton(1)) {
			drivetrain.fire(launchRate, true);
		} else if (joystick1.getRawButton(8)) {
			drivetrain.fire(-launchRate, true);
		} else {
			drivetrain.fire(0, true);
		}

		// Launch speed increase DONT CARE
		/*
		 * if (joystick2.getRawButton(3)) { if (launchRate < 1.0) { launchRate
		 * += .05; } System.out.println("Launch speed: " + launchRate);
		 * Timer.delay(.5); }
		 */

		// Launch speed decrease DONT CARE
		/*
		 * if (joystick2.getRawButton(2)) { if (launchRate > 0) { launchRate -=
		 * .05; } System.out.println("Launch speed: " + launchRate);
		 * Timer.delay(.5); }
		 */

		// Change speed up
		if (joystick1.getRawButton(11)) {
			if (shooterSpeed1 < 1.0) {
				shooterSpeed1 += .05;
			}

			if (shooterSpeed2 > -1.0) {
				shooterSpeed2 -= .05;
			}

			System.out.println("Shooter speed 1: " + shooterSpeed1
					+ "  Shooter speed 2: " + shooterSpeed2);
			Timer.delay(.5);
		}

		// Change speed down
		if (joystick1.getRawButton(10)) {
			if (shooterSpeed1 > 0) {
				shooterSpeed1 -= .05;
			}

			if (shooterSpeed2 < 0) {
				shooterSpeed2 += .05;
			}

			System.out.println("Shooter speed 1: " + shooterSpeed1
					+ "  Shooter speed 2: " + shooterSpeed2);
			Timer.delay(.5);
		}

		// Tilt
		if (joystick1.getRawButton(4)) {
			drivetrain.tilt(TILT_RATE);
		} else if (joystick1.getRawButton(5)) {
			drivetrain.tilt(-TILT_RATE);
		} else {
			drivetrain.tilt(0);
		}
	}

	public static double regression(double d) {
		return JOYSTICK_SENSITIVITY * (d * d * d) + (1 - JOYSTICK_SENSITIVITY)
				* d;
	}

	public void autonomous() {

		if (DISABLE_AUTONOMOUS)
			return;

		long start_time = System.currentTimeMillis();
		// drivetrain.moveTilt(TILT_BASE);
		drivetrain.drive(0, 0);
		if (tracking) {

			while (isAutonomous() && isEnabled()) {

				System.out.println("Tracking...");
				ParticleAnalysisReport r = tracker.track(null);

				boolean move = false;

				if (r != null) {

					if (Math.abs(r.center_mass_x_normalized) < TRACKING_ERROR) {
						drivetrain.drive(0, 0);
						break;
					} else if (move) {
						double turn = .3;
						if (r.center_mass_x_normalized < 0)
							turn = -turn;
						drivetrain.drive(0, turn);
					}

					System.out.println("Center: " + r.center_mass_x_normalized);

				}

				Timer.delay(.1);
			}
		}

		drivetrain.setShooter1(shooterSpeed1);
		drivetrain.setShooter2(shooterSpeed2);
		if (tracker != null) {
			tracker.takePicThreaded("AUTO-pic1.png");
			System.out.println("Taking picture");
		}
		Timer.delay(3);

		/*
		 * while (isAutonomous() && isEnabled()) { drivetrain.fire(LAUNCH_RATE,
		 * true); }
		 */

		long time = System.currentTimeMillis();
		while (System.currentTimeMillis() - time < 6000) {
			drivetrain.fire(LAUNCH_RATE, true);
		}

		drivetrain.setShooter1(0);
		drivetrain.setShooter2(0);

		/*
		 * long time = System.currentTimeMillis(); while
		 * (System.currentTimeMillis() - time < 5000) {
		 * drivetrain.fire(LAUNCH_RATE, true); }
		 * 
		 * drivetrain.drive(-.25,0); Timer.delay(3); drivetrain.drive(0,0 );
		 */

		drivetrain.drive(.5, 0);
		Timer.delay(2.2);
		drivetrain.drive(0, -.6);
		Timer.delay(2);
		drivetrain.drive(0, 0);

		System.out.println("Took " + (System.currentTimeMillis() - start_time)
				/ 1000.0 + " seconds.");

		/*
		 * 
		 * try { if (tracker != null) tracker.takePic("pic1.png");
		 * System.out.println("Taking picture"); } catch (Exception e) {
		 * System.out.println("Picture Failed"); }
		 */
	}

	public void autoTrack() {

		System.out.println("Tracking...");

		while (this.joystick1.getRawButton(1)) {

			ParticleAnalysisReport r = tracker.track(null);

			if (r != null) {

				if (Math.abs(r.center_mass_x_normalized) < TRACKING_ERROR) {
					drivetrain.drive(0, 0);
					System.out.println("DONE!");
					break;
				} else {
					double turn = .3;
					if (r.center_mass_x_normalized < 0)
						turn = -turn;
					drivetrain.drive(0, turn);
				}

				System.out.println("Center: " + r.center_mass_x_normalized);

			} else
				System.out.println("Didnt find anything!");

			Timer.delay(.1);
		}
	}
}