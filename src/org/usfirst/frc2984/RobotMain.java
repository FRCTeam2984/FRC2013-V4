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
	public final static double TILT_RATE = .2;
	public final static double LAUNCH_RATE = .5;
	public static final double LIFTER_RATE = .98;
	public static final int TILT_BASE = 428; //526
	private double shooterSpeed1, shooterSpeed2, launchRate;
	private boolean tracking;
	Station s;

	public void robotInit() {
		
		s = new Station();
		
		
		// Relays
		camLight = new Relay(Sensors.SHOOTER_RELAY);
		camLight.setDirection(Direction.kForward);
		baseLights = new Relay(Sensors.BASE_RELAY);
		baseLights.setDirection(Direction.kForward);

		// Joysticks
		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);

		gyro = new Gyro(Sensors.GYRO);
		drivetrain = new Drivetrain();

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
		//tracker = new Tracker();
		camLight.set(Value.kOn);
		baseLights.set(Value.kOn);
	}

	public void operatorControl() {
		
		while (isOperatorControl() && isEnabled()) {
			//System.out.println(drivetrain.tiltPot.getValue());
			s.updateDashboard();
			//System.out.println(s.printOutput());
			
			// Set Tilt to Normal
			if (joystick2.getRawButton(7)) {
				drivetrain.moveTilt(TILT_BASE);
				System.out.println("Setting tilt to normal");
				Timer.delay(.5);
			}

			// Shooter
			if (joystick2.getRawButton(5)) {
				drivetrain.setShooter1(shooterSpeed1);
				drivetrain.setShooter2(shooterSpeed2);
			} else {
				drivetrain.setShooter1(0.0);
				drivetrain.setShooter2(0.0);
			}

			// Launcher
			if (joystick2.getRawButton(1)) {
				drivetrain.fire(launchRate, true);
			} else if (joystick2.getRawButton(4)) {
				drivetrain.fire(-launchRate, true);
			} else{ drivetrain.fire(0, true); }
			 

			// Lifter
			if (joystick1.getRawButton(6)) {
				drivetrain.lift(LIFTER_RATE);
			} else if (joystick1.getRawButton(8)) {
				drivetrain.lift(-LIFTER_RATE);
			} else {
				drivetrain.lift(0);
			}

			// Launch speed increase
			if (joystick2.getRawButton(3)) {
				if (launchRate < 1.0) {
					launchRate += .05;
				}
				System.out.println("Launch speed: " + launchRate);
				Timer.delay(.5);
			}
			
			
			// Launch speed decrease
			if (joystick2.getRawButton(2)) {
				if (launchRate > 0) {
					launchRate -= .05;
				}
				System.out.println("Launch speed: " + launchRate);
				Timer.delay(.5);
			}

			// Change speed up
			if (joystick2.getRawButton(10)) {
				if (shooterSpeed1 > -1.0) {
					shooterSpeed1 -= .1;
					shooterSpeed2 -= .1;
				}
				System.out.println("Shooter speed: " + shooterSpeed2);
				Timer.delay(.1);
			}

			// Change speed down
			if (joystick2.getRawButton(9)) {
				if (shooterSpeed1 < 0) {
					shooterSpeed1 += .1;
					shooterSpeed2 += .1;
				}
				System.out.println("Shooter speed: " + shooterSpeed2);
				Timer.delay(.1);
			}

			// Tilt
			if (joystick2.getRawButton(8)) {
				drivetrain.tilt(TILT_RATE);
			} else if (joystick2.getRawButton(6)) {
				drivetrain.tilt(-TILT_RATE);
			} else {
				drivetrain.tilt(0);
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

	public static double regression(double d) {
		return JOYSTICK_SENSITIVITY * (d * d * d) + (1 - JOYSTICK_SENSITIVITY) * d;
	}

	public void autonomous() {
		drivetrain.moveTilt(TILT_BASE);
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
		Timer.delay(3);
			
		while (isAutonomous() && isEnabled()) {
			drivetrain.fire(LAUNCH_RATE, true);
		}
			
		/*
		long time = System.currentTimeMillis();
		while (System.currentTimeMillis() - time < 5000) {
			drivetrain.fire(LAUNCH_RATE, true);
		}
		
		drivetrain.drive(-.25,0);
		Timer.delay(3);
		drivetrain.drive(0,0
		);
		*/

		
		try {
			if(tracker != null) tracker.takePic("pic1.png");
		} catch (Exception e){
			System.out.println("Picture Failed");
		}
	}
}