/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * 
 * @author jason
 */
public class Drivetrain {

	private Jaguar left1;
	private Jaguar left2;
	private Jaguar right1;
	private Jaguar right2;
	private Jaguar shooter1;
	private Jaguar shooter2;
	private Jaguar lifter;
	private final double LAUNCHER_SPEED = .50;
	private final long LAUNCHER_TIME = 440;
	private Victor tilter;
	private Victor feeder;
	private boolean firing, timed;
    public DigitalInput liftLow, liftHigh, launchLimit;
	

	public Drivetrain(boolean timedFire) {
        shooter1 = new Jaguar(1);
        shooter2 = new Jaguar(2);
        feeder = new Victor(9);
        left1 = new Jaguar(6);
        left2 = new Jaguar(7);
        right1 = new Jaguar(4);
        right2 = new Jaguar(5);
        lifter = new Jaguar(8);
        tilter = new Victor(10);
		firing = false;
		timed = timedFire;
		
        liftLow = new DigitalInput(1);
        liftHigh = new DigitalInput(2);
        launchLimit = new DigitalInput(3);
	}

	public void drive(double throttle, double turn) {
		double left = 0.0;
		double right = 0.0;

		left = throttle - turn;
		right = -throttle - turn;

		tankDrive(left, right);
	}

	public void setShooter1(double d) {
		shooter1.set(d);
	}

	public void setShooter2(double d) {
		shooter2.set(d);
	}

	public void tankDrive(double left, double right) {
		left1.set(left);
		left2.set(left);
		right1.set(right);
		right2.set(right);
	}

	public void fire(double rate) {
		if (!timed) {
			feeder.set(rate);
		} else {
			if (firing)
				return;
			else
				firing = true;

			Thread t = new Thread() {
				public void run() {
					feeder.set(LAUNCHER_SPEED);
					Timer.delay(.5);
					while(!liftLow.get());
					feeder.set(0);
					firing = false;
				}
			};
			t.start();
		}
	}
	
	public void tilt(double rate){
		tilter.set(rate);
	}
	
	public void lift(double rate){
		if(rate > 0 && !liftHigh.get() || rate < 0 && !liftLow.get() || rate == 0)
			lifter.set(rate);
		else lifter.set(0);
	}
	
	
}
