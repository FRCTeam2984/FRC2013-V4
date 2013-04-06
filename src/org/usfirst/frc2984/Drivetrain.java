/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * 
 * @author The Great Francesco
 */
public class Drivetrain {

	private Jaguar left1;
	private Jaguar left2;
	private Jaguar right1;
	private Jaguar right2;
	private Jaguar shooter1;
	private Jaguar shooter2;
	private Jaguar lifter;
	private Victor tilter;
	private Victor feeder;
	private boolean firing;
	
    public DigitalInput liftLow, liftHigh, launchLimit;
	public AnalogChannel tiltPot;
	private RobotMain rm;
	
	public final static int TILT_HIGH = 930;
	public final static int TILT_LOW = 300;
	

	public Drivetrain(RobotMain robotMain) {
		
		rm = robotMain;
		
        shooter1 = new Jaguar(1);
        shooter2 = new Jaguar(2);
        feeder = new Victor(9);
        left1 = new Jaguar(6);
        left2 = new Jaguar(7);
        right1 = new Jaguar(3);
        right2 = new Jaguar(4);
        //Jag 5 is broken
        lifter = new Jaguar(8);
        tilter = new Victor(10);
		firing = false;
		
        liftLow = new DigitalInput(Sensors.LIFT_LOW);
        liftHigh = new DigitalInput(Sensors.LIFT_HIGH);
        launchLimit = new DigitalInput(Sensors.LAUNCHER_OPTICAL);
        
        tiltPot = new AnalogChannel(Sensors.TILT_POTENTIOMETER);
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

	public void fire(final double rate, boolean timed) {
		if (!timed) {
			feeder.set(rate);
		} else {
			if (firing || rate == 0)
				return;
			else
				firing = true;

			Thread t = new Thread() {
				public void run() {
					feeder.set(rate);
					Timer.delay(.18);
					long time = System.currentTimeMillis();
					while(!launchLimit.get() && System.currentTimeMillis() - time < 2000 && !(rate < 0 ? rm.joystick2.getRawButton(1) : rm.joystick2.getRawButton(4)));
					feeder.set(0);
					Timer.delay(.6);
					firing = false;
				}
			};
			t.start();
		}
	}
	
	public boolean tilt(double rate){
		if(rate < 0 && tiltPot.getValue() > TILT_LOW || rate > 0 && tiltPot.getValue() < TILT_HIGH || rate == 0){
			tilter.set(rate);
			return true;
		} else {
			tilter.set(0);
			return false;
		}
	}
	
	public void lift(double rate){
		if(rate > 0 && !liftHigh.get() || rate < 0 && !liftLow.get() || rate == 0)
			lifter.set(rate);
		else lifter.set(0);
	}
	
	public void moveTilt(final int potVal){
		if(potVal > TILT_HIGH || potVal < TILT_LOW)
			return;
		final double rate = (potVal - tiltPot.getValue() < 0 ? -RobotMain.TILT_RATE : RobotMain.TILT_RATE);
		tilt(rate);
		new Thread(){
			public void run(){
				while(Math.abs(tiltPot.getValue() - potVal) > 4)
					tilt(rate);
				tilt(0);
			}
		}.start();
	}
}