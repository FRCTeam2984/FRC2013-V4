package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Timer;

public class OpticalSensor extends Counter{
	
	private int rate;
	private RateTracker t;
	
	public OpticalSensor(int port){
		super(port);
		super.setUpSourceEdge(true, false);
		super.setSemiPeriodMode(true);
		super.setMaxPeriod(1);
		t = new RateTracker(this);
		
	}
	
	public void start(){
		super.start();
		t.start();
	}
	
	public void stop(){
		super.stop();
		t.stop();
	}
	
	/**
	 * Gets the rate per second
	 * @return
	 */
	public int getRate(){
		return rate;
	}
	
	private class RateTracker extends Thread {
		
		private boolean testing;
		private OpticalSensor sensor;
		
		public RateTracker(OpticalSensor o){
			testing = false;
			sensor = o;
		}
		
		public void stop(){
			testing = false;
		}

		public void run(){
			testing = true;
			long time = System.currentTimeMillis();
			while(testing){
				sensor.reset();
				Timer.delay(.1);
				sensor.rate = sensor.get() * 10;
			}
		}
	}
}
