package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCamera.ResolutionT;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.RGBImage;

public class Tracker {
	
    AxisCamera camera;
    CriteriaCollection cc;
    AutomatedCamera ac;
    public int redHigh, redLow, blueHigh, blueLow, greenHigh, greenLow;
	
	public Tracker(){
		camera = AxisCamera.getInstance();
		camera.writeResolution(ResolutionT.k640x480);
		camera.writeCompression(0);
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        //cc.addCriteria(MeasurementType, lower, upper, outsideRange)
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 50, 300, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 20, 120, false); 
        
        redHigh = 256;
        blueHigh = 256;
        greenHigh = 256;
        redLow = 0;
        blueLow = 0;
        greenLow = 250;
	}
	
	public ParticleAnalysisReport track(String file){
		try {
			
			
            ColorImage image = (file != null ? new RGBImage(file) : camera.getImage());
            BinaryImage thresholdImage = image.thresholdRGB(redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh);   // keep only green objects
            //thresholdImage.write("thresh.bmp");
            BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 2);  // remove small artifacts
            BinaryImage convexHullImage = bigObjectsImage.convexHull(false);          // fill in occluded rectangles
            BinaryImage filteredImage = convexHullImage.particleFilter(cc);// find filled in rectangles
            
            ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();  // get list of results
            
            /*for (int i = 0; i < reports.length; i++) {                                // print results
                ParticleAnalysisReport r = reports[i];
                System.out.println("Particle: " + i + ":  Center of mass x: " + r.center_mass_x);
            }
            System.out.println(filteredImage.getNumberParticles() + "  " + Timer.getFPGATimestamp());*/
            
            ParticleAnalysisReport close = null;
            for (int i = 0; i < reports.length; i++) {
                if(close == null)
                	close = reports[i];
                else {
                	if(close.center_mass_x_normalized > reports[i].center_mass_x_normalized)
                		close = reports[i];
                }
            }
            
            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            thresholdImage.free();
            image.free();
            
            return close;
            
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        } catch (AxisCameraException e) {
			e.printStackTrace();
		}
		
		return null;
	}
	
	public void takePic(String fileName){
		
		ColorImage i;
		try {
			i = camera.getImage();
			i.write(fileName);
			i.free();
		} catch (AxisCameraException e) {
			e.printStackTrace();
		} catch (NIVisionException e) {
			e.printStackTrace();
		}
	}
	
	public void takePicThreaded(final String fileName){
		
		new Thread(){
			public void run(){
				takePic(fileName);
			}
		}.start();
	}
	
	
	public void toggleAutoPicture(){
		if(ac != null){
			ac.stop();
			ac = null;
			
			System.out.println("AutoCam murdered!");
		}else {
			ac = new AutomatedCamera(1);
			ac.start();
			System.out.println("AutoCam started!");
		}
	}
	
	private class AutomatedCamera extends Thread {
		
		public final int freq;
		private boolean stop;
		
		public AutomatedCamera(int freq){
			this.freq = freq;
			stop = false;
		}
		
		public void stop(){
			stop = true;
		}
		
		public void run(){
			int num = 1;
			while(!stop){
				
				System.out.println("Saving pic " + "Automated-" + num + ".png");
				takePic("Automated-" + num + ".png");
				num++;
				Timer.delay(freq);
			}
		}
		
	}
}
