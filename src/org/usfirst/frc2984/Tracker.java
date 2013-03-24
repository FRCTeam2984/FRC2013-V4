package org.usfirst.frc2984;

import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.RGBImage;

public class Tracker {
	
    AxisCamera camera;
    CriteriaCollection cc;
    public int redHigh, redLow, blueHigh, blueLow, greenHigh, greenLow;
	
	public Tracker(){
		camera = AxisCamera.getInstance();
		camera.writeCompression(0);
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        //cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
        //cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false); 
        
        redHigh = 256;
        blueHigh = 256;
        greenHigh = 256;
        redLow = 215;
        blueLow = 215;
        greenLow = 215;
	}
	
	public ParticleAnalysisReport track(String file){
		try {
			
			
            ColorImage image = (file != null ? new RGBImage(file) : camera.getImage());
            BinaryImage thresholdImage = image.thresholdRGB(redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh);   // keep only green objects
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
			// TODO Auto-generated catch block
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
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (NIVisionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
