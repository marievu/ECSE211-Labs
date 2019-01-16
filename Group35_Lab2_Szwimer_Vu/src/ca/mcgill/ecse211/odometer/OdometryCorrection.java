/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * The OdometryCorrection class with the aid of the color sensor will detect when it sees lines that correspond to grid markings.
 * Not knowing the exact point in which the EV3 is placed, the y origin is set after it crosses the first black line detected while the x origin is set to zero
 * when it crosses the fourth sensed black line. This will result in having a value displayed upon returning to the spot in which the robot started (the real (0,0))
 *  and hence the correction is displayed
 * 
 */

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private double theta;
  private double Tile_size = 30.48; 
  private int lineCounterY = 0;  //counter for y-axis increments every time sensor detect a black line
  private int lineCounterX = 0;  //same as lineCounterY but for x-axis
  
  
  //setup the color sensor
  private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
  SampleProvider usSensor = colorSensor.getMode("Red");
  float [] usData = new float [usSensor.sampleSize()];
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    float intensity;
    Sound.setVolume(Sound.VOL_MAX);
    

    while (true) {
      correctionStart = System.currentTimeMillis();
      
      usSensor.fetchSample(usData, 0);
      intensity = usData[0];
  
      //if the light intensity is less 0.27 meaning if it is detecting a black line then do the following
      
        if (intensity <  0.27)
        {
  
        //fetches the theta value of the odometer
      	double [] getTheta = odometer.getXYT(); 
      	theta = getTheta[2];
      	
      	// correcting Y when theta ~ 0 or theta ~360
      	if((theta < 20 && theta > 0) || (theta > 340 && theta < 360)) {

      		odometer.setY(lineCounterY*Tile_size);
      		lineCounterY++;
      	}
      	//correcting X when theta ~ 90
      	else if(theta <100 && theta>80) {
      		
      		odometer.setX(lineCounterX*Tile_size);
      		lineCounterX++;
      	}
      	//correcting Y when theta ~ 180
      	else if(theta <190 && theta > 100) {
      		
      		odometer.setY((lineCounterY-1)*Tile_size);
      		lineCounterY--;
      	}
      	//correcting X when theta ~270
      	else if(theta <280 && theta>260) {
      		odometer.setX((lineCounterX-1)*Tile_size);
      		lineCounterX--;
      	}
      	Sound.beep();
        }
        
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
