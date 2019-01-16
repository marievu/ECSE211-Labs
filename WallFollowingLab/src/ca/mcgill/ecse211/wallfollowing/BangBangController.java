package ca.mcgill.ecse211.wallfollowing;

import org.jfree.chart.plot.ThermometerPlot;
import org.jfree.data.xy.IntervalXYDataset;

import lejos.hardware.motor.*;


/*  BangBangController takes in the distance recorded by the EV3 then calculates an error according to how 
 * far away the distance is from the bandCenter. It will then make the appropriate adjustments in order to keep 
 * the EV3 with in the given bandwidth. 
 * 
 * 
 * 
 */
public class BangBangController implements UltrasonicController {

	private final int bandCntr;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private static final int FILTER_OUT = 40;
	private int distError; //to detect the difference between bandwidth and the distance the EV3 is measuring 
	private int distance; 
	private int sleepInt= 50;  
	private int toClose= 180; //if the EV3 ever gets to close to the wall it will try to compensate by moving quicker then motorlow away from the wall
	private int filterControl;
	
	public BangBangController(int bandCntr, int bandwidth, int motorLow, int motorHigh) 
	{
		
		// Default Constructor
		this.filterControl =0;
		this.bandCntr = bandCntr;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

  @Override
  public void processUSData(int distance)
  {
	  if (distance >= 100  && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } else if (distance >= 100) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = distance;
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      this.distance = distance;
	    }

    this.distance = distance;
    
    distError=bandCntr-distance; 

    //if the EV3 is within the bandwidth, keep moving forward
    if (Math.abs(distError) <= bandwidth) 
    { 
    		WallFollowingLab.leftMotor.setSpeed(motorHigh); 
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    }
    //if the EV3 distance is less than 9 away from the wall make both wheels go backwards.
    //However the left motor should move back slower than the right
    else if (distance <= 14)
    {
    		WallFollowingLab.leftMotor.setSpeed(toClose); 
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.backward();
		WallFollowingLab.rightMotor.backward();
    }
    
   
 
    //if the EV3 is to close to the wall then move further away until within the proper range of the bandwitdth 
    else if (distError > 0) 
    { 
    	 	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	 	WallFollowingLab.rightMotor.setSpeed(motorLow);
    	 	WallFollowingLab.leftMotor.forward();
    	 	WallFollowingLab.rightMotor.backward();
    }  

   
    //if the EV3 is to far from the wall then move closer to the wall until within the proper range of bandwidth
    else if (distError<0)
    {
    		WallFollowingLab.leftMotor.setSpeed(motorLow);
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    }
    try {
		Thread.sleep(sleepInt);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	} // Allow other threads to get CPU

  }
   	
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  @Override
  public int readUSDistance() {
    return this.distance;
  }
}