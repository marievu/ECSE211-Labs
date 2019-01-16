package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/*PController changes the speed of the robot when turning right or left proportional to a constant
 *which depends on the error. The bigger the error, the bigger the correction.
 *
 */

public class PController implements UltrasonicController {

	  /* Constants */
	  private static final int MOTOR_SPEED = 200;
	  private static final int FILTER_OUT = 20;

	  private final int bandCenter;
	  private final int bandWidth;
	  private int distance;
	  private int filterControl;

	  public PController(int bandCenter, int bandwidth) {
	    this.bandCenter = bandCenter;
	    this.bandWidth = bandwidth;
	    this.filterControl = 0;

	    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
	    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	    WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
	  }

	  @Override
	  public void processUSData(int distance) {

	    // rudimentary filter - toss out invalid samples corresponding to null
	    // signal.
	    // (n.b. this was not included in the Bang-bang controller, but easily
	    // could have).
	    //
	    if (distance >= 255 && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } else if (distance >= 255) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = distance;
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      this.distance = distance;
	    }

	    //difference between bandCenter and distance 
	    int distError = bandCenter - distance;
	    
	    //scaling p so that there is a proportional p constant 
	    int p = Math.abs(8*distError);
	    
	   //capping the max p 
	    if (p > 255) {
	    	
	    	p = 255;
	    	
	    }
	    
	    //if too close to the wall reverse 
	    if (distance <= 14) {
	    	
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 100);
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 200);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.backward();
	    
	    //if in range of the proper bandWidth go straight 
	    } else if (Math.abs(distError) <= bandWidth) {
	    	
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.forward();
	    
	    //if too close to the wall turn right proportional to the p value 
	    } else if (distError > 0){
	    	
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + p);
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.forward();
	    
	    
	    } 
	  //if too far from the wall turn left proprotional to the p value 
	    else if (distError < 0) {
	    	
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + p);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.forward();
	    	
	    }
	    
	    
	  }


	  @Override
	  public int readUSDistance() {
	    return this.distance;
	  }

	}
