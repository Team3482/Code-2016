package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.Robot;
import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class Chassis extends Subsystem implements PIDOutput {

    public final AnalogInput rangeFinder = RobotMap.chassisRangeFinder;
    public final AHRS imu = RobotMap.chassisIMU;
    private final CANTalon frontLeft = RobotMap.chassisfrontLeft;
    private final CANTalon backLeft = RobotMap.chassisbackLeft; //encoder
    private final CANTalon frontRight = RobotMap.chassisfrontRight; 
    private final CANTalon backRight = RobotMap.chassisbackRight; //encoder
    private final RobotDrive robotDrive41 = RobotMap.chassisRobotDrive41;
    PIDController rotateController;

    boolean dirFrontLeft = false;
    boolean dirRearLeft = false;
    boolean dirFrontRight = false;
    boolean dirRearRight = false;
    StringBuilder sb = new StringBuilder();
	int loopsL = 0;
	int loopsR = 0;
    double driveWheelDiameter = 8; //inches
    double driveWheelCircumference = driveWheelDiameter*Math.PI;
    double driveWheelGearRatio = 12.75;
    double rotateToAngleRate;
    double maxTurningSpeed = .75;
    
    public final double holdSpeed = .25;
    
/*    int[] transferFunctionLUT5V =
    	{
    		255, 127, 93, 77, 67, 60, 54, 50, 47, 44, 42, 40, 38, 36, 35, 34,
    		32, 31, 30, 30, 29, 28, 27, 27, 26, 26, 25, 25, 24, 22, 20, 19,
    		19, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 14, 13,
    		13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10,
    		10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7,
    		7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6,
    		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    		6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    		5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    	};
  */  
    
    public Chassis() {
    	backLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	backLeft.reverseSensor(false);
    	backLeft.configEncoderCodesPerRev(250);
    	/*backLeft.configNominalOutputVoltage(+0f, -0f);
    	backLeft.configPeakOutputVoltage(+12f, -12f);
        
    	backLeft.setAllowableClosedLoopErr(0);
    	backLeft.setProfile(0);
    	backLeft.setF(0);
    	backLeft.setP(1.5);
    	backLeft.setI(0);
    	backLeft.setD(0);
    	    	
    	
    	backRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	backRight.reverseSensor(false);
    	backRight.configEncoderCodesPerRev(250);
    	backRight.configNominalOutputVoltage(+0f, -0f);
    	backRight.configPeakOutputVoltage(+12f, -12f);
        
    	backRight.setAllowableClosedLoopErr(0);
    	backRight.setProfile(0);
    	backRight.setF(0);
    	backRight.setP(1.5);
    	backRight.setI(0);
    	backRight.setD(0);*/

    	rotateController = new PIDController(0.01, 0, 0, 0, imu, this);
    	rotateController.setInputRange(-180.0f,  180.0f);
    	rotateController.setOutputRange(-0.5, 0.5);
        rotateController.setAbsoluteTolerance(2f);
        rotateController.setContinuous(true);
    }
        
    public void invertMotors() {
    	robotDrive41.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, !dirFrontLeft);
    	robotDrive41.setInvertedMotor(RobotDrive.MotorType.kRearLeft, !dirRearLeft);
    	robotDrive41.setInvertedMotor(RobotDrive.MotorType.kFrontRight, !dirFrontRight);
    	robotDrive41.setInvertedMotor(RobotDrive.MotorType.kRearRight, !dirRearRight);
    	dirFrontLeft = !dirFrontLeft;
    	dirRearLeft = !dirRearLeft;
    	dirFrontRight = !dirFrontRight;
    	dirRearRight = !dirRearRight;
    	System.out.println("inverted");
    }
    
    public void driveWithJoystick(Joystick s) {
		double deadZone = 0.0;
		double xAxis = s.getAxis(Joystick.AxisType.kX);
		double yAxis = s.getAxis(Joystick.AxisType.kY);
		//System.out.println("xaxis: " + xAxis);
		//System.out.println("yaxis: " + yAxis);
		
		// X sensitivity set by slider, Y sensitivity set by knob
		//double slider = SmartDashboard.getNumber("Slider 1");
		 //xAxis *= (slider / 100);
		//double knob = s.getAxis(Joystick.AxisType.kZ);
		//knob = 1 - (knob / 2);    // Format input from Z Axis
		//yAxis *= knob;
		// If the X or Y axes are in the deadzone, flip them to zero.
		if (xAxis < deadZone && xAxis > -deadZone) {
			xAxis = 0;
		}
		if (yAxis < deadZone && yAxis > -deadZone) {
			yAxis = 0;
		}
		robotDrive41.arcadeDrive(yAxis, xAxis);
	}
    
	//drives the robot with a joystick - xbox configuration
	public void driveWithXboxController(Joystick s) {
		double leftY = s.getRawAxis(1);
		double rightX = s.getRawAxis(4);
		//System.out.println("xaxis: " + rightX);
		//System.out.println("yaxis: " + leftY);
		double deadZone = 0.1;

		if (leftY < deadZone && leftY > -deadZone) {
			leftY = 0;
		}
		if (rightX < deadZone && rightX > -deadZone) {
			rightX = 0;
		}
		if(leftY == 0 && rightX == 0) {
			return;
		}
		robotDrive41.arcadeDrive(leftY, rightX*maxTurningSpeed);
	}
	
	public void imuData() {
		Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */

        /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(   "IMU_FusedHeading",     imu.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         imu.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       imu.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       imu.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        SmartDashboard.putNumber(   "Velocity_X",           imu.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           imu.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       imu.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       imu.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        SmartDashboard.putNumber(   "RawGyro_X",            imu.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            imu.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            imu.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           imu.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           imu.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           imu.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             imu.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             imu.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             imu.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = imu.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        SmartDashboard.putString(   "FirmwareVersion",      imu.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(   "QuaternionW",          imu.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          imu.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          imu.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          imu.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
	}
	
	public void PIDMove(double rotationsL, double rotationsR) {
		double motorOutput = backLeft.getOutputVoltage()/backLeft.getBusVoltage();
    	sb.append("Left:");
    	sb.append("\tout:");
	  	sb.append(motorOutput);
	  	sb.append("\tpos:");
        sb.append(backLeft.getPosition() );
    	sb.append("\terrNative:");
    	sb.append(backLeft.getClosedLoopError());
    	sb.append("\ttrg:");
    	sb.append(rotationsL);
    	if(++loopsL >= 10) {
          	loopsL = 0;
          	System.out.println(sb.toString());
        }
        sb.setLength(0);
        
        motorOutput = backRight.getOutputVoltage()/backRight.getBusVoltage();
    	sb.append("Right:");
    	sb.append("\tout:");
	  	sb.append(motorOutput);
	  	sb.append("\tpos:");
        sb.append(backRight.getPosition() );
    	sb.append("\terrNative:");
    	sb.append(backRight.getClosedLoopError());
    	sb.append("\ttrg:");
    	sb.append(rotationsR);
    	if(++loopsR >= 10) {
          	loopsR = 0;
          	System.out.println(sb.toString());
        }
        sb.setLength(0);
        
        backLeft.set(rotationsL);
        backRight.set(rotationsR);
        frontLeft.set(backLeft.getOutputVoltage());
        frontRight.set(backRight.getOutputVoltage());
	}
	
	//moves the robot to a location
	public void move(double moveValue, double rotateValue) {
		robotDrive41.arcadeDrive(-moveValue, rotateValue);
	}
	//Moves the robot a certain distance (inches) forward
	public void moveDistance() {
		robotDrive41.arcadeDrive(0.4, 0.0);
	} 
	
	public void stopMoving() {
		robotDrive41.arcadeDrive(0.0, 0.0);
	}
	
	//Rotates the robot by a given degrees
	public void rotateByAngle(double degrees) {
		double currentAngle = imu.getYaw();
		double absoluteAngle = currentAngle + degrees;
		if(absoluteAngle > 180) {
			double diff = absoluteAngle-180;
			absoluteAngle = -180 + diff;
		} else if(absoluteAngle < -180) {
			double diff = absoluteAngle+180;
			absoluteAngle = 180 + diff;
		}
		rotateToAngle(absoluteAngle);
	}
	
	public void setUpPID() {
		// Tuned with a 12.7 Volt battery
		double max = SmartDashboard.getNumber("Maximum Turn Output");
		rotateController.setOutputRange(-max, max);
		double P = SmartDashboard.getNumber("Gyro P", 0.01);
		double I = SmartDashboard.getNumber("Gyro I", 0.001) / 10.0;
		double D = SmartDashboard.getNumber("Gyro D", 0.0);
		//P = P / Math.abs(rotateController.getError());
		rotateController.setPID(P, I, D);
		/*if (Math.abs(rotateController.getError()) < 2) {
			rotateController.setPID(0.4, 0, 0);
			rotateController.setOutputRange(-0.45, 0.45);
		} else if (Math.abs(rotateController.getError()) < 5) { 
			rotateController.setPID(0.35, 0, 0);
			rotateController.setOutputRange(-0.45, 0.45);
		} else if (Math.abs(rotateController.getError()) < 12) { 
			rotateController.setPID(0.25, 0, 0);
			rotateController.setOutputRange(-max, max);
		} else if (Math.abs(rotateController.getError()) < 25) {
			rotateController.setPID(0.08, 0, 0);
			rotateController.setOutputRange(-max, max);
		} else {
			rotateController.setPID(0.02, 0, 0);
			rotateController.setOutputRange(-max, max);
		}*/
			System.out.print("rotate error" + rotateController.getError());
			System.out.print("rotating" + rotateToAngleRate);
			System.out.println("Angle: " + imu.getAngle());
	}
	
	//Rotates the robot to a given degrees (-180 to 180)
	public void rotateToAngle(double degrees) {
		enableRotation();
		rotateController.setSetpoint(degrees);
		setUpPID();
		move(0.0, rotateToAngleRate);
		//System.out.println(rotateController.getAvgError());
		//System.out.println("angle"+imu.getYaw()+" setpoint"+rotateController.getSetpoint());
		//move(0.0,0.0);
	}
	
	public void moveStraight(double speed, double targetRotations, double degrees) {
		enableRotation();
		double rotationsL = backLeft.getPosition();
		double error = (targetRotations - rotationsL);
		System.out.println("error: " + error);
		rotateController.setSetpoint(degrees);
		setUpPID();
//		if (Math.abs(error) <= 0.5)
//			speed *= error*2;
		if (Math.abs(error) < 0.1)
			move(0.0, rotateToAngleRate);
		else if (error > 0)
			move(speed, rotateToAngleRate);
		else if (error < 0)
			move(-speed, rotateToAngleRate);
		else
			move(0.0, 0.0);
		//move(0.0,0.0);
	}
	
	public double getRangeFinderVoltage() {
		return rangeFinder.getAverageVoltage();
	}
	
	//.37 for shoot, acceptable error of .01
	public void maintainDistanceVoltage(double targetDistanceVoltage, double degrees, double acceptableError, boolean hold) {
		enableRotation();
		double currentVoltage = rangeFinder.getAverageVoltage();
		//double estimateCurrentInches = transferFunctionLUT5V[rangeFinder.getValue()/4/4];
		double error = (targetDistanceVoltage - currentVoltage);
		System.out.println("error: " + error);
		rotateController.setSetpoint(degrees);
		setUpPID();
//		if (Math.abs(error) <= 0.5)
//			speed *= error*2;
		if (Math.abs(error) < acceptableError) {
			if(hold) {
			move(holdSpeed + 0.1, 0.0); //move(0.0, rotateToAngleRate);
			} else {
				move(0,0);
			}
		} else if (error > 0) {
			move(.6, 0.0); //move(.6, rotateToAngleRate); //.6
		} else if (error < 0) {
			move(0, 0.0); //move(0, rotateToAngleRate);
		} else {
			move(0.0, 0.0);
		}
		//move(0.0,0.0);
	}
	
	//inches to rotations
	public double distanceToTargetRotations(double distance) {
		return distance/driveWheelCircumference + backLeft.getPosition();
	}
	
	public void resetGyro() {
		imu.reset();
	}
	
	public double getRotateRate() {
		return rotateToAngleRate;
	}
	
	public double getTargetAngle() {
		return rotateController.getSetpoint();
	}
	
	public double getOutput() {
		return rotateToAngleRate;
	}
	public double getCurrentAngle() {
		return imu.getYaw();
	}
	
	public void printRotateInfo() {
		System.out.println("rotate error" + rotateController.getError());
		System.out.println("rotating" + rotateToAngleRate);
	}
	
	public void enableRotation() {
		rotateController.enable();
	}
	
	public void disableRotation() {
		rotateController.disable();
	}
	
	//stops the robot
	public void stop() {
		robotDrive41.stopMotor();
	}
	
	//sets the safety
	public void setSafety(boolean n) {
		robotDrive41.setSafetyEnabled(n);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}

