package org.usfirst.frc.team8080.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive robot = new RobotDrive(0,1,2,3);
	
	
	
	
	Joystick stick = new Joystick(0);
	Timer timer = new Timer();
	SmartDashboard dashboard = new SmartDashboard();
	
	
	Spark leftShooter = new Spark(4); //Spark is the motor controller brand. 4 is the port plugged into the RIO.
	Spark rightShooter = new Spark(5);//5 is the port plugged into the RIO.
	Boolean rightShoot = false;
	Boolean leftShoot = false;
	Boolean allShoot = false;
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	
	Boolean retractRope = false;
	Boolean catchRope = false;
	Victor leftClimber = new Victor(7);
	Victor rightClimber = new Victor(8);
	
	Spark ballCollector = new Spark(6);
	Boolean collectBalls = false;
	
	Boolean seeColor = false;
	Boolean enableColor = false;
	
	Boolean halfSpeed = true;
	Boolean buckAndShoot = false;
	
	double power = 1;
	
	double velX = 0;
	double velY = 0;

	double sensitivity = .6;
	
	boolean teleop = false;
	boolean record = false;
	long interval = 1;
	int index = -1;
	
	Thread visionThread;
	
	
	boolean ballPusherForward = false;
	boolean ballPusherBackward = false;
	Spark ballPusher = new Spark(9);
	
	
	Command[] centerCommands = {
		new Command("drive", 5),  //2.83
	};
	
	

	
	//left
	Command[] commands = {
		new Command("drive", 1.2),
		new Command("rotate", 60),
		new Command("drive", 1.79),
	};
	
	long metersPerSecond = 1;
	long anglesPerSecond = 30;
	
	boolean autoShoot = false;
	
	
	
	
	/*Spark zero = new Spark(0);
	Spark one = new Spark(1);
	Spark two = new Spark(2);
	Spark three = new Spark(3);
	Spark four = new Spark(4);
	Spark five = new Spark(5);
	Spark six = new Spark(6);
	Spark seven = new Spark(7);
	Spark eight = new Spark(8);
	Spark nine = new Spark(9);*/
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		gyro.reset();
		
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Debug", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				
				
				double[] color = mat.get(480/2, 640/2);
				double[] hsv = rgb2hsv(color[2], color[1], color[0]);
				
				
				Imgproc.putText(mat, "hue: " + hsv[0], new Point(0,50), Core.FONT_HERSHEY_PLAIN, 3, new Scalar(0,255,0));
				
				Imgproc.putText(mat, "seeColor: " + seeColor, new Point(0,100), Core.FONT_HERSHEY_PLAIN, 3, new Scalar(0,0,255));
				
				Imgproc.line(mat, new Point(590,0), new Point(590,500), new Scalar(0,255,0));
				if(hsv[0] <=  15 || hsv[0] > 340){
					seeColor = true;
				}else{
					seeColor = false;
				}
				
				// Put a rectangle on the image
				
				Imgproc.circle(mat, new Point(640/2, 480/2), 20, new Scalar(color[0], color[1], color[2]), -1);
				

				// Give the output stream a new image to display
				//Core.transpose(mat, mat);
				//Core.flip(mat, mat, 1);
				
				
				outputStream.putFrame(mat);
			}
		});
		
		
		visionThread.setDaemon(true);
		visionThread.start();
		
		
		
	}
	
	

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	
	
	@Override
	public void autonomousInit() {
		for (Command command : commands) {
			System.out.println(command.command + ", " + command.value);
			/*if(autoShoot){
			leftShooter.set(-power);
			rightShooter.set(power);
			ballCollector.set(-1);
		}else{
			leftShooter.set(0);
			rightShooter.set(0);
			ballCollector.set(0);
		}*/
			if(command.command == "drive"){
				
				
				double t= System.currentTimeMillis();
				double end = t + ( ( command.value * 1000 ) / (metersPerSecond));
				while(System.currentTimeMillis() < end) {
					System.out.println("driving forward");
					
					
					robot.arcadeDrive(-.7,.15);
				  try {
					  Thread.sleep( 50 );
				  } catch (InterruptedException e) {
					e.printStackTrace();
				  }
				}
				
				robot.arcadeDrive(-.5, .25);
				
			}
			
			if(command.command == "stop"){
				double t= System.currentTimeMillis();
				double end = t+ ( command.value );
				while(System.currentTimeMillis() < end) {
					/*if(autoShoot){
					leftShooter.set(-power);
					rightShooter.set(power);
					ballCollector.set(-1);
				}else{
					leftShooter.set(0);
					rightShooter.set(0);
					ballCollector.set(0);
				}*/
					robot.arcadeDrive(0,0);
					try {
						Thread.sleep( 50 );
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
			
			if(command.command == "rotate"){
				double startAngle = gyro.getAngle();
				double end = startAngle + command.value;
				while(gyro.getAngle() < end - 10) {
					System.out.println(gyro.getAngle());
					/*if(autoShoot){
						leftShooter.set(-power);
						rightShooter.set(power);
						ballCollector.set(-1);
					}else{
						leftShooter.set(0);
						rightShooter.set(0);
						ballCollector.set(0);
					}*/
					robot.arcadeDrive(0,-.6);
				  try {
					  Thread.sleep( 50 );
				  } catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				  }
				}
			}
			
			if(command.command == "startShoot"){
				autoShoot = true;
			}
			
			if(command.command == "stopShoot"){
				autoShoot = false;
			}
		}
		
	}
        
		
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		System.out.println(gyro.getAngle());
		if(autoShoot){
			leftShooter.set(-power);
			rightShooter.set(power);
			ballCollector.set(-1);
		}else{
			leftShooter.set(0);
			rightShooter.set(0);
			ballCollector.set(0);
		}
	}
	
	
	public double[] rgb2hsv (double r,double g,double b) {
		 double computedH = 0;
		 double computedS = 0;
		 double computedV = 0;
		 
		
		 r=r/255; g=g/255; b=b/255;
		 double minRGB = Math.min(r,Math.min(g,b));
		 double maxRGB = Math.max(r,Math.max(g,b));

		 // Black-gray-white
		 if (minRGB==maxRGB) {
		  computedV = minRGB;
		  return new double[]{0,0,computedV};
		 }

		 // Colors other than black-gray-white:
		 double d = (r==minRGB) ? g-b : ((b==minRGB) ? r-g : b-r);
		 double h = (r==minRGB) ? 3 : ((b==minRGB) ? 1 : 5);
		 computedH = 60*(h - d/(maxRGB - minRGB));
		 computedS = (maxRGB - minRGB)/maxRGB;
		 computedV = maxRGB;
		 return new double[] {computedH,computedS,computedV};
		}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		timer.reset();
		timer.start();
		teleop = true;
		
	}
	
	@Override
	public void disabledPeriodic(){
		teleop = false;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		//robot.arcadeDrive(.7,.25); //driveForward
		//robot.arcadeDrive(0,1); // rotate
		int forward = -1;
		if(stick.getRawButton(11)){
			forward = 1;
		}else{
			forward = -1;
		}
		if(stick.getRawButton(1)){ 
			robot.arcadeDrive(-stick.getY() * .7 * forward,-stick.getX() * sensitivity);

		}else{
			robot.arcadeDrive(-stick.getY()  * forward, -stick.getX() * .7);
		}
		
		//robot.arcadeDrive(stick.getY(),stick.getX());
		
		
		leftShoot = stick.getRawButton(5);
		rightShoot = stick.getRawButton(6);
		

		
		retractRope = stick.getRawButton(12);
		catchRope = stick.getRawButton(10);
		

		System.out.println(stick.getAxis(AxisType.kThrottle));
		
		if(retractRope){
			leftClimber.set(-1);
			rightClimber.set(-1);
			
		}else{
			leftClimber.set(Math.abs((stick.getAxis(AxisType.kThrottle) - 1)) / 2);
			rightClimber.set(Math.abs((stick.getAxis(AxisType.kThrottle) - 1)) / 2);
			
		}
		
			
		/*if(climbRope){
			//TODO
			leftClimber.set(1);
			rightClimber.set(1);
		}else if(catchRope){
			leftClimber.set(0.5);
			rightClimber.set(0.5);
		}else{
			leftClimber.set(0);
			rightClimber.set(0);
		}*/
		
		
		if(collectBalls){
			ballCollector.set(-1);
		}else{
			ballCollector.set(0);
		}
		
		if(ballPusherForward){
			ballPusher.set(1);
		}else if(ballPusherBackward){
			ballPusher.set(-1);
		}else{
			ballPusher.set(0);
		}
		
		
		
		if(leftShoot || allShoot){ //if shoot is true
			leftShooter.set(-power); //activate motors
		}else{ //if shoot is false
			leftShooter.set(0); //deactivate motors
		}
		
		if(rightShoot || allShoot){ //if shoot is true
			rightShooter.set(power);
		}else{ //if shoot is false
			rightShooter.set(0);
		}
	}
	
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	
}
