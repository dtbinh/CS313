package LineFollowing;

import java.awt.Color;
import java.awt.Point;

import compatibility.DifferentialPilot;
import ch.aplu.jgamegrid.GGMouse;
import ch.aplu.jgamegrid.GGMouseListener;
import ch.aplu.jgamegrid.GameGrid;
import ch.aplu.jgamegrid.Location;
import ch.aplu.jgamegrid.Location.CompassDirection;
import ch.aplu.robotsim.Gear;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.Motor;
import ch.aplu.robotsim.MotorPort;
import ch.aplu.robotsim.RobotContext;
import ch.aplu.robotsim.SensorPort;
import ch.aplu.robotsim.TouchSensor;
import ch.aplu.robotsim.UltrasonicSensor;
import ch.aplu.util.Monitor;

//================= Simulation Layer =================
public class Sim {
	LegoRobot robot;
	
	static LightSensor L_Sens_Left;
	static LightSensor L_Sens_Right;
	
	static TouchSensor ts1;
	static TouchSensor ts2;
  
	static UltrasonicSensor us;
	
	static DifferentialPilot pilot;
  
	private static int xOld, yOld;
  
	// ================= Environment =================
	static
	{
		
		//RobotContext.useObstacle("sprites/hazelnut.gif", 200, 375);
		//RobotContext.setLocation(10, 10);
		//RobotContext.useObstacle(RobotContext.channel);
		
		RobotContext.showNavigationBar();
		  
		
	

		RobotContext.setStartDirection(0);
		RobotContext.setStartPosition(Values.START_X, Values.START_Y);
		
		RobotContext.useBackground("sprites/grid2.gif");
		
		
		
		  Point[] nut_mesh =
			    {
			      new Point(10, 10), new Point(-10, 10),
			      new Point(-10, -10), new Point(10, -10)
			    };
			   
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 200, 350);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 300, 350);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 400, 350);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 150, 290);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 150, 190);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 250, 190);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 250, 90);
		  
		  
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 300, 250);
		  
		  RobotContext.useTarget("sprites/hazelnut.gif", nut_mesh, 400, 150);
	
		
	}
	
	@SuppressWarnings("unused")
	private static void _init(final GameGrid gg)
	{
	    gg.setTitle("Drag with left mouse button to draw the track");
	    gg.addMouseListener(
	      new GGMouseListener()
	      {
	        public boolean mouseEvent(GGMouse mouse)
	        {
	          Location loc =
	            gg.toLocationInGrid(mouse.getX(), mouse.getY());
	          switch (mouse.getEvent())
	          {
	            case GGMouse.lPress:
	              xOld = loc.x;
	              yOld = loc.y;
	              break;
	            case GGMouse.lDrag:
	              gg.getBg().drawLine(xOld, yOld, loc.x, loc.y);
	              xOld = loc.x;
	              yOld = loc.y;
	              break;
	            case GGMouse.lRelease:
	              Monitor.wakeUp();  // Start simulation
	              break;
	          }
	          return true;
	        }
	      }, GGMouse.lPress | GGMouse.lDrag | GGMouse.lRelease);
	    gg.getBg().setPaintColor(Color.black);
	    gg.getBg().setLineWidth(4);
	  }
	
	// ================= Setup =================
	public Sim() throws Exception
	{
		LegoRobot r = Setup();
		new Robot(r, L_Sens_Left, L_Sens_Right, us, pilot);
	
	}
  
	public LegoRobot Setup() {
		robot = new LegoRobot();
		
		
		Motor RightMotor = new Motor(MotorPort.A);
		Motor LeftMotor = new Motor(MotorPort.B);
		    
		L_Sens_Left = new LightSensor(SensorPort.S1);
		L_Sens_Right = new LightSensor(SensorPort.S2);
		
		us = new UltrasonicSensor(SensorPort.S3);
		
	//	us.setBeamAreaColor(Color.green);
//		us.setProximityCircleColor(Color.lightGray);
		
		Gear gear = new Gear();
		pilot = new DifferentialPilot(gear);
		
	    robot.addPart(L_Sens_Left);
	    robot.addPart(L_Sens_Right);
	    
	    robot.addPart(us);
	    
	    //robot.addPart(RightMotor);
	    //robot.addPart(LeftMotor);
	    
		us.setDirection(CompassDirection.WEST);
		
	    
	    robot.addPart(gear); 
	    
	    return robot;
	}
}