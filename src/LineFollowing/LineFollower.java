package LineFollowing;

import java.util.Iterator;
import java.util.LinkedList;

import compatibility.DifferentialPilot;
import compatibility.LCD;
import lejos.geom.Point;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import ch.aplu.jgamegrid.Actor;
import ch.aplu.jgamegrid.Location;
import ch.aplu.robotsim.LegoRobot;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.UltrasonicSensor;


class Robot {
	public static boolean wait = false;
	
	//================== Variables ==================	
	static private LightSensor L_Sens_Left;
	static private LightSensor L_Sens_Right;
	
	static private UltrasonicSensor ultraSonic;
	
	static public LegoRobot robot;
  
	static DifferentialPilot pilot;
	//================= Constructor =================
	Robot( 
			LegoRobot r,
	  		LightSensor lS, LightSensor rS,
	  		UltrasonicSensor us, DifferentialPilot p) {
	  	
		robot = r;
		
		AStar.init();
		Map.init();
		Navigator.init();
		
		
	  	ultraSonic = us;
	  	
	  	L_Sens_Left = lS;
	  	L_Sens_Right = rS;
	  	
	  	pilot  = p;
	  	//pilot  = new DifferentialPilot(56, 177, Motor.B, Motor.C, false);  
	  	// parameters in inches;
	  	
	    // create each behavior
		Behavior junction = new Junction();
		Behavior move = new Move();
		
	    // define an array (vector) of existing behaviors, sorted by priority
		Behavior[] behaviors = { move, junction };
	
	    // add the behavior vector to a new arbitrator and start arbitration
		Arbitrator arbitrator = new Arbitrator(behaviors);
		
		LCD.drawString("Robot", 0, 0);
		//pilot.setTravelSpeed(100);  // cm per second
		//Button.waitForAnyPress();
		LCD.clear();
		
		drawSensor();
		
		forward();
		
		arbitrator.start();
  	}
	
	//==================== Robot Action ==================== 
	public static void stop() {
		/*==== Motor Implementation ====
		LeftMotor.stop();
		RightMotor.stop();
		 */
		
		//==== Pilot Implementation ====
		pilot.stop();
		
	}
	
	public static void rotate(int x) {
		//pilot.rotate(x);

		Actor r = robot.getRobot();
		r.turn(x);
		
		//ultraSonic.
	}
	
	public static void rotateLeft() {
		pilot.rotateLeft();
		
		
	}

	public static void rotateRight() {
		pilot.rotateRight();		
	}
	
	public static void forward() {
		/*==== Motor Implementation ====
		LeftMotor.forward();
		RightMotor.forward();
		*/
		
		pilot.forward();
	}
	
	public static void travel(int dist) {

		Actor r = robot.getRobot();
		
		
		switch(Map.facing) {
		
		case Values.NORTH:
			//r.setY(r.getY() - dist);

			break;
			
		case Values.SOUTH:
		
			//r.setY(r.getY() + dist);
			break;
		
		case Values.EAST:
		
			//r.setX(r.getX() + dist);
			break;
		
		case Values.WEST:
		
			//r.setX(r.getX() - dist);
			break;
		
		}
		r.move(dist);
	}
	
	public static int senseLeftLight() { 	
		return L_Sens_Left.getValue(); 
	}
	
	public static int senseRightLight() {
		return L_Sens_Right.getValue();
	}
	
	public static int senseSonar() { 
		//System.out.println(ultraSonic.getDistance());
		return ultraSonic.getDistance(); 
		
	} 

	public static void drawSonic() {
		int distance = ultraSonic.getDistance();
		
		LCD.clear();
		LCD.drawString("ultraSonic: ", 0, 0);
		LCD.drawInt(distance, 0, 1);
		LCD.clear();
		
	}
	
	
	/**
	 * Draws values of the left and right
	 * Sensors on the screen. 
	 */
	public static void drawSensor() {

		/*
		int left = L_Sens_Left.getValue();
		int right = L_Sens_Right.getValue();
		
		LCD.drawString("Left Sensor: ", 0, 1);
		LCD.drawInt(left, 0, 2);
	
		LCD.drawString("Right Sensor: ", 0, 3);
		LCD.drawInt(right, 0, 4);
		*/
	}
}





//================= Navigation Layer =================
/**
 * @author Teo
 * Class Map wills tore information about 
 * the robots environment. It will be updated
 * in real time as the robot explores the maze
 */
class Map {
	
	static int virtualWorld[][];
	
	static int robotX;
	static int robotY;
	
	static int worldX;
	static int worldY;
	
	static int targetX;
	static int targetY;
	
	static int facing;
	
	public static void init() {
		robotX = Values.initRobotX;
		robotY = Values.initRobotY;
				
		facing = Values.initFacing;
		
		worldX = Values.GridX * 2 - 1;
		worldY = Values.GridY * 2 - 1;
		
		targetX = worldX - 1;
		targetY = worldY - 1;
		
		virtualWorld = new int[worldX][worldY];		
		
		int i;
		int j;
		
		for(i= 0; i < worldX; i++ ) {
			for(j = 0; j < worldY; j++)  {
				virtualWorld[i][j] = Values.BLOCKED;				
			}
		}
		
		for(i = 0; i < worldX; i+=2 ) {
			for(j = 0; j < worldY; j++)  {
				virtualWorld[i][j] = Values.CLEAR;
			}
		}
			
		for(i = 0; i < worldX; i++ ) {
			for(j = 0; j < worldY; j+= 2)  {
				virtualWorld[i][j] = Values.CLEAR;
			}
		}
		
		printMap();
	}
	
	public static int lookAhead() {
		int aheadX = 0;
		int aheadY = 0;
		
		switch(facing) {
		case Values.NORTH:
			aheadY = robotY + 1;
			aheadX = robotX;
			break;
			
		case Values.SOUTH:
			aheadY = robotY - 1;
			aheadX = robotX;
			break;
		
		case Values.EAST:
			aheadY = robotY;
			aheadX = robotX + 1;
			break;
		
		case Values.WEST:
			aheadY = robotY;
			aheadX = robotX - 1;
			break;
		}
		if(aheadX >= worldX || aheadX < 0 ||
				aheadY >= worldY || aheadY < 0) {
			return Values.BLOCKED;
		} else {
			return virtualWorld[aheadX][aheadY];
		}
	}

	public static Point getNextNodeCoord() {
		Point p = new Point(0, 0);
		
		switch(facing) {
		case Values.NORTH:
			p.y = robotY + 2;
			p.x = robotX;
			break;
			
		case Values.SOUTH:
			p.y = robotY - 2;
			p.x = robotX;
			break;
		
		case Values.EAST:
			p.y = robotY;
			p.x = robotX + 2;
			break;
		
		case Values.WEST:
			p.y = robotY;
			p.x = robotX - 2;
			break;
		}		
		return p;
	}
	
	public static void printMap() {
		
		for(int i= worldX - 1; i >=0; i-- ) {
			for(int j= 0; j < worldY; j++ ) {
				if(j == robotX && i == robotY)  {
					System.out.print(" [R] ");
				} else if(j == targetX && i == targetY) {
					System.out.print(" [X] ");
				} else if(virtualWorld[j][i] == Values.CLEAR){
					System.out.print(" [ ] ");
				} else if(virtualWorld[j][i] == Values.BLOCKED) {
					System.out.print(" ... ");
				} else if(virtualWorld[j][i] == Values.VISITED) {
					System.out.print(" [#] ");
				}
			}
			
			System.out.println();
		}
		
		
		for(int i= 0; i < worldX; i++ ) {
			for(int j= 0; j < worldY; j++ ) {
				Node n = null;
				if(AStar.openNodes[i][j] != null) {
					System.out.print("Open - ");
					n = AStar.openNodes[i][j];
				} else if(AStar.closedNodes[i][j] != null) {
					System.out.print("Closed - ");
					n = AStar.closedNodes[i][j];
				} else {
					continue;
				}
				
				LinkedList<Integer> path = n.getPath();
				
				Iterator<Integer> it = path.iterator();
				
				System.out.print("X:" + i + " " + "Y:" + j + " | ");
				 while(it.hasNext()) {
			        switch(it.next()) {
			    	case Values.NORTH:
						System.out.print(" /\\ ");
						break;
						
					case Values.SOUTH:
						System.out.print(" \\/ ");
						break;
					
					case Values.EAST:
						System.out.print(" > ");
						break;
					
					case Values.WEST:
						System.out.print(" < ");
						break;
					
			        }
				 }
				 System.out.println(" | " + n.getCost());
				 
			}
		}
		System.out.println();
	}

	public static boolean atTarget() {
		if(robotX == targetX && robotY == targetY) {
			return true;
		}
		return false;
	}
	
	public static void updateMap(int x) {
		int aheadX = 0;
		int aheadY = 0;
		
		switch(facing) {
		case Values.NORTH:
			aheadY = robotY + 1;
			aheadX = robotX;
			break;
			
		case Values.SOUTH:
			aheadY = robotY - 1;
			aheadX = robotX;
			break;
		
		case Values.EAST:
			aheadY = robotY;
			aheadX = robotX + 1;
			break;
		
		case Values.WEST:
			aheadY = robotY;
			aheadX = robotX - 1;
			break;
		}
		if(aheadX >= worldX || aheadX < 0 ||
				aheadY >= worldY || aheadY < 0) {
			return;
		} else if(virtualWorld[aheadX][aheadY] != Values.VISITED){
			virtualWorld[aheadX][aheadY] = x;
		}
	}
	
	public static void updateRobotPosition() {
		
		virtualWorld[robotX][robotY] = Values.VISITED; 
		
		switch(facing) {
			case Values.NORTH:
				robotY++;
				virtualWorld[robotX][robotY] = Values.VISITED;
				robotY++;
				virtualWorld[robotX][robotY] = Values.VISITED;
				break;
				
			case Values.SOUTH:
				robotY--;
				virtualWorld[robotX][robotY] = Values.VISITED;
				robotY--;
				virtualWorld[robotX][robotY] = Values.VISITED;
				break;
			
			case Values.EAST:
				robotX++;
				virtualWorld[robotX][robotY] = Values.VISITED;
				robotX++;
				virtualWorld[robotX][robotY] = Values.VISITED;
				break;
			
			case Values.WEST:
				robotX--;
				virtualWorld[robotX][robotY] = Values.VISITED;
				robotX--;
				virtualWorld[robotX][robotY] = Values.VISITED;
				break;
		}
		
		
	}
}


class Node {
	
	LinkedList<Integer> path = new LinkedList<Integer>();
	
	int x;
	int y;

	private int cost;
	
	public int getFValue() {
		return ((Values.targetX - x) + (Values.targetY - y) + cost); 	
	
	}
	
	
	public void setCost(int v) 					{ cost = v; 	}
	public void setPath(LinkedList<Integer> p) 	{ path = p; 	}
	
	public int 					getCost() 		{ return cost; 	}
	public LinkedList<Integer> 	getPath() 		{ return path;	}
	
	public void updatePath(int w) {
		//path.add(w);
		path.addLast(w);
	}
	
	Node(int c) {
		cost = c;
	}
	
	Node(int c, LinkedList<Integer> p,int in_x, int in_y) {
		cost = c;
		path = p;
		
		x = in_x;
		y = in_y;
	}
	
}

class AStar {
	static Node[][] openNodes;
	static Node[][] closedNodes;	
	

	public static void init() {
		int i = Values.targetX * 2 + 1;
		int j = Values.targetY * 2 + 1;
		
		
		openNodes = new Node[i][j];
		closedNodes = new Node[i][j];
		
		for(int it = 0; it < i; it-- ) {
			for(int jt = 0; j < j; jt++ ) {
				openNodes[it][jt] = null;
				closedNodes[it][jt] = null;
			}
		
		}
		
		
		openNodes[0][0] = new Node(0);
	}
	
	public static int currentF() {
		return Navigator.NODE_DISTANCE + openNodes[Map.robotX][Map.robotY].getCost();
	}
	
	public static int predictedF(int x, int y) {
		return ((Values.targetX - x) + (Values.targetY - y)); 	
	
	}
	
	public static int calculateTotalCost(int x, int y) {
		return predictedF(x, y) + currentF();
	}
	
	public static void addOpenNode(int x, int y, int c, LinkedList<Integer> newL) {		
		openNodes[x][y] = new Node(c, newL, x , y);
	}
	
	public static void removeOpenNode(int x, int y) {
		openNodes[x][y] = null;
	}
	
	public static void addClosedNode(int x, int y) {
		closedNodes[x][y] = openNodes[x][y];
	}

	public static Node getOpenNode(int x, int y) {
		return openNodes[x][y];
	}

	public static Node getNextNode() {
		
		Node solution[] = new Node[100];
		int minF = 10000;
		
		int k = 0;
		for(int i = 0; i < Values.targetX * 2 + 1; i++) {
			for(int j = 0; j < Values.targetY * 2 + 1; j++) {
				if(openNodes[i][j] != null && openNodes[i][j].getFValue() == minF ){
					k++;
					solution[k] = openNodes[i][j];
					
				}
				
				if(openNodes[i][j] != null && openNodes[i][j].getFValue() < minF ){
					k = 0;
					minF = openNodes[i][j].getFValue();
					solution[k] = openNodes[i][j];
					
				}
				
			
			}
		}
		
		Node n = solution[0];
		double minDist = 100000;
		double dist = 0;
		
		for(int i = 0; i<=k; i++) {
			System.out.println("Potential Solution: " + solution[k].x + " " +solution[k].y);
			System.out.println("Robot: " + Map.robotX + " " +Map.robotY);
			
			int x = solution[k].x - Map.robotX;
			int y = solution[k].y - Map.robotY;
			
			dist = Math.sqrt( x * x + y * y );
			
			System.out.println("Dist: " + dist);
			
			if(dist < minDist) {
				minDist = dist;
				n = solution[k];
			}
		}
		
		
		System.out.println("Next node is X = " + n.x + ", Y = " + n.y );
		return n;
	}

}

class Navigator {

	public static final int NODE_DISTANCE = 2;
	
	static LinkedList<Integer> pathMemory = new LinkedList<Integer>();
	
	//================= Initialisation =================
	public static void init() {
		
		
	}
	
	
	//================= Work =================

	public static void advance() {
		//Robot.forward();
		Map.updateRobotPosition();
	}
	
	public static void robotFace(int f) {
		switch(Map.facing) {
		
		case Values.NORTH:
			helperFace(Values.NORTH,Values.SOUTH, Values.WEST, Values.EAST, f );
			break;
			
		case Values.SOUTH:
			helperFace(Values.SOUTH,Values.NORTH, Values.EAST, Values.WEST, f );
			break;
		
		case Values.EAST:
			helperFace(Values.EAST,Values.WEST, Values.NORTH, Values.SOUTH, f );
			break;
		
		case Values.WEST:
			helperFace(Values.WEST,Values.EAST, Values.SOUTH, Values.NORTH, f );
			break;
		
		}
	}
	
	public static void helperFace(final int ahead, final int back, final int left, final int right, int change) {
		if(change == back ) {
			Robot.rotate(-180);
			Map.facing = back;
		} else if (change == left) {
			Robot.rotate(-90);
			Map.facing = left;
		} else if(change == right) {
			Robot.rotate(90);
			Map.facing = right;
		}
	}
	
	public static void moveForward() {
		
		while(!(Robot.senseLeftLight() < Values.thLight && Robot.senseRightLight() < Values.thLight)) {
			if(			//Offcourse Right
				Robot.senseLeftLight() < Values.thLight) 
			{
				Robot.drawSensor();
				adjustRight();
			} else if(	//Offcourse Left
					Robot.senseRightLight() < Values.thLight) 
			{
				Robot.drawSensor();
				adjustLeft();
			}
			
		}		
	}


	public static void adjustRight()
	{
		Robot.stop();
		Robot.rotateRight();
		
		while(Robot.senseRightLight() < Values.thLight && !Move.stop);
		
		Robot.stop();
		Robot.forward();
	}
	
	public static void adjustLeft()
	{
		Robot.stop();
		Robot.rotateLeft();
		
		while(Robot.senseRightLight() < Values.thLight && !Move.stop);
		
		Robot.stop();
		Robot.forward();
	}
	

	public static void updateNeightbourNodes(int[] clearPaths) {
		//update their cost value
		for(int i = 0; i < clearPaths.length; i++) {
			Navigator.robotFace(clearPaths[i]);
			
			int x = Map.robotX;
			int y = Map.robotY;
			
			switch(clearPaths[i]) {
			case Values.NORTH:
				y += NODE_DISTANCE;
				break;
			case Values.SOUTH:
				y -= NODE_DISTANCE;
				break;
			case Values.EAST:
				x += NODE_DISTANCE;
				break;
			case Values.WEST:
				x -= NODE_DISTANCE;
				break;
			}
			Node n = AStar.getOpenNode(x, y);
			if(n == null) {
				LinkedList<Integer> L = AStar.openNodes[Map.robotX][Map.robotY].getPath();
				LinkedList<Integer> newL = (LinkedList<Integer>) L.clone();
				
				newL.add(clearPaths[i]);
				
				AStar.addOpenNode(x, y, AStar.currentF(), newL );
			} else {
				//int cost = AStar.calculateTotalCost(x, y);
				//if(n.getCost() > cost) {
					//n.setCost(cost);
					//n.setPath(Navigator.getOpenNode(Map.robotX, Map.robotY).getPath());
				//}
			}
		}
	}
	
	public static int[] scanNeightbours() 
	{
		int[] clearPaths = new int[4];
		int index = 0;
		
		
		for(int i = Values.EAST; i <= Values.SOUTH; i++) {
			Navigator.robotFace(i);
			
			if(Map.lookAhead() != Values.BLOCKED && Map.lookAhead() != Values.VISITED ) {
				
				if(Robot.senseSonar() < Values.thSonar && Robot.senseSonar() != -1) {
					Map.updateMap(Values.BLOCKED);
					
					System.out.println(Robot.senseSonar() + " Path Blocked");
				} else {
					Map.updateMap(Values.CLEAR);
					clearPaths[index++] = i;
					
					System.out.println(Robot.senseSonar() + " Path Clear");
				}
			}
			
			try{Thread.sleep(100);}catch(Exception e){}
		}
		return clearPaths;
	}

	public static void followPathMem() {

		System.out.println("Executing Memory Step");
		
		if(pathMemory.size() == 1) System.out.println("arrived at destination");
		
		Navigator.robotFace(pathMemory.pop());
		
	}
	


	public static void findPath(Node n) {

		pathMemory = new LinkedList<Integer>();
		
		int skip = 0;
		
		LinkedList<Integer> nodePath = n.getPath();
		Iterator<Integer> iN = nodePath.iterator();
		
		LinkedList<Integer> returnPath = AStar.closedNodes[Map.robotX][Map.robotY].getPath();
		Iterator<Integer> iB = returnPath.iterator();
		
		if(!(Map.robotX == 0 && Map.robotY == 0)) {
			
			while(iB.hasNext() && iN.hasNext()) {
				
				if(iB.next() == iN.next()) {
					skip++;
				} else {
					break;
				}
			}
			
			System.out.println("SKIP IS :::: " + skip);
			
			System.out.println("Memorising return path");
			iB = returnPath.descendingIterator();
			int count = 0;
			
			while(iB.hasNext() && returnPath.size() - skip > count) {
				count++;
				
			    int dir = (int) iB.next();
			     
			    switch(dir) {
			    case Values.NORTH:
			    	pathMemory.add(Values.SOUTH);
			    	break;
			    case Values.SOUTH:
			    	pathMemory.add(Values.NORTH);
			    	break;
			    case Values.EAST:
			    	pathMemory.add(Values.WEST);
			    	break;
			    case Values.WEST:
			    	pathMemory.add(Values.EAST);
			    	break;
			    default:
			    	 continue;
			    }        				
			}
		}
		
		System.out.println("Memorising path to node");	
		iN = nodePath.iterator();
		
		while(iN.hasNext()) {		
			if(skip == 0) {
				pathMemory.add((int) iN.next());
			} else {
				iN.next();
				skip--;
			}
		}
		System.out.println("Memorization complete");
	}
	
	public static boolean isMemEmpty() {
		return pathMemory.isEmpty();		
	}
}





//================= Action Layer =================
class Junction implements Behavior
{
	boolean stop = false;	
	
	public void action()			// what to do
	{	
		
		stop = false;
		
		if(Robot.wait) {
			return;
		}
		
		if(Map.atTarget() == true ) {
			System.out.println("At target");
			
			Robot.stop();
			//Robot.robot.getRobot().setX(Values.START_X);
			//Robot.robot.getRobot().setY(Values.START_Y);
			
			Navigator.pathMemory = (LinkedList<Integer>) AStar.openNodes[Map.robotX][Map.robotY].getPath().clone();
			
			Map.robotX = 0;
			Map.robotY = 0;
			
			Robot.wait = true;
			
			return;
		}
		
		System.out.println("==== Junction behaviour ====");

		Robot.stop();	
		//Robot.travel(9);
		Robot.pilot.travel(200);
		Robot.stop();	
		
		Map.printMap();
		Robot.drawSensor();
		
		
		
		//===== Important Stuff here =====
			
		if(Navigator.isMemEmpty()) {
			//scan neighbour nodes
			System.out.println("Scanning");
			int clearPaths[] = Navigator.scanNeightbours();
			System.out.println();
			
			//update their cost value
			System.out.println("Updating");
			Navigator.updateNeightbourNodes(clearPaths);
			System.out.println();
			
			//close current node
			AStar.addClosedNode(Map.robotX, Map.robotY);
			AStar.removeOpenNode(Map.robotX, Map.robotY);
			
			
			//getNodeWithSmallestCost
			System.out.println("Get next node");
			Node n = AStar.getNextNode();
			System.out.println();
		
			//memorize path
			System.out.println("Memorize path");
			Navigator.findPath(n);
			System.out.println();
			
		} 		
		Navigator.followPathMem();
		Navigator.advance();
		
		try{Thread.sleep(1000);}catch(Exception e){}
		
		System.out.println("==== Done ====");
		//================================
			
			
		LCD.clear();
	
		stop = true;
	}

	public void suppress()			// how to stop doing it
	{	
		Robot.stop();
		stop = true;
	}

	public boolean takeControl()	// when to start doing it
	{		
		return (Robot.senseLeftLight() < Values.thLight &&
				Robot.senseRightLight() < Values.thLight);
	}
}


class Move implements Behavior
{
	static boolean stop = false;
		
	public void action()			// what to do
	{	
		stop = false;
		Robot.wait = false;
		
		Robot.forward();
		
		System.out.println("===== Move Behaviour =====");
		Robot.drawSensor();
		
		Navigator.moveForward();
		System.out.println("===== Done =====");
		
		Robot.stop();
		
		//try{Thread.sleep(100);}catch(Exception e){}	
	
	}
	
	public void suppress()			// how to stop doing it
	{	
		Robot.stop();
		stop = true;
	}

	public boolean takeControl()	// when to start doing it
	{
		return (Robot.senseLeftLight() > Values.thLight &&
				Robot.senseRightLight() > Values.thLight);
	}	
}

