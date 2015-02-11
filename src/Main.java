

import LineFollowing.Sim;




public class Main
{
	public static void main(String[] args)
	{
		
		//DifferentialPilot pilot = new DifferentialPilot(56, 177, Motor.B, Motor.C, false);  // parameters in inches
		
		try {
			new Sim();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}





