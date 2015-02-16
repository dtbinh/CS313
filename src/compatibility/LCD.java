package compatibility;

public class LCD 
{
	public static void drawString(String s, int x, int y) {
		System.out.println(s);
	}
	public static void clear() {
		System.out.println();
		System.out.println();
	}	
	public static void drawInt(int v, int x, int y) {
		System.out.println(v);
	}
}