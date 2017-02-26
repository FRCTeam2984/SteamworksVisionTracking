package org.usfirst.frc.team2984.robot.util;

import java.text.NumberFormat;

import org.opencv.core.Rect;
import org.opencv.imgproc.Moments;

public class VisionTarget {

	private static final double NORMAL_WIDTH = 1.65;
	
	private double angle;
	private double distance;
	private double robotAngle;
	
	public VisionTarget(Rect rectA, Rect rectB, Moments muA, Moments muB){
		double xA = (muA.get_m10() / muA.get_m00());
		double yA = (muA.get_m01() / muA.get_m00());
		double xB = (muB.get_m10() / muB.get_m00());
        double yB = (muB.get_m01() / muB.get_m00());
        double avgHeight = rectA.height + rectB.height;
        avgHeight /= 2;
        double distApart = Math.abs(xA - xB);
        System.out.println(distApart);
        this.distance = this.distance(avgHeight);
        this.angle = angle(distApart, avgHeight);
        this.robotAngle = this.robotAngle((xA + xB)/2D);
	}
	
	private double distance(double hight){
		//TODO Get Real Values
		double verticalAngle = hight/240*(35/180D*Math.PI);
		return 5/Math.tan(verticalAngle);
	}
	
	/**
	 * finds the angle based on the distance from the peg and the height of the rectangles.
	 * The height is used to find the distance that should be between the two rectangles.
	 * This height is then used along with the distance between the rectangles to find the angle offset.
	 * This is done with a triangle where one side is the distance between the two rectangles and the other is the appropriate width.
	 * @param dist The distance between the two rectangles in pixels
	 * @param hight The height of the rectangles in pixels
	 * @return the angle offset of the peg [0, 90]
	 */
	private double angle(double dist, double hight){
		double widthNomialNormalized = NORMAL_WIDTH*hight;
//		System.out.print("Dist: " + dist + "height" + hight);
//		System.out.print("Ratio" + hight + " " + dist + " " + widthNomialNormalized);
		double angle = Math.acos(Math.min(dist / widthNomialNormalized, 1));
		return angle;
	}
	
	private double robotAngle(double center){
		return center*2/320-1;
	}
	
	public double getAngle() {
		return angle;
	}

	public double getDistance() {
		return distance;
	}

	public double getRobotAngle() {
		return robotAngle;
	}
	
	public String toString(){
		NumberFormat format = NumberFormat.getInstance();
		format.setMaximumFractionDigits(2);
		format.setMinimumFractionDigits(2);
		return "{ Distance: " + format.format(this.distance) + ", RobotAngle: " + format.format(this.robotAngle) + ", Angle: " + format.format(this.angle) + "}";
	}
}
