package org.usfirst.frc.team2984.robot.util;

import static org.opencv.core.Core.inRange;

import java.io.File;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;


public class Main extends Thread {

	private static final double NORMAL_WIDTH = 1.65;
	
	private Rect left;
	private Rect right;
	private Mat output;
	private Mat processingMat;
	private Mat tmp;
    private Scalar minc;
    private Scalar maxc;
	
	private volatile boolean shouldProcess;
	private volatile boolean hasTrack;
	private volatile double offsetAngle;
	private volatile double robotAngle;
	private volatile double distance;
	
	public Main(){
		minc = new Scalar(58, 144, 133);
		maxc = new Scalar(133, 255, 255);
		this.shouldProcess = true;
		this.hasTrack = false;
		this.offsetAngle = -1;
		this.robotAngle = -1;
		this.distance = -1;
		this.output = new Mat();
		this.tmp = new Mat();
		this.processingMat = new Mat();
	}
	
	/**
	 * Starts the camera capture, sets resolution and exposure, and starts processing the video.
	 */
	@Override
	public void run() {
		
	}
	
	public static void main(String[] args){
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		File folder = new File("/home/max/Desktop/pics/blender");
		Main main = new Main();
		File[] files = folder.listFiles();
		Arrays.sort(files, new Comparator<File>(){

			@Override
			public int compare(File f0, File f1) {
				return f0.getName().replace("Left", "").replace("Right", "").compareTo(f1.getName().replace("Left", "").replace("Right", ""));
			}
			
		});
		NumberFormat format = NumberFormat.getInstance();
		format.setMinimumFractionDigits(2);
		format.setMaximumFractionDigits(2);
		NumberFormat percent = NumberFormat.getPercentInstance();
		boolean now = false;
		double prev = 0;
		for(int i = 0; i<files.length; i++){
			File f = files[i];
			if(f.getName().contains("Binary") || f.getName().contains("Blurred")){
				continue;
			}
			Mat m = Imgcodecs.imread(f.getAbsolutePath(), Imgcodecs.CV_LOAD_IMAGE_COLOR);
			Mat resizeimage = new Mat();
//			Size sz = new Size(320,240);
//			Imgproc.resize( m, resizeimage, sz );
			main.process(m);
			if(main.hasTrack){
//				System.out.println("For angle of " + i + " calculated was: " + format.format(main.getAngle()*180/Math.PI) + " " + percent.format((Math.abs(main.getAngle()*180/Math.PI + i))/i) + " error");
			}
			if(now){
				System.out.println("Diff" + (main.offsetAngle + prev)*90D/Math.PI);
				now = false;
			} else {
				now = true;
				prev = main.offsetAngle;
			}
//			System.out.println("Name: " + f.getName() + " Distance: " + (main.hasTrack ? main.distance : -1) +
//					" Angle: " + (main.hasTrack ? main.offsetAngle*180/Math.PI : -1) + " Robot Angle: " + (main.hasTrack ? main.robotAngle : -1));			

		}
	}
	
	/**
	 * Finds the rectangles and then calculates each of the measurements and updates them into the local variables.
	 * @param source The image to process
	 */
	public void process(Mat source){
		Rect[] rects = this.findRects(source);
		if(rects != null){
			if(rects[0].x < rects[1].x){
				this.left = rects[0];
				this.right = rects[1];
			} else {
				this.left = rects[1];
				this.right = rects[0];
			}
			double averageHight = this.left.height + this.right.height;
			averageHight /= 2.0;
			double distance = distance(averageHight);
			double angle = angle(Math.abs(this.right.x + this.right.width/2D - this.left.x -this.left.width/2D), averageHight);
			angle *= (this.left.height > this.right.height) ? -1 : 1;
			double robotAngle = robotAngle((this.right.x + this.left.x + this.right.width/2D + this.left.width/2D)/2.0);
			hasTrack = true;
			this.distance = distance;
			this.offsetAngle = angle;
			this.robotAngle = robotAngle;

		} else {
			hasTrack = false;
		}
	}
	
	private double robotAngle(double center){
		return center*2/320-1;
	}
	
	/**
	 * Finds the distance from the peg, Right not it uses the wrong algorithm, it should just be a linear regression.
	 * @param hight the height of the rectangles
	 * @return the distance from he camera to the rectangles.
	 */
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
	
	/**
	 * Finds the rectangles in the given picture.
	 * It first filters out all other colors by searching for a color range, creating a binary image.
	 * Then it blurs the image. Then if runs a contour finder on the image. Then it makes sure that there are only two reults.
	 * If there are more or less it will return null.
	 * @param source The image to look in
	 * @return null (if not fount) or the two found rectangles
	 */
	private Rect[] findRects(Mat source){
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        inRange(output, minc, maxc, output);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.blur(output, processingMat, new Size(3, 3));
        Imgproc.findContours(processingMat, contours, tmp, Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
        int i = 0;
        while(i < contours.size()){
        	if(Imgproc.contourArea(contours.get(i)) < 40){
        		contours.remove(i);
        		continue;
        	}
        	i++;
        }
        if(contours.size() == 2){
        	Rect rectA = Imgproc.boundingRect(contours.get(0));
            Rect rectB = Imgproc.boundingRect(contours.get(1));
            return new Rect[]{rectA, rectB};
        }
    	System.out.print("NOT TWO BUT " + contours.size() + "-");
        return null;
	}
	
	/**
	 * Sets whether or not the tracker should be tracking, if not it sleeps for 10ms and then checks if it should check again.
	 * @param tracking whether or not to track
	 */
	public synchronized void setTracking(boolean tracking){
		this.shouldProcess = tracking;
	}
	
	/**
	 * Gets the angle offset from the peg's view, 0 is dead center, + is clockwise
	 * @return the angle [-90, 90]
	 */
	public synchronized double getAngle(){
		return this.offsetAngle;
	}

	/**
	 * Returns the distance in inches from the peg
	 * @return the distance [0, infinity]
	 */
	public synchronized double getDistance(){
		return this.distance;
	}
	
	public synchronized double robotAngle(){
		return this.robotAngle;
	}
	
	/**
	 * Returns whether or not the tracker has a track.
	 * @return whether or not the tracker has a track.
	 */
	public synchronized boolean hasTrack(){
		return this.hasTrack && this.shouldProcess;
	}
	
}
