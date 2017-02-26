package org.usfirst.frc.team2984.robot.util;

import static org.opencv.core.Core.inRange;

import java.awt.GridLayout;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class DualCameraLive extends Thread {

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
	
	public DualCameraLive(int hl){
		minc = new Scalar(0, 0, hl);
		maxc = new Scalar(180, 255, 255);
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
		VideoCapture rightCamera = new VideoCapture(1);
		rightCamera.set(Videoio.CV_CAP_PROP_FRAME_WIDTH, 320);
		rightCamera.set(Videoio.CV_CAP_PROP_FRAME_HEIGHT, 240);
		rightCamera.set(Videoio.CV_CAP_PROP_XI_EXPOSURE, 4);

		VideoCapture leftCamera = new VideoCapture(2);
		leftCamera.set(Videoio.CV_CAP_PROP_FRAME_WIDTH, 320);
		leftCamera.set(Videoio.CV_CAP_PROP_FRAME_HEIGHT, 240);
		leftCamera.set(Videoio.CV_CAP_PROP_XI_EXPOSURE, 4);

		Mat rightFrame = new Mat();
		rightCamera.read(rightFrame);
		Mat leftFrame = new Mat();
		leftCamera.read(leftFrame);
		DualCameraLive right = new DualCameraLive(200);
		DualCameraLive left = new DualCameraLive(120);

		NumberFormat format = NumberFormat.getInstance();
		format.setMinimumFractionDigits(2);
		format.setMaximumFractionDigits(2);
		GridLayout experimentLayout = new GridLayout(0,2);
		JFrame frame0 = new JFrame();
		frame0.setLayout(experimentLayout);
		Frame rightPanel = new Frame();
		Frame leftPanel = new Frame();
        frame0.add(rightPanel );
        frame0.add(leftPanel );
        frame0.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame0.setTitle("Blob");
        frame0.setSize(1280, 510);
        frame0.setLocation(100, 100);
        frame0.setVisible(true);
		if(!rightCamera.isOpened()){
            System.out.println("Error");
        }
        else {                  
            while(true){        

                if (rightCamera.read(rightFrame) && leftCamera.read(leftFrame)){
                	right.process(rightFrame);
                	left.process(leftFrame);
        			rightPanel.setImage(rightFrame);
        			leftPanel.setImage(leftFrame);
        			System.out.println(right.hasTrack + " " + left.hasTrack);
        			if(right.hasTrack && left.hasTrack){
        				System.out.println("Delta " + format.format(right.robotAngle + left.robotAngle) + " " + format.format(right.robotAngle) + " " + format.format(left.robotAngle));
        			}
        			System.out.println();
                }
            }   
        }
	}
	
	private static void processBlobs(Mat frame, int hl, NumberFormat format, Frame panel){
		Mat colorConvert = new Mat();
        Imgproc.cvtColor(frame, colorConvert, Imgproc.COLOR_BGR2HSV);
		List<Mat> hsv = new ArrayList<Mat>();   //destination array
		Core.split(colorConvert,hsv);
		Mat h = hsv.get(0);
		Mat s = hsv.get(1);
		Mat v = hsv.get(2);
		Scalar low = new Scalar(hl);
		Scalar high = new Scalar(255);
        inRange(h, low, high, h);
        inRange(s, low, high, s);
        inRange(v, low, high, v);

		double findBlobsMinArea = 500.0;
		double[] findBlobsCircularity = {0.1, 1};
		boolean findBlobsDarkBlobs = false;
		MatOfKeyPoint output = new MatOfKeyPoint();
		findBlobs(v, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, output);
		KeyPoint[] blobs = output.toArray();
		Features2d.drawKeypoints(v, output, frame);
		panel.setImage(frame);
		for(KeyPoint p : blobs){
			System.out.print(" BLOB: " + format.format(p.pt.x) + " " + format.format(p.pt.y));
		}
	}
	
	private static void findBlobs(Mat input, double minArea, double[] circularity,
			Boolean darkBlobs, MatOfKeyPoint blobList) {
			FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
			try {
				File tempFile = File.createTempFile("config", ".xml");

				StringBuilder config = new StringBuilder();

				config.append("<?xml version=\"1.0\"?>\n");
				config.append("<opencv_storage>\n");
				config.append("<thresholdStep>10.</thresholdStep>\n");
				config.append("<minThreshold>50.</minThreshold>\n");
				config.append("<maxThreshold>220.</maxThreshold>\n");
				config.append("<minRepeatability>2</minRepeatability>\n");
				config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
				config.append("<filterByColor>1</filterByColor>\n");
				config.append("<blobColor>");
				config.append((darkBlobs ? 0 : 255));
				config.append("</blobColor>\n");
				config.append("<filterByArea>1</filterByArea>\n");
				config.append("<minArea>");
				config.append(minArea);
				config.append("</minArea>\n");
				config.append("<maxArea>");
				config.append(33930);
				config.append("</maxArea>\n");
				config.append("<filterByCircularity>0</filterByCircularity>\n");
				config.append("<minCircularity>");
				config.append(circularity[0]);
				config.append("</minCircularity>\n");
				config.append("<maxCircularity>");
				config.append(circularity[1]);
				config.append("</maxCircularity>\n");
				config.append("<filterByInertia>1</filterByInertia>\n");
				config.append("<minInertiaRatio>");
				config.append(circularity[0]);
				config.append("</minInertiaRatio>\n");
				config.append("<maxInertiaRatio>");
				config.append(circularity[1]);
				config.append("</maxInertiaRatio>\n");
				config.append("<filterByConvexity>0</filterByConvexity>\n");
				config.append("</opencv_storage>\n");
				FileWriter writer;
				writer = new FileWriter(tempFile, false);
				writer.write(config.toString());
				writer.close();
				blobDet.read(tempFile.getPath());
			} catch (IOException e) {
				e.printStackTrace();
			}

			blobDet.detect(input, blobList);
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
			double angle = angle(Math.abs(this.right.x - this.left.x), averageHight);
			angle *= (this.left.height > this.right.height) ? -1 : 1;
			double robotAngle = robotAngle((this.right.x + this.left.x)/2.0);
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
//		System.out.print("Ratio" + dist/widthNomialNormalized + " " + dist + " " + widthNomialNormalized);
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
