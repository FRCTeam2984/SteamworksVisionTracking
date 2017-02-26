package org.usfirst.frc.team2984.robot.util;

import static org.opencv.core.Core.inRange;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class DualCameraTracker {

	//CONSTANTS TO BE FACTORED OUT
	private static final double ACCURATE_ANGLE_THESHOLD = 15;
	
	private double cameraWidth;
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	
	public DualCameraTracker(double cameraWidth){
		this.cameraWidth = cameraWidth;
	}
	
	public DoubleVisionTarget process(Mat leftImage, Mat rightImage){
		this.binarizeImage(leftImage);
		this.binarizeImage(rightImage);
		VisionTarget leftTarget = this.findTarget(leftImage);
		VisionTarget rightTarget = this.findTarget(rightImage);
		if(leftTarget != null && rightTarget != null){
			System.out.println("Left: " + leftTarget + " Right: " + rightTarget);
		}
		return null;
	}
	
	private void binarizeImage(Mat m){
		Mat colorConvert = new Mat();
        Imgproc.cvtColor(m, colorConvert, Imgproc.COLOR_BGR2HSV);
		List<Mat> hsv = new ArrayList<Mat>();   //destination array
		Core.split(colorConvert,hsv);
		Mat v = hsv.get(2);
		Scalar low = new Scalar(170);
		Scalar high = new Scalar(255);
        inRange(v, low, high, m);
	}
	
	private VisionTarget findTarget(Mat m){
		boolean findContoursExternalOnly = false;
		findContours(m, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 60.0;
		double filterContoursMinPerimeter = 0;
		double filterContoursMinWidth = 3.0;
		double filterContoursMaxWidth = 1000;
		double filterContoursMinHeight = 10.0;
		double filterContoursMaxHeight = 1000;
		double[] filterContoursSolidity = {0.0, 100};
		double filterContoursMaxVertices = 10.0;
		double filterContoursMinVertices = 4.0;
		double filterContoursMinRatio = 0.1;
		double filterContoursMaxRatio = 0.5;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
		if(filterContoursOutput.size() == 2){
        	Rect rectA = Imgproc.boundingRect(filterContoursOutput.get(0));
            Rect rectB = Imgproc.boundingRect(filterContoursOutput.get(1));
			Moments muA = Imgproc.moments(filterContoursOutput.get(0), false);
			Moments muB = Imgproc.moments(filterContoursOutput.get(1), false);

	        return new VisionTarget(rectA, rectB, muA, muB);
		}
		return null;
	}
	
		
	private void findContours(Mat input, boolean externalOnly,
			List<MatOfPoint> contours) {
			Mat hierarchy = new Mat();
			contours.clear();
			int mode;
			if (externalOnly) {
				mode = Imgproc.RETR_EXTERNAL;
			}
			else {
				mode = Imgproc.RETR_LIST;
			}
			int method = Imgproc.CHAIN_APPROX_SIMPLE;
			Imgproc.findContours(input, contours, hierarchy, mode, method);
	}
	
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
			double minPerimeter, double minWidth, double maxWidth, double minHeight, double
			maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
			minRatio, double maxRatio, List<MatOfPoint> output) {
			final MatOfInt hull = new MatOfInt();
			output.clear();
			//operation
			for (int i = 0; i < inputContours.size(); i++) {
				final MatOfPoint contour = inputContours.get(i);
				final Rect bb = Imgproc.boundingRect(contour);
				if (bb.width < minWidth || bb.width > maxWidth) continue;
				if (bb.height < minHeight || bb.height > maxHeight) continue;
				final double area = Imgproc.contourArea(contour);
				if (area < minArea) continue;
				if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
				Imgproc.convexHull(contour, hull);
				MatOfPoint mopHull = new MatOfPoint();
				mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
				for (int j = 0; j < hull.size().height; j++) {
					int index = (int)hull.get(j, 0)[0];
					double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
					mopHull.put(j, 0, point);
				}
				final double solid = 100 * area / Imgproc.contourArea(mopHull);
				if (solid < solidity[0] || solid > solidity[1]) continue;
				if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
				final double ratio = bb.width / (double)bb.height;
				if (ratio < minRatio || ratio > maxRatio) continue;
				output.add(contour);
			}
		}
}
