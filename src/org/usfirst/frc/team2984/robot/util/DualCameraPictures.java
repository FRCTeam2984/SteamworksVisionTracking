package org.usfirst.frc.team2984.robot.util;

import java.io.File;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.Comparator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public class DualCameraPictures {

	public static void main(String[] args){
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );

		DualCameraTracker tracker = new DualCameraTracker(10D);
		File folder = new File("/home/max/Desktop/pics/blender");
		File[] files = folder.listFiles();
		Arrays.sort(files, new Comparator<File>(){

			@Override
			public int compare(File f0, File f1) {
				return f0.getName().replace("Left", "").replace("Right", "").compareTo(f1.getName().replace("Left", "").replace("Right", ""));
			}
			
		});
		boolean left = false;
		boolean print = false;
		Mat previous = null;
		for(int i = 0; i<files.length; i++){
			File f = files[i];
			if(f.getName().contains("Binary") || f.getName().contains("Blurred")){
				continue;
			}
			Mat m = Imgcodecs.imread(f.getAbsolutePath(), Imgcodecs.CV_LOAD_IMAGE_COLOR);
			if(!print){
				left = f.getName().contains("Left");
				previous = m;
				print = true;
			} else {
				if(left){
					tracker.process(previous, m);
				} else {
					tracker.process(m, previous);
				}
				print = false;
			}
		}
	}
	
}
