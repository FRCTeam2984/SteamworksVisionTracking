package org.usfirst.frc.team2984.robot.util;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;

import javax.swing.JPanel;

import org.opencv.core.Mat;


public class Frame extends JPanel{

    BufferedImage image;


    @Override
    public void paint(Graphics g) {
    	if(image != null)
        g.drawImage(image, 0, 0, this);
    }

    public Frame() {
    }

    public Frame(BufferedImage img) {
    }   
 
    public void setImage(Mat image){
    	this.image = MatToBufferedImage(image);
    	this.repaint();
    }
    
    public BufferedImage MatToBufferedImage(Mat frame) {
        //Mat() to BufferedImage
        int type = 0;
        if (frame.channels() == 1) {
            type = BufferedImage.TYPE_BYTE_GRAY;
        } else if (frame.channels() == 3) {
            type = BufferedImage.TYPE_3BYTE_BGR;
        }
        BufferedImage image = new BufferedImage(frame.width(), frame.height(), type);
        WritableRaster raster = image.getRaster();
        DataBufferByte dataBuffer = (DataBufferByte) raster.getDataBuffer();
        byte[] data = dataBuffer.getData();
        frame.get(0, 0, data);

        return image;
    }

}