/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.camera.UDPServer;
import frc.robot.camera.VisionData;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

/** Pipeline for a Nickel-and-Dimelight 
 *  
 *  Detects features via HSV and shape thresholding
 * 
 *  See https://en.wikipedia.org/wiki/HSL_and_HSV
 */
public class NickelDimelightPipeline extends ColorInfoPipeline
{
    /** Hue (0-180), Luminance (0-255), Saturation (0-255) filter */
    private final Scalar hsv_min = new Scalar( 75-20,  30.0,  50.0),
                         hsv_max = new Scalar( 75+20, 255.0, 255.0);

    /** HSV - filtered version of current frame */
    protected final Mat filt1 = new Mat(), filt2 = new Mat(), filt = new Mat();

    /** Detected contours */
    private final List<MatOfPoint> contours = new ArrayList<>();

    /** Temporary data for contour filter */
    protected final Mat tmp = new Mat();

    /** Vision data that we send via UDP */
    private final VisionData vision_data = new VisionData();

    /** UDP server used to send vision data */
    private final UDPServer udp_server;

    NickelDimelightPipeline(final CvSource output, final int width, final int height) throws Exception
    {
        super(output, width, height);

        udp_server = new UDPServer();

        // Put initial values on dashboard
        SmartDashboard.setDefaultNumber("HueMin", hsv_min.val[0]);
        SmartDashboard.setDefaultNumber("HueMax", hsv_max.val[0]);
        SmartDashboard.setDefaultNumber("SatMin", hsv_min.val[1]);
        SmartDashboard.setDefaultNumber("SatMax", hsv_max.val[1]);
        SmartDashboard.setDefaultNumber("ValMin", hsv_min.val[2]);
        SmartDashboard.setDefaultNumber("ValMax", hsv_max.val[2]);
        SmartDashboard.setDefaultNumber("AreaMin", 0.0);
        SmartDashboard.setDefaultNumber("AreaMax", width * height);
        SmartDashboard.setDefaultNumber("AspectMin", 0.0);
        SmartDashboard.setDefaultNumber("AspectMax", 20);
        SmartDashboard.setDefaultNumber("FullnessMin", 0.0);
        SmartDashboard.setDefaultNumber("FullnessMax", 100.0);
        SmartDashboard.setDefaultNumber("CircularityMin", 0.0);

        SmartDashboard.setDefaultBoolean("SetHSV", false);
    }
    
    @Override
    public void process(final Mat frame)
    {
        prepare(frame);

        // Get snapshot of HSV in center?
        if (SmartDashboard.getBoolean("SetHSV", false))
        {
            // Reset button
            SmartDashboard.putBoolean("SetHSV", false);

            SmartDashboard.putNumber("HueMin", (center_h-10 + 180.0) % 180.0);
            SmartDashboard.putNumber("HueMax", (center_h+10) % 180.0);
            SmartDashboard.putNumber("SatMin", Math.max(0, center_s-10));
            SmartDashboard.putNumber("SatMax", Math.min(center_s+10, 255));
            SmartDashboard.putNumber("ValMin", Math.max(0, center_v-10));
            SmartDashboard.putNumber("ValMax", Math.min(center_s+10, 255));
        }


        // Filter on Hue, Saturation and value
        hsv_min.val[0] = SmartDashboard.getNumber("HueMin", hsv_min.val[0]);
        hsv_max.val[0] = SmartDashboard.getNumber("HueMax", hsv_max.val[0]);
        hsv_min.val[1] = SmartDashboard.getNumber("SatMin", hsv_min.val[1]);
        hsv_max.val[1] = SmartDashboard.getNumber("SatMax", hsv_max.val[1]);
        hsv_min.val[2] = SmartDashboard.getNumber("ValMin", hsv_min.val[2]);
        hsv_max.val[2] = SmartDashboard.getNumber("ValMax", hsv_max.val[2]);

        // The OpenCV hue range is
        // red    green     blue     red
        // 0.......70........120.....180
        //
        // If we want 'green', we can use a hue min..max of say 60..80.
        // But if we want 'red', we really need to check both 0..10 and 170..180.
        //
        // Support that by allowing min=170, max=10,
        // detecting the wraparound at 180 if min > max.
        if (hsv_min.val[0] <= hsv_max.val[0])
        {
            // If we want a red range of 0..10, that's fine.
            // So is a red range of 170..180.
            // Or 10..170 to get all colors except red.
            Core.inRange(hsv, hsv_min, hsv_max, filt);
        }
        else
        {
            // But if we want a range of say 170..10, 'wrapping around' the 180 degree point,
            // we need to check this in two steps
            double hue_min = hsv_min.val[0];
            double hue_max = hsv_max.val[0];

            // Check 0 .. 10
            hsv_min.val[0] = 0.0;
            hsv_max.val[0] = hue_max;
            Core.inRange(hsv, hsv_min, hsv_max, filt1);

            // Check 170..180
            hsv_min.val[0] = hue_min;
            hsv_max.val[0] = 180.0;
            Core.inRange(hsv, hsv_min, hsv_max, filt2);

            // Add the result of 0..10 and 170..180
            Core.add(filt1, filt2, filt);

            // Restore limits "170..10"
            hsv_min.val[0] = hue_min;
            hsv_max.val[0] = hue_max;
        }

        // Find contours
        contours.clear();
        Imgproc.findContours(filt, contours, tmp, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Get largest contour
        int largest_contour_index = -1;
        double largest_area = SmartDashboard.getNumber("AreaMin", 0.0);
        final double area_max = SmartDashboard.getNumber("AreaMax", width * height);
        final double aspect_min = SmartDashboard.getNumber("AspectMin", 0.0);
        final double aspect_max = SmartDashboard.getNumber("AspectMax", 20);
        final double fullness_min = SmartDashboard.getNumber("FullnessMin", 0.0);
        final double fullness_max = SmartDashboard.getNumber("FullnessMax", 100.0);
        final double circularity_min = SmartDashboard.getNumber("CircularityMin", 0.0);

        for (int i=0; i<contours.size(); ++i)
        {
            final MatOfPoint contour = contours.get(i);

            // Filter on contour's area
            final double area = Imgproc.contourArea(contour);
            if (area < largest_area  ||  area > area_max)
                continue;
                        
            // Filter on aspect ratio 0 (tall) .. 1 (square) .. 20 (wide)
            final Rect bounds = Imgproc.boundingRect(contour);
            final double aspect = (double)bounds.width / bounds.height;
            if (aspect < aspect_min  ||  aspect > aspect_max)
                continue;
                
            // Filter on fullness (percent): 0% (hollow) .. 100% (solid, full)
            final double fullness = 100.0 * area / (bounds.width * bounds.height);
            if (fullness < fullness_min  ||  fullness > fullness_max)
                continue;

            // Perimeter
            final MatOfPoint2f contour2f = new MatOfPoint2f();
            contour.convertTo(contour2f, CvType.CV_32F);
            final double perimeter = Imgproc.arcLength(contour2f, true);
                            
            // Circularity = 4*Math.PI*area / perimeter^2
            // Circle:
            //      4*pi*(pi*r*r)/(2*pi*r)^2 =
            //      4*pi*pi*r*r/(4*pi*pi*r*r) = 1
            //
            // Square:
            //      4*pi*d*d/(4*d)^2 =
            //      4*pi*d*d/(16*d*d) = pi/4 = 0.78
            final double circularity = 4*Math.PI*area / (perimeter*perimeter);
            SmartDashboard.putNumber("Circularity", circularity);
            if (circularity < circularity_min)
                continue;

            // Imgproc.drawContours(frame, contours, i, overlay_bgr);
            // Imgproc.rectangle(frame, bounds.tl(), bounds.br(), overlay_bgr);

            // Passed all tests: This is so far the largest area that we like
            largest_area = area;
            largest_contour_index = i;
        }

        // Found anything?
        if (largest_contour_index >= 0)
        {
            // Show largest contour
            Imgproc.drawContours(frame, contours, largest_contour_index, overlay_bgr);

            // Arrow from mid-bottom of image to center of blob
            final MatOfPoint largest_contour = contours.get(largest_contour_index);
            final Rect bounds = Imgproc.boundingRect(largest_contour);
            final int horiz_pos = bounds.x + bounds.width/2;
            final int vert_pos  = bounds.y + bounds.height/2;
            Imgproc.arrowedLine(frame,
                                new Point(width/2, height-1),
                                new Point(horiz_pos, vert_pos),
                                overlay_bgr);

            // Publish direction to detected blob in pixels from center
            // 0 - In center or not found, i.e. no reason to move
            // positive 1 .. width/2: Blob is to the right of center
            // negative -1 .. -width/2: .. left of center
            final int direction = horiz_pos - width/2;
            // Publish distance to detected blob in pixels from center
            // 0 - In center or not found, i.e. no reason to move
            // positive 1 .. height/2: Blob is ahead of center
            // negative -1 .. -height/2: .. below center
            final int distance = height/2 - vert_pos;
            SmartDashboard.putNumber("Direction", direction);
            SmartDashboard.putNumber("Distance", distance);

            // Send info ASAP via UDP
            vision_data.direction = direction;
            vision_data.distance = distance;
            udp_server.send(vision_data);

            SmartDashboard.putNumber("Area", largest_area);    

            final double fullness = 100.0 * largest_area / (bounds.width * bounds.height);
            SmartDashboard.putNumber("Fullness", fullness);    

            final double aspect = (double)bounds.width / bounds.height;
            SmartDashboard.putNumber("Aspect", aspect);    
        }
        else
        {
            SmartDashboard.putNumber("Direction", 0);
            SmartDashboard.putNumber("Distance", 0);
            SmartDashboard.putNumber("Area", 0);    
            SmartDashboard.putNumber("Fullness", -1);    
            SmartDashboard.putNumber("Aspect", -1);   

            // Send info ASAP via UDP
            vision_data.clear();
            udp_server.send(vision_data);
        }

        final String info = String.format("# %3d HSV %3d %3d %3d",
                                         calls.get(),
                                         center_h,
                                         center_s,
                                         center_v);
        showInfo(frame, info);

        // Publish 'output'
        // Typically show 'frame', i.e., original image with overlay.
        // But could show other intermediate image while debugging.
        output.putFrame(frame);
    }
}
