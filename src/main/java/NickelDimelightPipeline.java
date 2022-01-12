/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
    protected final Mat filt = new Mat();

    /** Detected contours */
    private final List<MatOfPoint> contours = new ArrayList<>();

    /** Temporary data for contour filter */
    protected final Mat tmp = new Mat();

    NickelDimelightPipeline(final CvSource output, final int width, final int height)
    {
        super(output, width, height);

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
    }
    
    @Override
    public void process(final Mat frame)
    {
        prepare(frame);

        // Filter on Hue, Saturation and value
        hsv_min.val[0] = SmartDashboard.getNumber("HueMin", hsv_min.val[0]);
        hsv_max.val[0] = SmartDashboard.getNumber("HueMax", hsv_max.val[0]);
        hsv_min.val[1] = SmartDashboard.getNumber("SatMin", hsv_min.val[1]);
        hsv_max.val[1] = SmartDashboard.getNumber("SatMax", hsv_max.val[1]);
        hsv_min.val[2] = SmartDashboard.getNumber("ValMin", hsv_min.val[2]);
        hsv_max.val[2] = SmartDashboard.getNumber("ValMax", hsv_max.val[2]);
        Core.inRange(hsv, hsv_min, hsv_max, filt);

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
            // TODO Send info? udp.send(direction, distance);

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
            // udp.send(0, 0);
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
