/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/** Pipeline that provides info about color in center of image */
public class ColorInfoPipeline implements VisionPipeline
{
    // Scaling factor for reduced size of processed image
    public static final int scale = 2;

    private final CvSource output;
    private final int width, height, proc_width, proc_height;

    // Intermediate images used for processing
    private final Mat small = new Mat(),
                      norm = new Mat(),
                      blur = new Mat(),
                      hsv = new Mat();

    // Counter for calls to `process()`
    public AtomicInteger calls = new AtomicInteger();

    // Colors for drawing overlay
    private final Scalar overlay_bgr = new Scalar(200.0, 100.0, 255.0), contrast_bgr = new Scalar(0, 0, 0);

    ColorInfoPipeline(final CvSource output, final int width, final int height)
    {
        this.output = output;
        this.width = width;
        this.height = height;
        proc_width = width / scale;
        proc_height = height / scale;
    }

  @Override
  public void process(final Mat frame)
  {
    // In principle, should be possible to re-use Mat()s:
    // 1) Resize original frame to smaller tmp1
    // 2) Normalize tmp1 into tmp2
    // 3) Convert RGB from tmp2 into HLS tmp1
    // .. but that resulted in strange values for HLS,
    // like H > 180.
    // So re-using Mats across process calls,
    // but within one process call always using it for the
    // same purpose.

    // Resize to use less CPU & memory to process 
    Imgproc.resize(frame, small, new Size(proc_width, proc_height));
    
    // Scale colors to use full 0..255 range in case image was dark
    Core.normalize(small, norm, 0.0, 255.0, Core.NORM_MINMAX);

    // When moving the camera, or turning auto-focus off and de-focusing,
    // we would detect the target, but when standing still and in perfect focus,
    // we missed it?!
    // --> Blurring the image helps detect the target!
    Imgproc.blur(norm, blur, new Size(8, 8));

    // Convert to HSV
    Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);

    // Probe BGR and HSV at center of image,
    int center_b = 0, center_g = 0, center_r = 0;
    int center_h = 0, center_s = 0, center_v = 0;
    // Average over 9 pixels at center x, y +-1
    final byte[] probe = new byte[3];
    int avg = 0;
    for (int x=-1; x<=1; ++x)
        for (int y=-1; y<=1; ++y)
        {
            norm.get(proc_height/2 + x, proc_width/2 + y, probe);
            center_b += Byte.toUnsignedInt(probe[0]);
            center_g += Byte.toUnsignedInt(probe[1]);
            center_r += Byte.toUnsignedInt(probe[2]);
            hsv.get(proc_height/2 + x, proc_width/2 + y, probe);
            center_h += Byte.toUnsignedInt(probe[0]);
            center_s += Byte.toUnsignedInt(probe[1]);
            center_v += Byte.toUnsignedInt(probe[2]);
            ++avg;
        }
    center_b /= avg;
    center_g /= avg;
    center_r /= avg;
    center_h /= avg;
    center_s /= avg;
    center_v /= avg;
    
    // Show rect in center of image where pixel info is probed
    Imgproc.rectangle(frame,
                      new Point(width/2 - 2, height/2 - 2),
                      new Point(width/2 + 2, height/2 + 2),
                      overlay_bgr);
    
    // Show info at bottom of image.
    // Paint it twice, overlay-on-black, for better contrast
    SmartDashboard.putNumber("Center B", center_b);
    SmartDashboard.putNumber("Center G", center_g);
    SmartDashboard.putNumber("Center R", center_r);
    SmartDashboard.putNumber("Center H", center_h);
    SmartDashboard.putNumber("Center S", center_s);
    SmartDashboard.putNumber("Center V", center_v);
    final String info = String.format("# %3d BGR %3d %3d %3d HSV %3d %3d %3d",
                                      calls.incrementAndGet(),
                                      center_b,
                                      center_g,
                                      center_r,
                                      center_h,
                                      center_s,
                                      center_v);
    Imgproc.putText(frame,
                    info,
                    new Point(1, height-16),
                    Core.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    contrast_bgr,
                    1);
    Imgproc.putText(frame,
                    info,
                    new Point(2, height-15),
                    Core.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    overlay_bgr,
                    1);

    // Publish 'output'
    // Typically show 'frame', i.e., original image with overlay.
    // But could show 'blur' or any other intermediate image while debugging.
    //output.putFrame(blur);
    output.putFrame(frame);
  }
}
