/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/** Pipeline that provides info about color in center of image */
public class ColorInfoPipeline extends PlainCopyPipeline
{
    // Intermediate images used for pre-processing
    private final Mat norm = new Mat(),
                      blur = new Mat();
    
    /** HSV version of current frame */
    protected final Mat hsv = new Mat();

    /** RGB at center of image */
    protected int center_b = 0, center_g = 0, center_r = 0;

    /** HSV at center of image */
    protected int center_h = 0, center_s = 0, center_v = 0;

    ColorInfoPipeline(final CvSource output, final int width, final int height)
    {
        super(output, width, height);
    }

    /** Pre-process the frame
     * 
     *  Resize, normalixe, blur, convert to HSV
     */
    public void prepare(final Mat frame)
    {
        calls.incrementAndGet();

        // In principle, should be possible to re-use Mat()s:
        // 1) Resize original frame to smaller tmp1
        // 2) Normalize tmp1 into tmp2
        // 3) Convert RGB from tmp2 into HLS tmp1
        // .. but that resulted in strange values for HLS,
        // like H > 180.
        // So re-using Mats across process calls,
        // but within one process call always using it for the
        // same purpose.

        // Scale colors to use full 0..255 range in case image was dark
        Core.normalize(frame, norm, 0.0, 255.0, Core.NORM_MINMAX);

        // When moving the camera, or turning auto-focus off and de-focusing,
        // we would detect the target, but when standing still and in perfect focus,
        // we missed it?!
        // --> Blurring the image helps detect the target!
        Imgproc.blur(norm, blur, new Size(8, 8));

        // Convert to HSV
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);

        // Probe BGR and HSV at center of image,
        center_b = center_g = center_r = 0;
        center_h = center_s = center_v = 0;

        // Average over 9 pixels at center x, y +-1
        final byte[] probe = new byte[3];
        int avg = 0;
        for (int x=-1; x<=1; ++x)
            for (int y=-1; y<=1; ++y)
            {
                norm.get(height/2 + x, width/2 + y, probe);
                center_b += Byte.toUnsignedInt(probe[0]);
                center_g += Byte.toUnsignedInt(probe[1]);
                center_r += Byte.toUnsignedInt(probe[2]);
                hsv.get(height/2 + x, width/2 + y, probe);
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

        SmartDashboard.putNumber("Center B", center_b);
        SmartDashboard.putNumber("Center G", center_g);
        SmartDashboard.putNumber("Center R", center_r);
        SmartDashboard.putNumber("Center H", center_h);
        SmartDashboard.putNumber("Center S", center_s);
        SmartDashboard.putNumber("Center V", center_v);

        // Show rect in center of image where pixel info is probed
        Imgproc.rectangle(frame,
                          new Point(width/2 - 2, height/2 - 2),
                          new Point(width/2 + 2, height/2 + 2),
                          overlay_bgr);
    }

    @Override
    public void process(final Mat frame)
    {
        prepare(frame);

        final String info = String.format("# %3d RGB %3d %3d %3d HSV %3d %3d %3d",
                                          calls.get(),
                                          center_r,
                                          center_g,
                                          center_b,
                                          center_h,
                                          center_s,
                                          center_v);
        showInfo(frame, info);

        // Publish 'output'
        // Typically show 'frame', i.e., original image with overlay.
        // But could show 'blur' or any other intermediate image while debugging.
        //output.putFrame(blur);
        output.putFrame(frame);
      }
}
