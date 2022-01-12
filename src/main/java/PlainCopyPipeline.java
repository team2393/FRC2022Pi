/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/** Pipeline that simply copies each frame to the output video stream
 * 
 *  Well, and it adds a counter so we can see how many frames
 *  get processed.
 *  .. and provides a helper for adding info text to the frame.
 */
public class PlainCopyPipeline implements VisionPipeline
{
    /** Counter for calls to `process()` */
    protected AtomicInteger calls = new AtomicInteger();

    protected final CvSource output;
    protected final int width, height;

    /** Colors for drawing overlay */
    protected final Scalar overlay_bgr = new Scalar(200.0, 100.0, 255.0), contrast_bgr = new Scalar(0, 0, 0);

    PlainCopyPipeline(final CvSource output, final int width, final int height)
    {
        this.output = output;
        this.width = width;
        this.height = height;
    }

    /** Show info at bottom of image.
     *  @param frame Where to show the info
     *  @param info Info text to show
     */
    protected void showInfo(final Mat frame, final String info)
    {
        // Add 'info' text to bottom of frame.
        // We don't know what's in the image.
        // Is it bright? -> Should paint text in dark color.
        // Is it dark?   -> Should paint text in bright color
        // --> Paint it twice, overlay-on-black,
        //     to show up no matter what's in the image
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
    }

    @Override
    public void process(final Mat frame)
    {
        // String.format() uses a 'printf' format.
        // Originally introduced by the C 'printf',
        // it's now available in pretty much every programming language.
        // Quirky, but universal.
        //
        // '%' starts a placeholder for values that follow.
        // '%3d' prints a decimal (int, long) using 3 characters: "  0", " 12", "123"
        // '%03d' prints a decimal (int, long) using 3 characters and filling with 0: "000", "012", "123"
        // '%5.2f' prints a floating point number (double), using 5 characters, and 2 numbers after the '.': " 3.14", " 0.00"
        // '%s' prints a string
        final String info = String.format("Call # %03d",
                                          calls.incrementAndGet());
        showInfo(frame, info);

        // Publish 'output'
        output.putFrame(frame);
      }
}
