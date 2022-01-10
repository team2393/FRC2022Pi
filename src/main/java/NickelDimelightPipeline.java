/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.cscore.CvSource;
import org.opencv.core.Mat;

/** Pipeline for a Nickel-and-Dimelight 
 *  
 *  Detects features via HSV and shape thresholding
 */
public class NickelDimelightPipeline extends ColorInfoPipeline
{
    NickelDimelightPipeline(final CvSource output, final int width, final int height)
    {
        super(output, width, height);
    }
    
    @Override
    public void process(final Mat frame)
    {
        prepare(frame);

        // TODO Filter on HSV threshold

        // TODO Filter on contour's area

        // TODO Filter on contour's fullness

        // TODO Filter on contour's aspect ratio

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
