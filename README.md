FRC2022 Software for Pi
=======================

Prepare Pi
----------

.. with WPILib image as per
https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/index.html

Network Config
--------------

The rPi should be accessible as http://wpilibpi.local or `ssh pi@wpilibpi.local` (password `raspberry`)
with an IP address assigned by the robot radio's DHCP, but for reliability we
set a fixed address of 10.23.93.36 via the network tab on http://wpilibpi.local

Build
-----

While you can open the FRC2022Pi folder in VS Code for editing,
there is no 'build' or 'deploy' button.
You need to open a `cmd.exe` window,
`cd \Users\YOUR_NAME\git\FRC2022Pi` and then invoke `make.bat`,
which basically does this:
1) Setup: `\Users\Public\wpilib\2022\frccode\frcvars2022.bat`
2) Cleanup: `gradlew.bat clean`
3) Finally build: `gradlew.bat build`

When running gradle the first time, PC needs internet connection.
Later re-builds can be done while on the robot wifi,
sand just `gradlew.bat build` tends to suffice.

Deploy
------

On robot wifi, open the rPi web dashboard via http://wpilibpi.local or http://10.23.93.36

1) Make the rPi writable by selecting the "Writable" tab
2) In the rPi web dashboard Application tab, select the "Uploaded Java jar"
   option for Application
3) Click "Browse..." and select the "FRC2022Pi-all.jar" file in
   your desktop project directory in the build/libs subdirectory
4) Click Save

The application will be automatically started.  Console output can be seen by
enabling console output in the Vision Status tab.

View
----

 * Original camera image with controls: http://wpilibpi.local:1181 or http://10.23.93.36:1181/
 * Processed image: http://10.23.93.36:1182/
 * Just the processed stream: http://10.23.93.36:1182/stream.mjpg

Monitor
-------

If all fails, need to connect HDMI monitor and keyboad to Pi and watch the boot up messages, which include the IP address.
Can then log in via pi/raspberry.

For basic monitoring, use the rPi web dashboard:

 * "Vision Status"
 * Enable "Console Output"

