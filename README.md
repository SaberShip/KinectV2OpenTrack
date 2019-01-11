## Intro
This project is intended for those who wish to use an XBox One Kinect (Kinect v2) as a head-tracking source for games supported by the "opentrack" project.

The opentrack project is located here: <http://github.com/opentrack/opentrack>

## Requirements
You will need a few things
- Microsoft Visual Studio 2017 (To build the executable)
- Microsoft Kinect SDK V2: [here](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
- Microsoft Kinect V2 Sensor 
  - With the official PC USB adapter kit
  - Or with USB power mod (Google search "modify kinect for pc")

## Setup
Assuming you have opentrack installed and KinectV2OpenTrack built

- First open up opentrack and set the input to UDP with a port of 4242 and pitch +90.
- Next simply connect up the Kinect v2 sensor and run KinectV2OpenTrack.exe
- A console should appear and if all is working the Kinect should light up and tracking will begin.
- Select an output for your purposes in opentrack and start.


## Notes and Thanks
This was a quick one day project for me as I wanted some headtracking solution and had a modded Kinect V2 laying around. It works better than I thought it would even in low light! Personally I use it to play Elite Dangerous with and it should be good enough tracking for most cockpit based games.

It ended up being easier than I thought to get tracking up and running with open track.

- So big thanks to the opentrack team as the UDP interfacing was straight forward and their program is helpful and easy to use. 
- Also Thanks to Microsof for the SDK and examples that made this code easy to make.