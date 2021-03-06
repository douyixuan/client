# LabVIEW Client
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/main.jpg" 
width="400"  />
</p>


### Setup
1. Download LabVIEW 2018 (32 bit) from this [website](http://www.ni.com/download/labview-development-system-2018/7406/en/).
2. Download VeloView from this [website](https://www.paraview.org/download/).
3. Download the monoDrive Simulator from [here](https://www.monodrive.io/download).

### Description 

This example demonstrates how to use LabVIEW as a client for the monoDrive Simulator.

#### Instructions:

1. Make sure you download the monoDrive Simulator from https://www.monodrive.io/download
2. Run the **VehicleAI.exe**
3. Make sure you the simulator is running before you run the **monoDrive Simulator Interface Example.vi**.
4. Run the VI with the default values (Close_loop mode).
5. Click on the GUI tab to look at the camera images.
6. On the GUI tab use the steering control and the forward control to move the car (on close_loop mode).
7. On the Sensors tab look at the data from other sensors.

#### To Run in Replay mode:

1. Change the Trajectory Config path with a replay configuration. i.e. LeftTurnCrossWalk.json 
2. Select replay in the Running mode control.
3. Hit Run.
4. Click on the GUI tab to look at the camera images.

#### To Run in Replay_step mode:
1. Change the Trajectory Config path with a replay configuration. i.e. LeftTurnCrossWalk.json
2. Select replay_step in the Running mode control.
3. Hit Run.
4. Click on the GUI tab to look at the camera images.

For technical support contact us at <b>support@monodrive.io</b>
