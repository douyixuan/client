## mono_check_simulator.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__check__simulatorc.png" 
width="400"  />
</p>

### Description 
Check if the simulator is open and running. If it is not running then it will stop running the client.

### Inputs
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Vehicle.exe error (String):** Output from the command line if the simulator is not running.
- **Connected (Boolean) :** True if the simulator is running.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
