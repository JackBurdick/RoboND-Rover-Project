[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
# Search and Sample Return Project
![alt text][image_0] 

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html).  

## The Simulator
Download the simulator build appropriate for your operating system.  Here are the links for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).  

You can test out the simulator by;
- opening it
- choosing "Training Mode".  
- Use the mouse or keyboard to navigate around the environment

## Dependencies
- Python 3
- Jupyter Notebooks
- Environment (`./environment.yml`)
    - Create the included environment with;
    - `conda env create -f environment.yml`
        - *Note that this has only been tested in Windows*
        - more information can be found [here](https://github.com/ryan-keenan/RoboND-Python-Starterkit): 

## Recording Data
Test data:
    - `./test_dataset`.  
        - a csv file with the output data for steering, throttle position etc. and the pathnames to the images recorded in each run.
    - `./calibration_images`
        - a few images to do some initial calibration steps with  

To record data on your own;
  1. Create a new folder to store the image data in.
  2. Launch the simulator and choose "Training Mode" then hit "r".  
  3. Navigate to the directory you want to store data in, select it
  4. Drive around collecting data.  
  5. Hit "r" again to stop data collection.

## Data Analysis
`Rover_Project_Test_Notebook.ipynb` includes functions for performing the various steps of this project.

The last two cells in the notebook are for running the analysis on a folder of test images to create a map of the simulator environment and write the output to a video.  These cells should run as-is and save a video called `test_mapping.mp4` to the `output` folder.

## Navigating Autonomously
- `drive_rover.py`
    - will navigate the environment in autonomous mode.  
    - This script calls functions from within;
        - `perception.py`
            - `perception_step()`: processing steps and updates the rover map.
        - `decision.py`.
            - `decision_step()`: includes an example of a conditional statement you could use to navigate autonomously based on the rover's state and the results of the `perception_step()` analysis.
To Use;
> `drive_rover.py` should work as is if you have all the required Python packages installed. Call it at the command line like this: `python drive_rover.py`. Then launch the simluator and choose "Autonomous Mode".
 

**Note: running the simulator with different resolution and graphics quality may produce different results!**


