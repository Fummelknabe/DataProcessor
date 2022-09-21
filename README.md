# Data Processor

This package calculates parameters for Sensorfusion of an motion estimation with specific data.

`loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/big_loop_park_sunny_maxW.json", 1);
loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/eight_park_spatial_memory.json", 2);
loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/drone_shot_park_7_9_2.json", 2);`

## Usage

Load Positional Data to the stack with: 
`loadDataToStack(path::String);`.
This Data is used to calculate the parameters. To check the amount of data to train with: 
`getLength()`.
Initialize training with: `train()`.
If you want to save the resulting parameters to JSON then: `train(saveAsFile=true)`.

Tip: Use `;` to avoid prints to console when using in REPL.