# Data Processor

This package calculates parameters for Sensorfusion of an motion estimation with specific data.

`loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/big_loop_park_sunny_maxW.json");
loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/eight_park_spatial_memory.json");`

## Usage

Load Positional Data to the stack with: 
`loadDataToStack(path::String);`.
This Data is used to calculate the parameters. To check the amount of data to train with: 
`getLength()`.
Initialize training with: `train()`.

Tip: Use `;` to avoid prints to console when using in REPL.