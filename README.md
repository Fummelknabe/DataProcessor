# Data Processor

This package calculates parameters for Sensorfusion of an motion estimation with specific data.

Tip: If methods are called in REPL use ";" to avoid prints.

`
]activate .
instantiate
using DataProcessor
loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/big_loop_park_sunny_maxW.json");
loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/eight_park_spatial_memory.json");
getLength()
train()
`

## Results

PredictionParameters(false, true, 12.300134f0, true, 0.09999818f0, true, 0.175f0, 0.030000007f0, 0.7600027f0, -1.4901161f-9, 2.9802323f-9, 0.2f0, 3.099999f0, 0.0f0, 0.33333334f0)