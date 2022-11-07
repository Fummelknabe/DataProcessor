# This file implements functions that are used to handle data.

using JSON

# The first value of the magnetometer
firstMagValue = -1

"""
Converts a dictionary to PositionalData datatype.

# Arguments
- `dict::Dict`: The dict to convert
- `rotateCameraCoords::Bool`: Should the camera coordinates be rotated to match IMU data
# Returns 
- `PositionalData`: All the positional data combined in one datatype
"""
function convertDictToPosData(dict::Dict, rotateCameraCoords::Bool)
    posData = PositionalData()
        
    posData.steerAngle = dict["steerAngle"] - 120
    posData.sensorAngle = dict["sensorAngle"]
    posData.maxSpeed = dict["maxSpeed"]
    posData.sensorSpeed = dict["sensorSpeed"]
    posData.imuMag = dict["imuMag"]
    camPos = dict["cameraPos"]
    camPos = [camPos[1], -camPos[3], camPos[2], camPos[4]]
    if rotateCameraCoords 
        firstMagValue == -1 && global firstMagValue = posData.imuMag
        camPos = transformCameraCoords(Float32.(camPos), convertMagToCompass(firstMagValue)) 
    end
    posData.cameraPos = camPos
    posData.cameraOri = dict["cameraOri"]
    posData.imuGyro = deg2rad.(dict["imuGyro"])
    posData.imuAcc = dict["imuAcc"]    
    posData.deltaTime = dict["deltaTime"]
    posData.cameraConfidence = dict["cameraConfidence"] ./ 100
    posData.command = dict["command"]

    return posData
end

"""
Loads positional data from a json file. The file is selected manually through a dialog in the GUI.

# Optional Arguments
- `rotateCameraCoords::Bool`: Should the camera coordinates be rotated to match IMU data
- `flipCameraCoords::Bool`: Additional rotation of the camera data by 180 degrees
- `loadGPSData::Bool`: Should gps data be loaded, otherwise its `[0, 0]`
- `deleteData::Bool`: Deletes part of positional data that contain no information
# Returns 
- `StructArray{PositionalData}`: An array holding the positional data
"""
function loadFromJSon(rotateCameraCoords::Bool, path::String)
    posData = StructArray(PositionalData[]);
    try
        posDataDicts = JSON.parsefile(path, dicttype=Dict, inttype=Int64);

        for dict in posDataDicts        
            push!(posData, convertDictToPosData(dict, rotateCameraCoords));
        end
    catch e
        if e isa SystemError
            @warn "The given file does not exist."
        else
            println(e)
        end
    end

    return posData
end

"""
This funciton loads the true positional value from a .json file.

# Argument 
- `path::String`: The path to the .json file.
# Returns 
- `Matrix{Float32}`: Matrix containing the position information in its coloums. 
"""
function loadFromJSon(path::String)
    posData = Matrix{Float32}(undef, 3, 0)
    try
        posDataDicts = JSON.parsefile(path, dicttype=Dict, inttype=Int64);

        for dict in posDataDicts        
            posData = hcat(posData, [dict["x"], dict["y"], dict["z"]]);
        end
    catch e
        if e isa SystemError
            @warn "The given file does not exist."
        else
            println(e)
        end
    end

    return posData
end

"""
This function loads parameters used for estimation from json file. 

# Arguments 
- `filePath::String`: Path to .json file containing parameter information. 

# Returns
- `PredictionSettings`: The settings described in the json file
"""
function loadParamsFromJSon(filePath::String)
    params = PredictionParameters()
    paramsDict = JSON.parsefile(filePath, dicttype=Dict, inttype=Int64)

    params.exponentCC = paramsDict["exponentCC"]
    params.speedExponentCC = paramsDict["speedExponentCC"]
    params.kalmanFilterCamera = paramsDict["kalmanFilterCamera"]
    params.kalmanFilterGyro = paramsDict["kalmanFilterGyro"]
    params.UKF = paramsDict["UKF"]
    params.measurementNoiseC = paramsDict["measurementNoiseC"]
    params.measurementNoiseG = paramsDict["measurementNoiseG"]
    params.measurementNoiseS = paramsDict["measurementNoiseS"]
    params.processNoiseC = paramsDict["processNoiseC"]
    params.processNoiseG = paramsDict["processNoiseG"]
    params.processNoiseS = paramsDict["processNoiseS"]
    params.odoGyroFactor = paramsDict["odoGyroFactor"]
    params.odoMagFactor = paramsDict["odoMagFactor"]
    params.odoSteerFactor = paramsDict["odoSteerFactor"]
    params.steerAngleFactor = paramsDict["steerAngleFactor"]
    params.speedUseSinCC = paramsDict["speedUseSinCC"]
    params.useSinCC = paramsDict["useSinCC"]
    params.σ_forSpeedKernel = paramsDict["σ_forSpeedKernel"]
    params.ΨₒmagInfluence = paramsDict["ΨₒmagInfluence"]
    params.κ = paramsDict["κ"]
    params.α = paramsDict["α"]

    return params
end

"""
This function saves parameters from `PredictionParameters` to a .json file.

# Argument 
- `params::PredictionParameters`: The parameters to save as a file. 
# Optional Arguments 
- `fileName::Union{String, Nothing}`: The file name to save the parameters to. If `nothing` "pred_params.json" is used.
"""
function saveParamsJSon(params::PredictionParameters; fileName::Union{String, Nothing}=nothing)
    name = isnothing(fileName) ? "pred_params" : fileName
    open("params/$(name).json", "w") do ioStream
        JSON.print(ioStream, params, 4)
    end
end
