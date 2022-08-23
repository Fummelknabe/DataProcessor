using JSON

export PositionalData
"""
Struct to store the data acquired by the AT-RP
"""
mutable struct PositionalData
    steerAngle::Integer
    sensorAngle::Integer
    maxSpeed::Float32
    sensorSpeed::Float32
    cameraPos::Vector{Float32}
    cameraOri::Vector{Float32}    
    imuGyro::Vector{Float32}
    imuAcc::Vector{Float32}
    imuMag::Vector{Float32}
    deltaTime::Float32
    cameraConfidence::Float32

    PositionalData() = new(0, 0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0)
end

firstMagValue = -1

function convertDictToPosData(dict::Dict, rotateCameraCoords::Bool)
    posData = PositionalData()
        
    posData.steerAngle = dict["steerAngle"]
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

    return posData
end

function loadFromJSon(rotateCameraCoords::Bool, path::String)
    posData = StructArray(PositionalData[])
    posDataDicts = JSON.parsefile(path, dicttype=Dict, inttype=Int64)

    for dict in posDataDicts        
        push!(posData, convertDictToPosData(dict, rotateCameraCoords))
    end
    
    return posData
end
