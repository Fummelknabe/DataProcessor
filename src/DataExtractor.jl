using JSON

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

function loadParamsFromJSon(filePath::String)
    params = PredictionParameters()
    paramsDict = JSON.parsefile(filePath, dicttype=Dict, inttype=Int64)

    params.exponentCC = paramsDict["exponentCC"]
    params.speedExponentCC = paramsDict["speedExponentCC"]
    params.kalmanFilterCamera = paramsDict["kalmanFilterCamera"]
    params.kalmanFilterGyro = paramsDict["kalmanFilterGyro"]
    params.measurementNoiseC = paramsDict["measurementNoiseC"]
    params.measurementNoiseG = paramsDict["measurementNoiseG"]
    params.processNoiseC = paramsDict["processNoiseC"]
    params.processNoiseG = paramsDict["processNoiseG"]
    params.odoGyroFactor = paramsDict["odoGyroFactor"]
    params.odoMagFactor = paramsDict["odoMagFactor"]
    params.odoSteerFactor = paramsDict["odoSteerFactor"]
    params.steerAngleFactor = paramsDict["steerAngleFactor"]
    params.speedSinCC = paramsDict["speedSinCC"]
    params.useSinCC = paramsDict["useSinCC"]
    params.σ_forSpeedKernel = paramsDict["σ_forSpeedKernel"]
    params.ΨₒmagInfluence = paramsDict["ΨₒmagInfluence"]

    return params
end

function saveParamsJSon(params::PredictionParameters; fileName::Union{String, Nothing}=nothing)
    name = isnothing(fileName) ? "pred_params" : fileName
    open("params/$(name).json", "w") do ioStream
        JSON.print(ioStream, params, 4)
    end
end
