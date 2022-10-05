using JSON

firstMagValue = -1

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

# This method is used for laoding true pos
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

function saveParamsJSon(params::PredictionParameters; fileName::Union{String, Nothing}=nothing)
    name = isnothing(fileName) ? "pred_params" : fileName
    open("params/$(name).json", "w") do ioStream
        JSON.print(ioStream, params, 4)
    end
end
