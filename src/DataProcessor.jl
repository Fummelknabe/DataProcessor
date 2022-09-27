module DataProcessor

using StructArrays
using LinearAlgebra
using ProgressMeter
using Random
using Statistics

include("Structs.jl")

"""
This method loads hardcoded data files.
"""
function loadTrainData()
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data_21_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data_12_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data1_22_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data2_22_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data3_22_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data1_26_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data2_26_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data3_26_09_3.json", 3);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data4_26_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data5_26_09_1.json", 1);
    loadDataToStack("C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/Recorded Data/train_data6_26_09_1.json", 1);
end

"""
This method loads hardcoded parameter files.
"""
function loadInitialParameters()
    addInitialParameter(param="params/initial_params/pred_params1.json");
    addInitialParameter(param="params/initial_params/pred_params2.json");
    addInitialParameter(param="params/initial_params/pred_params3.json");
    addInitialParameter(param="params/initial_params/pred_params4.json");
    addInitialParameter();
    addInitialParameter();
    addInitialParameter();
    addInitialParameter();
end


export loadDataToStack
"""
Loads a new JSOn File in RAM as Struct Array from PositionalData.

# Arguments
- `path::String`: The path to the .json file where the data is stored. 
- `numberOfLoops::Int`: The number of loops the robot made in this data set. 

# Optional Arguments
- `rotateCameraCoords::Bool`: Rotate the camera position so it fits onto prediction.
- `pathPosTracking::String`: The path to the .json file containing the correct positional information for the data.
"""
function loadDataToStack(path::String, numberOfLoops::Int; rotateCameraCoords::Bool=true, pathPosTracking::String="")
    posData = loadFromJSon(rotateCameraCoords, path);
    if length(posData) == 0
        @warn "No data was added to the stack."
        return
    end

    push!(trainData, (numberOfLoops, posData, cmp(pathPosTracking, String("")) == 0 ? nothing : loadFromJSon(pathPosTracking)));
end

export getLength
getLength() = @info "Length of stack: " * string(length(trainData))

export addInitialParameter
"""
This method adds Parameters to a list used for random restart. If no argument is given the parameters will be randomly generated.

# Optional Argument
- `param::Union{String, PredictionParameters}`: The Parameters can be directly provided or the path to the JSON file containing the information.
"""
function addInitialParameter(;param::Union{String, PredictionParameters}=".")
    if param isa String
        if param != "."
            push!(initialParameters, loadParamsFromJSon(param))
            return
        end
    else
        push!(initialParameters, param)
        return
    end

    # Add random parameter
    push!(initialParameters, PredictionParameters(bitrand(1)[1], 
                                                  bitrand(1)[1], 
                                                  bitrand(1)[1],
                                                  rand(Float32, 1)[1]*10, 
                                                  bitrand(1)[1], 
                                                  rand(Float32, 1)[1]*10, 
                                                  bitrand(1)[1], 
                                                  rand(Float32, 1)[1]*0.3, 
                                                  rand(Float32, 1)[1], 
                                                  rand(Float32, 1)[1]+0.01, 
                                                  rand(Float32, 1)[1], 
                                                  rand(Float32, 1)[1]*0.3+0.1, 
                                                  rand(Float32, 1)[1]*100, 
                                                  rand(Float32, 1)[1]*0.3+0.1, 
                                                  rand(Float32, 1)[1]*100, 
                                                  rand(Float32, 1)[1]*0.3+0.1, 
                                                  rand(Float32, 1)[1]*100,
                                                  rand(Float32, 1)[1], 
                                                  bitrand(1)[1],
                                                  bitrand(1)[1]+0.01,
                                                  bitrand(1)[1]*0.01))
end


export train
"""
This function executes the hillclimbing itself. The data to train with should be given in `trainData`. 

# Optional Arguments
- `maxIterations::Integer`: The maximum iterations after which the algorithm terminates.
- `minError::Float64`: The minimum error to achieve if the maximum iterations were not exceded.
- `maxIterChangeParams`: How many iterations should be looked for better parameters. If greater, greater deviation in parameters from initial parameter possible. 
- `saveAsFile::Bool`: If resulting parameters should be saved as a JSON file.
- `randomRestart::Bool`: Restart algorithm with initial parameters provided in `initialParameters`.
- `checkLoops::Bool`: If the number of loops should be checked upon estimation.
"""
function train(;maxIterations::Integer=1000, minError::Float64=1.0, maxIterChangeParams=100, saveAsFile::Bool=false, randomRestart::Bool=false, rri::Integer=1, checkLoops::Bool=true)
    len = length(trainData)
    len == 0 && throw(ArgumentError("No data was added to the stack! Cannot train."))
    (randomRestart && length(initialParameters) == 0) && throw(ArgumentError("For random restart to work, initial parameters must be provided using `addInitialParameter`."))

    # Starting Parameters
    params = randomRestart ? initialParameters[rri] : PredictionParameters()
    @info "Training with $(len) data points..."
    @info "Start parameter: $(params)"
    
    # Error with starting parameters
    meanError = calculateError(trainData, params, checkLoops)
    @info "Error of starting params: $(meanError)"

    i = 0
    prog = Progress(maxIterations, 1, "Training...", 50)

    while meanError > minError && i < maxIterations
        inner_i = 0
        while true 
            # find best parameter from selection P
            P = getNewParams(params)
            newMeanError = Inf64
            localParams = P[1]

            for p ∈ P
                #@info "Current Params: $(p)."
                e = calculateError(trainData, p, checkLoops)

                # if new error is smaller take parameter
                if e < newMeanError
                    newMeanError = e
                    localParams = p
                end
            end
            inner_i += 1

            # Break out of loop
            if newMeanError < meanError 
                meanError = newMeanError 
                params = localParams
                break 
            # No better parameter was found 
            elseif inner_i == maxIterChangeParams 
                break
            end
        end

        if inner_i == maxIterChangeParams
            println()
            @info "No better Value was found -> local minima with Parameters: $(params) with error: $(meanError) after $(i) iteration(s)"

            if saveAsFile saveParamsJSon(params, fileName=randomRestart ? "pred_params$(rri)" : nothing) end

            return (randomRestart && length(initialParameters) > rri) ? train(maxIterations=maxIterations, minError=minError, maxIterChangeParams=maxIterChangeParams, saveAsFile=saveAsFile, randomRestart=true, rri=rri+1) : params
        end

        i += 1
        next!(prog)
    end

    println()
    @info "Training finished with mean error: $(meanError) and Parameters: $(params)"

    if saveAsFile saveParamsJSon(params, fileName=randomRestart ? "pred_params$(rri)" : nothing) end    

    return (randomRestart && length(initialParameters) > rri) ? train(maxIterations=maxIterations, minError=minError, maxIterChangeParams=maxIterChangeParams, saveAsFile=saveAsFile, randomRestart=true, rri=rri+1) : params
end

export calculateAverageSpeed
function calculateAverageSpeed(;data::Union{Nothing, typeof(StructArray(PositionalData[]))}=nothing, leaveZeros::Bool=false)
    # Removing trailing and leading zero speed values from data
    function f(V::Vector{Float32})
        vₒ = Vector{Float32}(undef, 0)
        firstNonZero = false
        for i ∈ 1:length(V)
            if V[i] != 0.0 || (!(V[i:end] == zeros(Float32, length(V[i:end]))) && firstNonZero)
                firstNonZero = true
                push!(vₒ, V[i])
            end
        end
        return vₒ
    end

    if isnothing(data) 
        if length(trainData) == 0 @warn "Stack is empty!" else @info "Using data stored in stack." end

        for d ∈ trainData
            m = mean(leaveZeros ? d[2].sensorSpeed : f(d[2].sensorSpeed))
            println("Average Speed in data set: $(m)")
        end
    else
        m = mean(leaveZeros ? data.sensorSpeed : f(data.sensorSpeed))
        println("Average Speed in data set: $(m)")
    end
end

#helper function
function transformVecToString(v::Vector{T}) where T <: Number
    s = String("")
    for n ∈ v s = s*string(n)*" " end
    return s
end
export saveDataToFile
function saveDataToFile(data::Union{StructArray, Matrix}, filename::String)
    if data isa Matrix
        @info "Save positional [x, y, z] data in file."

        open(filename*".data", "w") do io 
            println(size(data)[2])
            for i ∈ 1:size(data)[2]
                write(io, string(data[1, i])*" "*string(data[2, i])*" "*string(data[3, i])*"\n");
            end
        end;
    elseif data isa StructArray
        @info "Save sensor data from struct in file."

        open(filename*".data", "w") do io             
            for i ∈ 0:length(data)
                if i == 0
                    s = String("i steerAngle sensorAngle maxSpeed sensorSpeed cameraPosX cameraPosY cameraPosZ cameraPosChange cameraOriX cameraOriY cameraOriZ cameraOriW imuGyroX imuGyroY imuGyroZ imuAccX imuAccY imuAccZ imuMagX imuMagY imuMagZ deltaTime cameraConfidence")
                    write(io, s*"\n");
                    continue
                end

                s = String("")

                newData = data[i]

                s = s*string(i)*" "
                s = s*string(newData.steerAngle)*" "
                s = s*string(newData.sensorAngle)*" "
                s = s*string(newData.maxSpeed)*" "
                s = s*string(newData.sensorSpeed)*" "
                s = s*transformVecToString(newData.cameraPos)
                s = s*transformVecToString(newData.cameraOri)
                s = s*transformVecToString(newData.imuGyro)
                s = s*transformVecToString(newData.imuAcc)
                s = s*transformVecToString(newData.imuMag)
                s = s*string(newData.deltaTime)*" "
                s = s*string(newData.cameraConfidence)

                write(io, s*"\n");
            end
        end;
    else
        @warn "Data given is not supported!"
        return
    end    
end

include("Sensorfusion.jl")
include("HillClimbing.jl")
include("DataExtractor.jl")
trainData = Vector{Tuple{Int64, typeof(StructArray(PositionalData[])), Union{Nothing, Matrix{Float32}}}}(undef, 0);
initialParameters = Vector{PredictionParameters}(undef, 0);

end # module
