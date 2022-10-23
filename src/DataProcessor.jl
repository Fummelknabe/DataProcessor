module DataProcessor

using StructArrays
using LinearAlgebra
using ProgressMeter
using Random
using Statistics
using Base.Threads: @threads
using Images
using FileIO
using Colors

include("Structs.jl")

"""
This method loads hardcoded data files apart from data recorded at night.
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
function loadInitialParameters(; numberOfRandomParams::Int=0)
    # changed order of parameters
    addInitialParameter(param="params/initial_params/init_params2.json");
    addInitialParameter(param="params/initial_params/init_params1.json");
    addInitialParameter(param="params/initial_params/init_params3.json");
    addInitialParameter(param="params/initial_params/init_params4.json");
    
    for i ∈ 1:numberOfRandomParams        
        addInitialParameter();
    end
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

            @threads for p ∈ P
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

export calculateAverageParameters
function calculateAverageParameters(params::Vector{Tuple{PredictionParameters, Float64}}; saveAsFile::Bool=true)
    averageParameters = PredictionParameters(false, false, false, 0, false, 0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0)
    
    # collect errors
    errors = Vector{Float64}(undef, 0)
    for p ∈ params push!(errors, p[2]) end

    (maxError, minError) = (maximum(errors), minimum(errors))
    overAllWeights = 0
    for p ∈ params
        weight = (1 - (p[2] - minError)/(2 + maxError - minError))

        averageParameters.exponentCC += weight*p[1].exponentCC
        averageParameters.speedExponentCC += weight*p[1].speedExponentCC
        averageParameters.steerAngleFactor += weight*p[1].steerAngleFactor
        averageParameters.odoSteerFactor += weight*p[1].odoSteerFactor
        averageParameters.odoGyroFactor += weight*p[1].odoGyroFactor
        averageParameters.odoMagFactor += weight*p[1].odoMagFactor
        averageParameters.processNoiseC += weight*p[1].processNoiseC
        averageParameters.measurementNoiseC += weight*p[1].measurementNoiseC
        averageParameters.processNoiseG += weight*p[1].processNoiseG
        averageParameters.measurementNoiseG += weight*p[1].measurementNoiseG
        averageParameters.processNoiseS += weight*p[1].processNoiseS
        averageParameters.measurementNoiseS += weight*p[1].measurementNoiseS
        averageParameters.σ_forSpeedKernel += weight*p[1].σ_forSpeedKernel
        averageParameters.κ += weight*p[1].κ
        averageParameters.α += weight*p[1].α

        if weight == 1 # best parameter
            averageParameters.kalmanFilterCamera = p[1].kalmanFilterCamera
            averageParameters.kalmanFilterGyro = p[1].kalmanFilterGyro
            averageParameters.UKF = p[1].UKF
            averageParameters.useSinCC = p[1].useSinCC
            averageParameters.speedUseSinCC = p[1].speedUseSinCC
            averageParameters.ΨₒmagInfluence = p[1].ΨₒmagInfluence
        end

        overAllWeights += weight
    end
    overAllWeights = Float32(overAllWeights)
    averageParameter = PredictionParameters(averageParameters.kalmanFilterCamera,
                                            averageParameters.kalmanFilterGyro,
                                            averageParameters.UKF,
                                            averageParameters.exponentCC / overAllWeights,
                                            averageParameters.useSinCC,
                                            averageParameters.speedExponentCC / overAllWeights,
                                            averageParameters.speedUseSinCC,
                                            averageParameters.steerAngleFactor / overAllWeights,
                                            averageParameters.odoSteerFactor / overAllWeights,
                                            averageParameters.odoGyroFactor / overAllWeights,
                                            averageParameters.odoMagFactor / overAllWeights,
                                            averageParameters.processNoiseC / overAllWeights,
                                            averageParameters.measurementNoiseC / overAllWeights,
                                            averageParameters.processNoiseG / overAllWeights,
                                            averageParameters.measurementNoiseG / overAllWeights,
                                            averageParameters.processNoiseS / overAllWeights,
                                            averageParameters.measurementNoiseS / overAllWeights,
                                            averageParameters.σ_forSpeedKernel / overAllWeights,
                                            averageParameters.ΨₒmagInfluence,
                                            averageParameters.κ / overAllWeights,
                                            averageParameters.α / overAllWeights)

    if saveAsFile saveParamsJSon(averageParameter, fileName="averaged_params") end
    return averageParameter
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
function saveDataToFile(data::Union{StructArray, Matrix}, filename::String; correctOffset::Bool=false)
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
            offset = Vector{Float32}(undef, 3)         
            for i ∈ eachindex(data)
                if i == 1
                    s = String("i steerAngle sensorAngle maxSpeed sensorSpeed cameraPosX cameraPosY cameraPosZ cameraPosChange cameraOriX cameraOriY cameraOriZ cameraOriW imuGyroX imuGyroY imuGyroZ imuAccX imuAccY imuAccZ imuMagX imuMagY imuMagZ deltaTime cameraConfidence")
                    write(io, s*"\n");
                    correctOffset && global offset = data[i].imuGyro
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
                s = s*transformVecToString(newData.imuGyro - offset)
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

export extractPositionFromPathImage
function extractPositionFromPathImage(path::String, startPos::Tuple{Int, Int}, pixelSize::Float32, filename::String; angleToRotate::Float32=Float32(0.0))
	img = load(path)

    pxlPositions = Matrix{Float32}(undef, 2, 0)
    alreadyVisited = Vector{Tuple{Int, Int}}(undef, 0)

    currentPos = startPos
    found = true

    while found
        found = false    
        candidates = Vector{Tuple{Int, Int}}(undef, 0)
        for x ∈ -1:1
            for y ∈ -1:1
                if x == 0 && y == 0 continue end
                #println(img[currentPos[1]+x, currentPos[2]+y].r)
                if img[currentPos[1]+x, currentPos[2]+y].r >= 0.5 && !((currentPos[1]+x, currentPos[2]+y) in alreadyVisited)
                    push!(alreadyVisited, currentPos)
                    push!(alreadyVisited, (currentPos[1]+x, currentPos[2]+y))
                    push!(candidates, (currentPos[1]+x, currentPos[2]+y))                    
                    found = true
                end
            end
        end
        if length(candidates) == 1
            currentPos = candidates[1]
            pxlPositions = hcat(pxlPositions, [currentPos[1], currentPos[2]])
        elseif length(candidates) > 1 # to disallow 0
            for p ∈ candidates
                if abs(p[1] - currentPos[1]) + abs(p[2] - currentPos[2]) == 2
                    currentPos = p 
                    pxlPositions = hcat(pxlPositions, [currentPos[1], currentPos[2]])
                    break
                end
            end
        end
    end

    @info "End Pixel: $(pxlPositions[:, end])"

    # Now calculate the dimensions with the pixel size

    # define start position as zero
    realPositions = pxlPositions .- pxlPositions[:, 1]
    realPositions = realPositions .* [pixelSize, pixelSize]

    # 55px ~ 80cm => 1 px ~ 1.4545
    # Save real positions matrix as data file 
    open(filename*".data", "w") do io    
        for i ∈ 1:size(realPositions)[2]
            if i == 1
                s = String("i x y")
                write(io, s*"\n");
            end

            s = String("")

            newPos = [cos(angleToRotate) -sin(angleToRotate); sin(angleToRotate) cos(angleToRotate)]*realPositions[:,i]

            s = s*string(i)*" "
            s = s*string(newPos[1])*" "
            s = s*string(newPos[2])*" "

            write(io, s*"\n");
        end
    end;

    return realPositions
end

export modifyTruePos
function modifyTruePos(filename::String, transform::Matrix{Float64})
    lines = Vector{String}(undef, 0)
    try
        lines = readlines(filename*".data")
    catch e
        if e isa SystemError
            @info "The file does not exist!"
            return
        end
        throw(e)
    end
    
    for i ∈ eachindex(lines)
        if i == 1 continue end
        splitted = split(lines[i], " ")
        vec = [parse(Float64, splitted[2]), parse(Float64, splitted[3]), 1.0]
        # transform the data
        vec = transform * vec
        lines[i] = "i $(vec[1]) $(vec[2])"
    end

    open(filename*"_transformed"*".data", "w") do io    
        for i ∈ eachindex(lines)
            write(io, lines[i]*"\n");
        end
    end;
end

include("Sensorfusion.jl")
include("HillClimbing.jl")
include("DataExtractor.jl")
include("ArbitraryFunctions.jl")
trainData = Vector{Tuple{Int64, typeof(StructArray(PositionalData[])), Union{Nothing, Matrix{Float32}}}}(undef, 0);
initialParameters = Vector{PredictionParameters}(undef, 0);

end # module
