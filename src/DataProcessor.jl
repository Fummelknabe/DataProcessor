module DataProcessor

using StructArrays
using LinearAlgebra
using ProgressMeter

export loadDataToStack
"""
Loads a new JSOn File in RAM as Struct Array from PositionalData.

# Arguments
- `path::String`: The path to the .json file where the data is stored. 
- `rotateCameraCoords::Bool`: Rotate the camera position so it fits onto prediction.
"""
function loadDataToStack(path::String; rotateCameraCoords::Bool=true)
    posData = loadFromJSon(rotateCameraCoords, path);
    if length(posData) == 0
        @warn "No data was added to the stack."
        return
    end

    push!(trainData, posData);
end

export getLength
getLength() = @info "Length of stack: " * string(length(trainData))

export train
function train(maxIterations::Integer=1000, minError::Float64=1.0, maxIterChangeParams=100)
    len = length(trainData)
    if len == 0
        @error "No data was added to the stack! Cannot train."
        return
    end

    # Starting Parameters
    params = PredictionParameters()
    @info "Training with $(len) data points..."
    @info "Start parameter: $(params)"
    
    # Error with starting parameters
    meanError = calculateError(trainData, params)
    @info "Error of starting params: $(meanError)"

    i = 0
    prog = Progress(maxIterations, 1, "Training...", 50)

    while meanError > minError && i < maxIterations
        inner_i = 0
        while true 
            P = getNewParams(params)
            newMeanError = Inf64

            for p ∈ P
                #@info "Current Params: $(p)."
                e = calculateError(trainData, p)

                # if new error is smaller take parameter
                if e < newMeanError
                    newMeanError = e
                    params = p
                end
            end
            inner_i += 1

            # Break out of loop
            if newMeanError < meanError || inner_i == maxIterChangeParams 
                meanError = newMeanError 
                break 
            end
        end

        if inner_i == maxIterChangeParams
            println()
            @info "No better Value was found -> local minima with Parameters: $(params)"
            return params
        end

        i += 1
        next!(prog)
    end

    println()
    @info "Training finished with mean error: $(newMeanError) and Parameters: $(params)"
    return params
end

include("Structs.jl")
include("Sensorfusion.jl")
include("HillClimbing.jl")
include("DataExtractor.jl")
trainData = Vector{typeof(StructArray(PositionalData[]))}(undef, 0);

end # module
