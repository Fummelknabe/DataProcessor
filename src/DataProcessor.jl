module DataProcessor

using StructArrays
using LinearAlgebra

export loadDataToStack
"""
Loads a new JSOn File in RAM as Struct Array from PositionalData.
C:/Users/Hurensohn/Documents/UniKrams/Bachelorarbeit/SensorFusionBA_ATRP/data/pos_data.json

# Arguments
- `path::String`: The path to the .json file where the data is stored. 
"""
function loadDataToStack(path::String)
    posData = loadFromJSon(true, path);
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

    while meanError > minError && i < maxIterations
        inner_i = 0
        do 
            P = getNewParams(params)
            newMeanError = Inf64

            for p ∈ P
                e = calculateError(trainData, p)

                # if new error is smaller take parameter
                if e < newMeanError
                    newMeanError = e
                    params = p
                end
            end
            inner_i += 1
        while newMeanError > meanError && inner_i < maxIterChangeParams end

        if inner_i == maxIterChangeParams
            @info "No better Value was found -> local minima!"
            return params
        end

        i += 1
    end

    return params
end

include("Structs.jl")
include("Sensorfusion.jl")
include("HillClimbing.jl")
include("DataExtractor.jl")
trainData = Vector{typeof(StructArray(PositionalData[]))}(undef, 0);

end # module
