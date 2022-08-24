module DataProcessor

using StructArrays

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
function train()
    len = length(trainData)
    if len == 0
        @error "No data was added to the stack! Cannot train."
        return
    end

    @info "Training with $(len) data points..."
end

include("Structs.jl")
include("Sensorfusion.jl")
include("DataExtractor.jl")
trainData = Vector{typeof(StructArray(PositionalData[]))}(undef, 0);

end # module
