module DataProcessor

using StructArrays

export loadDataToStack
"""
Loads a new JSOn File in RAM as 
"""
function loadDataToStack(path::String)
    posData = loadFromJSon(true, path)
    if length(posData) == 0
        @warn "No data was added to the stack."
        return
    end

    push!(data, posData)
end

include("DataExtractor.jl")
data = Vector{typeof(StructArray(PositionalData[]))}

end # module
