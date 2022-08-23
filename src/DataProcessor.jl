module DataProcessor

using StructArrays

data = Vector{StructArray(PositionalData[])}

export loadDataToStack
"""
Loads a new JSOn File in RAM as 
"""
function loadDataToStack(path::String)
    push!(data, loadFromJSon(true, path))
end

include("DataExtractor.jl")

end # module
