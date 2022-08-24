
function calculateError(trainData::Vector{typeof(StructArray(PositionalData[]))}, params::PredictionParameters)
    X = Vector{typeof(StructArray(PositionalState[]))}(undef, 0)

    for d ∈ trainData
        push!(X, predictFromRecordedData(d, params))
    end

    accumulatedError = 0.0
    for x ∈ X
        accumulatedError += norm(x[1].position - x[length(x)].position)
    end

    return accumulatedError / length(X)
end