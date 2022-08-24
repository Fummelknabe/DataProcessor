
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

function getNewParams(params::PredictionParameters)
    possibleParams = Vector{PredictionParameters}(undef, 0)

    params.exponentCC += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.exponentCC -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.exponentCC += 0.1

    params.speedExponentCC += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.speedExponentCC -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.speedExponentCC += 0.1

    params.odoGyroFactor += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.odoGyroFactor -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.odoGyroFactor += 0.1

    params.odoMagFactor += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.odoMagFactor -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.odoMagFactor += 0.1

    params.odoSteerFactor += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.odoSteerFactor -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.odoSteerFactor += 0.1

    params.steerAngleFactor += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.steerAngleFactor -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.steerAngleFactor += 0.1

    params.σ_forSpeedKernel += 0.1
    push!(possibleParams, PredictionParameters(params))
    params.σ_forSpeedKernel -= 0.2
    push!(possibleParams, PredictionParameters(params))
    params.σ_forSpeedKernel += 0.1

    params.speedSinCC = !params.speedSinCC
    push!(possibleParams, PredictionParameters(params))
    params.speedSinCC = !params.speedSinCC

    params.useSinCC = !params.useSinCC
    push!(possibleParams, PredictionParameters(params))
    params.useSinCC = !params.useSinCC

    params.kalmanFilterCamera = !params.kalmanFilterCamera
    push!(possibleParams, PredictionParameters(params))
    params.kalmanFilterCamera = !params.kalmanFilterCamera  
    
    params.kalmanFilterGyro = !params.kalmanFilterGyro
    push!(possibleParams, PredictionParameters(params))
    params.kalmanFilterGyro = !params.kalmanFilterGyro  

    if params.kalmanFilterCamera         
        params.processNoiseC += 0.1
        push!(possibleParams, PredictionParameters(params))
        params.processNoiseC -= 0.2
        push!(possibleParams, PredictionParameters(params))
        params.processNoiseC += 0.1

        params.measurementNoiseC += 0.1
        push!(possibleParams, PredictionParameters(params))
        params.measurementNoiseC -= 0.2
        push!(possibleParams, PredictionParameters(params))
        params.measurementNoiseC += 0.1
    end

    if params.kalmanFilterGyro        
        params.processNoiseG += 0.1
        push!(possibleParams, PredictionParameters(params))
        params.processNoiseG -= 0.2
        push!(possibleParams, PredictionParameters(params))
        params.processNoiseG += 0.1

        params.measurementNoiseG += 0.1
        push!(possibleParams, PredictionParameters(params))
        params.measurementNoiseG -= 0.2
        push!(possibleParams, PredictionParameters(params))
        params.measurementNoiseG += 0.1
    end

    return possibleParams
end