export PositionalData
"""
Struct to store the data acquired by the AT-RP
"""
mutable struct PositionalData
    steerAngle::Integer
    sensorAngle::Integer
    maxSpeed::Float32
    sensorSpeed::Float32
    cameraPos::Vector{Float32}
    cameraOri::Vector{Float32}    
    imuGyro::Vector{Float32}
    imuAcc::Vector{Float32}
    imuMag::Vector{Float32}
    deltaTime::Float32
    cameraConfidence::Float32

    PositionalData() = new(0, 0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0)
end

export PositionalState
mutable struct PositionalState
    position::Vector{Float32}
    v::Float32
    P_c::Matrix{Float32}
    P_g::Matrix{Float32}
    Ψ::Float32
    θ::Float32
end

export PredictionSettings
mutable struct PredictionSettings
    kalmanFilterCamera::Bool
    kalmanFilterGyro::Bool
    exponentCC::Float32
    steerAngleFactor::Float32
    odoSteerFactor::Float32
    odoGyroFactor::Float32
    odoMagFactor::Float32
    processNoiseC::Float32
    measurementNoiseC::Float32
    processNoiseG::Float32
    measurementNoiseG::Float32
end