# This file contains descriptions of several structs.

export PositionalData
"""
Struct to store the positional data acquired by the AT-RP.

# Fields 
- `steerAngle::Integer`: The steering angle as set in control of the robot. 
- `sensorAngle::Integer`: The steer angle as reported by the steer sensor. 
- `maxSpeed::Float32`: Max speed of the robt (19-40) as arbitrary speed value. 
- `sensorSpeed::Float32`: The speed measured by the wheel encoder. 
- `cameraPos::Vector{Float32}`: The position as estimated by the camera.
- `cameraOri::Vector{Float32}`: The camera orientation as quaternion.
- `imuGyro::Vector{Float32}`: The angular velocity in all 3 axis of gyroscope.
- `imuAcc::Vector{Float32}`: The acceleration in all 3 axis of accelerometer.
- `imuMag::Vector{Float32}`: The magnetic field strength in all 3 axis of magnetometer.
- `deltaTime::Float32`: The time passed since last state update. 
- `cameraConfidence::Float32`: The camera confidence value as reported by VO-System.
- `command::String`: The command send to the robot. 
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
    command::String

    PositionalData() = new(0, 0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0, String(""))
end

export PositionalState
"""
This struct describes the positional state used throughout the project. 
# Fields
- `position::Vector{Float32}`: The position of the robot.
- `v::Float32`: The speed value.
- `Ψ::Float32`: The yaw angle.
- `θ::Float32`: The pitch angle. 
- `ϕ::Float32`: The roll angle.
- `P_c::Matrix{Float32}`: The covariance matrix for the KF updating camera data.
- `P_g::Matrix{Float32}`: The covariance matrix for the KF updating gyroscope data.
- `Σ::Matrix{Float32}`: The covariance matrix for the UKF.
- `Χ::Vector{Vector{Float32}}`: The sigma points for the UKF.
"""
mutable struct PositionalState
    position::Vector{Float32}
    v::Float32    
    Ψ::Float32
    θ::Float32
    ϕ::Float32
    P_c::Matrix{Float32}
    P_g::Matrix{Float32}
    Σ::Matrix{Float32}
    Χ::Vector{Vector{Float32}}
end

export PredictionParameters
"""
This struct holds the parameters that influence the estimation.
"""
mutable struct PredictionParameters
    kalmanFilterCamera::Bool
    kalmanFilterGyro::Bool
    UKF::Bool
    exponentCC::Float32
    useSinCC::Bool          
    speedExponentCC::Float32   
    speedUseSinCC::Bool
    steerAngleFactor::Float32
    odoSteerFactor::Float32
    odoGyroFactor::Float32
    odoMagFactor::Float32
    processNoiseC::Float32
    measurementNoiseC::Float32
    processNoiseG::Float32
    measurementNoiseG::Float32
    processNoiseS::Float32
    measurementNoiseS::Float32
    σ_forSpeedKernel::Float32   
    ΨₒmagInfluence::Bool
    κ::Float32
    α::Float32

    # Beginning parameters (Maybe change with random restart)
    PredictionParameters() = new(false, false, true, 5, false, 5, false, 1.0, 0.33, 0.66, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 1.0, 1/3, false, 1.0, 0.003)

    PredictionParameters(params::PredictionParameters) = new(
        params.kalmanFilterCamera,
        params.kalmanFilterGyro,
        params.UKF, 
        params.exponentCC,
        params.useSinCC,
        params.speedExponentCC,
        params.speedUseSinCC,
        params.steerAngleFactor,
        params.odoSteerFactor,
        params.odoGyroFactor,
        params.odoMagFactor,
        params.processNoiseC,
        params.measurementNoiseC,
        params.processNoiseG,
        params.measurementNoiseG,
        params.processNoiseS,
        params.measurementNoiseS,
        params.σ_forSpeedKernel,
        params.ΨₒmagInfluence,
        params.κ,
        params.α
    )

    PredictionParameters(kalmanFilterCamera, kalmanFilterGyro, UKF, exponentCC, useSinCC, speedExponentCC, speedSinCC, steerAngleFactor, odoSteerFactor, odoGyroFactor, odoMagFactor, processNoiseC, measurementNoiseC, processNoiseG, measurementNoiseG, processNoiseS, measurementNoiseS, σ_forSpeedKernel, ΨₒmagInfluence, κ, α) = new(
        kalmanFilterCamera, kalmanFilterGyro, UKF, exponentCC, useSinCC, speedExponentCC, speedSinCC, steerAngleFactor, odoSteerFactor, odoGyroFactor, odoMagFactor, processNoiseC, measurementNoiseC, processNoiseG, measurementNoiseG, processNoiseS, measurementNoiseS, σ_forSpeedKernel, ΨₒmagInfluence, κ, α
    )
end