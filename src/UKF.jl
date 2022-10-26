#= Use as State: 
    x = [p_x, p_y, p_z, Ψ, Θ]
    # The Roll angle is not used here as it is not relevant for position change
   Use as Input:
    u = [w_x, w_y, w_z, v, dt, steerAngle]
=#
function f(s::Vector{Float32}, u::Vector{Float32})
    Ψₜ = Ψ(s[4], u[5], u[6], β(u[6]), u[4])
    θₜ = θ_ang(s[5], u[5], [u[1], u[2], u[3]])
    ṡ = changeInPosition([u[1], u[2], u[3]], u[4], Ψₜ, θₜ, u[5], β=β(u[6]))
    p = s[1:3] + u[5]*ṡ

    return [p[1], p[2], p[3], Ψₜ, θₜ]
end

"""
This performs the whole prediction step for the UKF.

# Arguments
- `μₜ₋₁::Vector{Float32}`: The previous mean e.g. state.
- `wₘ::Vector{Float32}`: The weights for the mean.
- `wₖ::Vector{Float32}`: The weights for the covariance.
- `Χₜ₋₁::Vector{Vector{Float32}}`: The previous Sigma points.
- `uₜ::Vector{Float32}`: The input for the system.
- `Σₜ₋₁::Matrix{Float32}`: The covariance of previous estimation.
- `p::PredictionSettings`: Parameters to influence estimation.

# Returns
- `μₜ̇`: Predicted state.
- `Χₜ`: Next Sigma Points.
- `Σₜ`: New covariance.
"""
function UKF_prediction(μₜ₋₁::Vector{Float32}, wₘ::Vector{Float32}, wₖ::Vector{Float32}, Χₜ₋₁::Vector{Vector{Float32}}, uₜ::Vector{Float32}, Σₜ₋₁::Matrix{Float32}, p::PredictionParameters)
     F = Matrix{Float32}(undef, n, 0)
    for i ∈ 0:2*n  F = hcat(F, f(Χₜ₋₁[i+1], uₜ)) end
    μₜ̇ = sum(wₘ[i+1]*F[:, i+1] for i ∈ 0:2*n)

    Χₜ = generateSigmaPoints(μₜ₋₁, Σₜ₋₁, p)

    Σₜ = sum(wₖ[i+1]*(F[:, i+1] - μₜ̇)*transpose(F[:, i+1] - μₜ̇) for i ∈ 0:2*n) + p.processNoiseS*Matrix(I, size(Σₜ₋₁))# not sure if this should be added inside of sum

    return μₜ̇, Χₜ, Σₜ
end


"""
Update step of the unscented Kalman Filter.

# Returns
- `μₜ`: The new mean state.
- `Σₜ`: The new covariance.
"""
function UKF_update(μₜ̇::Vector{Float32}, wₘ::Vector{Float32}, wₖ::Vector{Float32}, Χₜ::Vector{Vector{Float32}}, Σₜ̇::Matrix{Float32}, p::PredictionParameters, measurement::Vector{Float32}, ratedCC::Float32)
    Zₜ = Matrix{Float32}(undef, n, 0)
    for i ∈ 1:2*n+1
        Zₜ = hcat(Zₜ, Χₜ[i])# normally: Hₛ*Χₜ[i]
    end
    zₜ = sum(wₘ[i+1]*Zₜ[:, i+1] for i ∈ 0:2*n)

    Sₜ = sum(wₖ[i+1]*(Zₜ[:, i+1] - zₜ)*transpose(Zₜ[:, i+1] - zₜ) for i ∈ 0:2*n) + ((1-ratedCC)*p.measurementNoiseS)*Matrix(I, n, n)        

    # calculate Kalman gain
    Kₜ = sum(wₖ[i+1]*(Χₜ[i+1] - μₜ̇)*transpose(Zₜ[:, i+1] - zₜ) for i ∈ 0:2*n) * Float32.(inv(Float64.(Sₜ))) 

    μₜ = μₜ̇ + Kₜ*(measurement - zₜ)
    Σₜ = Σₜ̇ - Kₜ*Sₜ*transpose(Kₜ)
    Σₜ[isnan.(Σₜ)] .= 0.0
    Σₜ[isinf.(Σₜ)] .= 1e10

    return μₜ, Σₜ
end

"""
Compute the weights for the unscented transform.

# Arguments
- `mean::Bool`: If true computes weights for mean, if not for covariance.
- `params::PredictionSettings`: Parameters to influence estimation.
"""
function computeWeights(mean::Bool, p::PredictionParameters)
    w = Vector{Float32}(undef, 0)
    λ = p.α^2*(n + p.κ) - n

    if mean push!(w, λ/(n+λ))
    else push!(w, λ/(n+λ) + (1-p.α^2+2)) end

    for i ∈ 1:2*n
        push!(w, 1/(2*(n+λ)))
    end 
    return w
end

"""
Generate Sigma points from previous mean and covariance.

# Arguments
- `μₜ::Vector{Float32}`: The previous state containing the position and θ, Ψ
- `Σₜ::Matrix{Float32}`: The covariance of previous estimation.
- `p::PredictionSettings`: Parameters to influence estimation.
"""
function generateSigmaPoints(μₜ₋₁::Vector{Float32}, Σₜ₋₁::Matrix{Float32}, p::PredictionParameters)
    # Define vector holding sigma points
    Χ = Vector{Vector{Float32}}(undef, 0)

    # Add the last mean
    push!(Χ, μₜ₋₁)

    λ = p.α^2*(n + p.κ) - n
    
    matrixRoot = real.(sqrt((n + λ) * Σₜ₋₁))
    # This is not easily done as alot of matrix operations lead to inaccuray
    # So receiving matrix is often not hermitian
    #matrixRoot = cholesky((n + λ) * Σₜ₋₁).U

    # Add the remaining sigma points spread around mean
    for i ∈ 1:n
        push!(Χ, μₜ₋₁ + matrixRoot[:, i])#round.(μₜ₋₁ + matrixRoot[:, i], digits=4))
    end

    for i ∈ 1:n
        push!(Χ, μₜ₋₁ - matrixRoot[:, i])#round.(μₜ₋₁ - matrixRoot[:, i], digits=4))
    end
    return Χ
end
