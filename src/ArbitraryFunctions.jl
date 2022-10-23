export estimateWithAngularVel
function estimateWithAngularVel(posData::StructVector{PositionalData}, fileName::String; correctOffset::Bool=false)
    estimation = Matrix{Float32}(undef, 3, 1)
    offset = Vector{Float32}(undef, 3)

    for i ∈ eachindex(posData)        
        ω = posData[i].imuGyro
        δt = posData[i].deltaTime
        if correctOffset && i == 1 offset = ω end

        estimation = hcat(estimation, [estimation[1, i] + (ω[1] - offset[1])*δt; estimation[2, i] + (ω[2] - offset[2])*δt; estimation[3, i] + (ω[3] - offset[3])*δt])
    end

    # save matrix as .data file 
    open(fileName*".data", "w") do io             
        for i ∈ eachindex(posData)
            if i == 1
                s = String("i roll pitch yaw")
                write(io, s*"\n");
            end

            s = String("")

            newOrientation = estimation[:, i]

            s = s*string(i)*" "
            s = s*string(newOrientation[1])*" "
            s = s*string(newOrientation[2])*" "
            s = s*string(newOrientation[3])*" "

            write(io, s*"\n");
        end
    end;
end

export translation
function translation(t::Vector{Float64})
    length(t) < 2 && throw(ErrorException("Vector has to be of size 2!"))
    return [1.0 0.0 t[1]; 0.0 1.0 t[2]; 0.0 0.0 1.0]
end

export rotation 
function rotation(α::Float64)
    return [cos(α) -sin(α) 0.0; sin(α) cos(α) 0.0; 0.0 0.0 1.0]
end