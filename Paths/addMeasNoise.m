function [Z] = addMeasNoise(Z, N, NLandmarks)
%ADDMEASNOISE Summary of this function goes here
%   Detailed explanation goes here
    for k = 3:N
        for n = 1:NLandmarks
            if Z(k, 3, n) ~= 0
                

                Z(k, 1, n) = Z(k, 1, n) + normrnd(0,Z(k, 1, n) *  0.005);
                Z(k, 2, n) = Z(k, 2, n) + normrnd(0, 0.01);

            else
                break
            end
        end
    end
end

