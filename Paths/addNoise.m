function [V,W] = addNoise(V, W, N)
%ADDNOISE Summary of this function goes here
%   Detailed explanation goes here

    for k = 3:N

        V(k) = V(k)  + normrnd(0, 0.1 * V(k));
        if abs(W(k)) < 0.1
            W(k) = W(k) + normrnd(0,0.02);
        else

            W(k) = W(k) + normrnd(0, 0.1 * abs(W(k)));
        end
        
    end

end

