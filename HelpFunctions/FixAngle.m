function Ang = FixAngle(Ang)
%FIXANGLE Summary of this function goes here
%   Detailed explanation goes here
    if Ang > pi
        Ang = Ang - 2*pi;
    elseif Ang < -pi
        Ang = Ang + 2*pi;
    end
       
end

