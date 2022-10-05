function Visualize(Mean, Sig, NLandmarks, N, Map, X, Y, PoseMeans, PoseCovs, C)
%VISUALIZE Summary of this function goes here
%   Detailed explanation goes here
    figure()

    AuxSig2 = zeros(2);

    p = plot(Map(1, 1:NLandmarks), Map(2, 1:NLandmarks), 'x', 'color', 'm');

    set(p, 'MarkerSize',10);
    hold all

    p = plot(X(1:N), Y(1:N), '+', 'color', 'k');

    set(p, 'MarkerSize',8);
    hold all

    p = plot(PoseMeans(1,2:N), PoseMeans(2,2:N), 'o', 'color', 'k');

    set(p, 'MarkerSize',8);
    hold all

    for cnt = 2:N

        [myX, myY] = cov2elli(PoseMeans(1:2,cnt),PoseCovs(1:2,1:2,cnt),2,20);
        line(myX, myY, 'color', 'r');

    end

    for lm = 1: size(C, 1)

         AuxSig2(1:2,1:2) = Sig(2 + lm*2 : 3 + lm*2, 2 + lm*2 : 3 + lm*2);


        [myX, myY] = cov2elli(Mean(lm*2 + 2: lm*2 + 3),AuxSig2,2,20);
        line(myX, myY, 'color', 'b');

    end
    
    
end

