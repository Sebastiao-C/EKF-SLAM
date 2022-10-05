function [Mean, Sig, Map, X, Y, PoseMeans, PoseCovs, C, FANGLE] = FullSim(NLandmarks, N, SquareSize, MeasRange)
    % N is the number of "steps"
    addpath 'Paths';
    addpath 'HelpFunctions';

    % Estimations
    Mean = zeros(2*NLandmarks + 3, 1);
    Sig = zeros(2* NLandmarks + 3);

    % Initial angle
    Mean(3) = rand()* 2 * pi;
    FANGLE = Mean(3);

    % Actions for the N steps (vector with u's)
    V = zeros(N, 1);
    W = zeros(N, 1);
    X = zeros(N, 1);
    Y = zeros(N, 1);
    A = zeros(N, 1);
    Z = zeros(N, 3, NLandmarks);

    V(1) = 5;
    W(1) = 0;
    X(1) = 0;
    Y(1) = 0;
    A(1) = Mean(3);


    Map = makeMap(NLandmarks, SquareSize);

    [X, Y, ~, W, V, Z] = makePath(X, Y, A, W, V, N, NLandmarks, Map, Z, SquareSize, MeasRange);

    [Z] = addMeasNoise(Z, N, NLandmarks);
    [V,W] = addNoise(V, W, N);
    U = [V W];

    PoseMeans = zeros(2,N);
    PoseCovs = zeros(2,2,N);

    C = [];

    % Running the algorithm in each step
    for k = 2:N
        numSights = size(find(Z(k, 1:3, 1:NLandmarks)), 1)/3; %Is this right? Seems like it
        Zin = zeros(3, numSights);

        Zin(1:3, 1:numSights) = Z(k, 1:3, 1:numSights);

        if numSights == 0
            Zemp = 1;
        else
            Zemp = 0;
        end

        if size(C,1) == 0
            Cemp = 1;
        else
            Cemp = 0;
        end

        [Mean, Sig, C] = EKF_SLAM_KC(NLandmarks, Mean, Sig, U(k-1, :), C, Zin, Cemp, Zemp);

        Sig = (Sig + Sig')/2;

        PoseMeans(1:2, k) = Mean(1:2);
        % Submatrix of Sig, with only the x and y positions
        AuxSig = zeros(2);
        AuxSig(1:2,1:2) = Sig(1:2,1:2);
        PoseCovs(1:2,1:2,k) = AuxSig(1:2,1:2);


        if k == N      % Mudar este valor para ver como vai variando a elipse à volta do landmark mais à direita (a verde)
            Visualize(Mean, Sig, NLandmarks, N, Map, X, Y, PoseMeans, PoseCovs, C);
        end

    end

end

