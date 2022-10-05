% The actual algorithm. Currently only with motion update
% See probabilistic robotics, chapter 10

% See EKF_SLAM_KC2 in parent folder for commented code, if necessary
function [mean, sig, Cs] = EKF_SLAM_KC(NLandmarks, prevMean, prevSig, u, Cs, Zs, CEmpty, ZsEmpty)

    % Fx from the book
    Fx = zeros(3, 2*NLandmarks + 3);
    Fx(1, 1) = 1;
    Fx(2, 2) = 1;
    Fx(3, 3) = 1;

    if u(2) == 0
        motionUpdate = [u(1) * cos(prevMean(3));
                        u(1) * sin(prevMean(3));
                        0];
                    
    % The general case which is also in the book
    else
        motionUpdate = [ -(u(1)/u(2))*sin(prevMean(3)) + (u(1)/u(2))*sin(prevMean(3) + u(2));
                     (u(1)/u(2))*cos(prevMean(3)) - (u(1)/u(2))*cos(prevMean(3) + u(2));
                     u(2)];    
    end

    % Mean update
    mean = prevMean + Fx' * motionUpdate;
    
    % w = 0 case, also not in the book. Since the general case seemed to me
    % to be the derivative of motionUpdate, I put the derivative of the w =
    % 0 case.
    if u(2) == 0
        g = [0 0 u(1) * sin(prevMean(3));
            0 0 (-u(1) * cos(prevMean(3)));
            0 0 0];
    else
        % The general case which is also in the book
        g = [ 0 0 (u(1)/u(2))*cos(prevMean(3)) - (u(1)/u(2))*cos(prevMean(3) + u(2));
              0 0 (u(1)/u(2))*sin(prevMean(3)) - (u(1)/u(2))*sin(prevMean(3) + u(2));
              0 0 0];
    end

    % From the book
    G = eye(2*NLandmarks + 3) + Fx' * g * Fx;
     
    % From the motion model part of the book, the velocity model. Not sure
    % if makes ANY sense, but it's not crucial, I think...

    if u(2) == 0
        R1 = 0.1 * abs(u(1) * cos(prevMean(3)))   + 0.5;
        R2 = 0.1 * abs(u(1) * sin(prevMean(3)))   + 0.5;
        R3 = 0.02;
    else
        R1 = 0.1 * abs(-(u(1)/u(2))*sin(prevMean(3)) + (u(1)/u(2))*sin(prevMean(3) + u(2))) + 0.5;
        R2 = 0.1 * abs((u(1)/u(2))*cos(prevMean(3)) - (u(1)/u(2))*cos(prevMean(3) + u(2))) + 0.5;
        R3 = 0.1 * abs(u(2)) + 0.02;
    end
    
    R = [ R1^2 0 0;
          0 R2^2 0;
          0 0 R3^2];

    % Covariance matrix update
    sig = G * prevSig * G' + Fx' * R * Fx;
    
    sigPhi = 0.05;
    
    Q = [ 0 0;
          0 sigPhi^2];
      
    if ZsEmpty ~= 1
        ZHs = zeros(3, size(Zs, 2));
        Js  = zeros(size(Zs, 2),1);
        K = zeros(size(Zs, 2), 2 * NLandmarks + 3, 2);
        H = zeros(size(Zs, 2), 2, 2*NLandmarks + 3);
        newIfOne = zeros(size(Zs,2),1);
        for n = 1:size(Zs, 2)
            j = size(Cs,1) + 1;

            if CEmpty ~= 1    %inutil?
                for c = 1: size(Cs, 1)
                    if Zs(3, n) == Cs(c, 1)
                        j = c;
                        break;
                    end

                end

            else
                j = 1; 
            end
            
            
            if j == size(Cs,1) + 1
                mean(2*j + 2) = mean(1) + Zs(1,n) * cos(Zs(2,n) + mean(3));
                mean(2*j + 3) = mean(2) + Zs(1,n) * sin(Zs(2,n) + mean(3));
                
                %Acrescentar ao Cs
                Cs(j, 1) = Zs(3,n);
                
                newIfOne(n) = 1;
                
                if CEmpty == 1
                    CEmpty = 0;
                end
            end
            
            Q(1,1) = (0.02 * Zs(1, n))^2 + 0.2; 
            
            Js(n,1) = j;
            
            del = [mean(2*j + 2) - mean(1); 
                   mean(2*j + 3) - mean(2)];
            q = del' * del;
            
            ZHs(1, n) = sqrt(q);
            ZHs(2, n) = atan2(del(2), del(1)) - mean(3);
            
            ZHs(2, n) = FixAngle(ZHs(2,n));
            Zs(2, n) = FixAngle(Zs(2,n));
              
            ZHs(3, n) = Cs(j,1);
            
            Fxj = zeros(5, 2 * NLandmarks + 3);
            Fxj(1, 1) = 1;
            Fxj(2, 2) = 1;
            Fxj(3, 3) = 1;
            Fxj(4, 2 * j + 2) = 1;
            Fxj(5, 2 * j + 3) = 1;            
                    
            H1 = [ -sqrt(q) * del(1), -sqrt(q) * del(2), 0, sqrt(q) * del(1), sqrt(q) * del(2)];
            H2 = [ del(2), -del(1), -q, -del(2), del(1)];
                                    
            L = ((1/q) * [H1; H2])* Fxj;
            
            H(n, :, :) = L(:, :);
            
            A(:,:) = H(n, :, :);

            MV = L * sig * L' + Q;
      
            B = (sig * L')/(MV);    %podia ser com L, nao A
            
            K(n, 1: size(B, 1), 1:size(B,2)) = B(1: size(B, 1), 1:size(B,2));
            
            if newIfOne(n) ~= 1
                B(:,:) = K(n, :, :);

                res = (Zs(:, n) - ZHs(:,n));
                res(2,1) = FixAngle(res(2,1));
                res = res(1:2);

                mean = mean + B * res;

            end
            
            D = B * A;
            if newIfOne(n) ~= 1
                sig = (eye(2 * NLandmarks + 3) - D) * sig;
            else

                Ll = [cos(mean(3)) * cos(Zs(2, n)) - sin(mean(3))*sin(Zs(2, n)), -Zs(1, n)*(cos(mean(3))*sin(Zs(2, n)) + sin(mean(3))*cos(Zs(2, n)));
                    sin(mean(3)*cos(Zs(2, n)) + cos(mean(3))*sin(Zs(2, n))), Zs(1, n)*(-sin(mean(3))*sin(Zs(2, n)) + cos(mean(3))*cos(Zs(2, n)))];                            

                Lr = [1, 0, -Zs(1, n)*sin(Zs(2, n))*cos(Zs(2, n))-Zs(1, n)*cos(Zs(2, n))*sin(Zs(2, n));
                    0, 1, Zs(1, n) * cos(Zs(2, n))^2-Zs(1, n)*sin(Zs(2, n))^2];

                matr = Lr * sig( 1:3, :);
                
                sig(2*Js(n,1) + 2: 2*Js(n,1) + 3, :) = matr;

                sig(:, 2 * Js(n,1) + 2: 2 * Js(n,1) + 3) = sig(2*Js(n,1) + 2: 2*Js(n,1) + 3, :)';
                sig(2*Js(n,1) + 2: 2*Js(n,1) + 3, 2*Js(n,1) + 2: 2*Js(n,1) + 3) = Lr * sig(1:3, 1:3) * Lr' + Ll * Q * Ll';
            
            end
            
        end
        
    end

end



