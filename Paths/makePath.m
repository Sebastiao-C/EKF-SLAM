
function [X, Y, A, W, V, Z] = makePath(X, Y, A, W, V, N, NLandmarks, Map, Z, SquareSize, MeasRange)

    % Create a random path
    for k = 2:N

        ang = atan2(Y(k-1), X(k-1));
        if abs(A(k-1)) >= pi
            if A(k-1) > 0
                A(k-1) = A(k-1) - 2*pi;
            else
                A(k-1) = A(k-1) + 2*pi;
            end
        end
        
        diff = A(k-1) - ang;
        
        if abs(diff) >= pi
            if diff > 0
                diff = diff - 2*pi;
            else
                diff = diff + 2*pi;
            end
        end
        
        if (X(k-1)^2 + Y(k-1)^2) > (4/5)*(SquareSize/2)^2
            
            if abs(diff) <= (pi/3)
                if abs(W(k-1)) > 0.4
                    if W(k-1) > 0
                        W(k) = W(k-1) - 0.1;
                    else
                        W(k) = W(k-1) + 0.1;
                    end
                elseif diff > 0
                    W(k) = 1.2;
                else
                    W(k) = -1.2;
                end

            else
                W(k) = normrnd(0, 0.1);
            end


        elseif (X(k-1)^2 + Y(k-1)^2) > (3/5)*(SquareSize/2)^2


            if abs(diff) <= pi/5
                if abs(W(k-1)) > 0.4
                    if W(k-1) > 0
                        W(k) = W(k-1) - 0.1;
                    else
                        W(k) = W(k-1) + 0.1;
                    end
                
                elseif diff > 0
                    W(k) = 0.5;
                else
                    W(k) = -0.5;
                end

            else
                W(k) = normrnd(0, 0.1);
            end

        else
            W(k) = normrnd(0, 0.1);
        end



        % Linear velocity related to previous (by choice, just because)
        %sigma = 1;
        %r = normrnd(0,sigma);
        r = 0;
        V(k) = V(k-1) + r;

        if W(k-1) == 0
            X(k) = X(k-1) + V(k-1) * cos(A(k-1));
            Y(k) = Y(k-1) + V(k-1) * sin(A(k-1));
            A(k) = A(k-1);

        else
            X(k) = X(k-1)  -(V(k-1)/W(k-1))*sin(A(k-1)) + (V(k-1)/W(k-1))*sin(A(k-1) + W(k-1));
            Y(k) = Y(k-1) + (V(k-1)/W(k-1))*cos(A(k-1)) - (V(k-1)/W(k-1))*cos(A(k-1) + W(k-1));
            A(k) = A(k-1) + W(k-1);

        end
        

        count = 0;
        for j = 1:NLandmarks
            d = (X(k)-Map(1,j))^2 + (Y(k)-Map(2,j))^2;
            if d < MeasRange^2
                count = count + 1;

                mt = atan2(Map(2,j)-Y(k), Map(1,j)-X(k)) - A(k);
                if abs(mt) > pi
                    if mt > 0
                        mt = mt - 2*pi;
                    else
                        mt = mt + 2*pi;
                    end
                end

                Z(k, 1:3, count) = [sqrt(d); mt ;j]; % Depois acrescentar ruido
            end
        end

    end
end
