function Map =  makeMap(NLandmarks, squareSize)
    Map = rand(2, NLandmarks) * squareSize;
    Map(:,:) = Map(:,:) - squareSize/2;
end

