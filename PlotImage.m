function PlotImage(img, X, Y, Z)
%PlotEnvironment Plot floor and wall
%   Use surf to embed image in Matlab
% X, Y, Z has form [x1, x2; x3, x4]
    surf(X,Y,Z,'CData',imread(img),'FaceColor','texturemap');
end

