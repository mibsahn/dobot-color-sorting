function sugarCane = PlotEnvironment()
%PLOTENVIRONMENT Summary of this function goes here
%   Detailed explanation goes here

PlotImage('Title.jpg',[-0.7 0.7; -0.7 0.7],[0.7 0.7; 0.7 0.7],[0.5 0.5; -0.22 -0.22]);
PlotImage('_0.jpg',[-0.7 -0.7; 0.7 0.7],[-0.7 0.7; -0.7 0.7],[-0.22 -0.22; -0.22 -0.22]);
PlaceObject('Table.ply',[0 0 0]);
PlaceObject('SmallTable.ply',[0.02 -0.28 -0.1]);
PlaceObject('SmallTable.ply',[0.31 0.16 -0.1]);
PlaceObject('Box.ply',[0.02 -0.28 -0.1]);
PlaceObject('Box.ply',[0.30 0 -0.22]);
PlaceObject('SugarCaneJuiceExtractor.ply', [0.31 0 0]);
sugarCane(1) = PlaceObj('SugarCaneR.ply');
sugarCane(1).MoveObj([ [0 -0.3 -0.02] zeros(1,3)]);
sugarCane(2) = PlaceObj('SugarCaneY.ply');
sugarCane(2).MoveObj([ [0 -0.27 -0.02] zeros(1,3)]);
sugarCane(3) = PlaceObj('SugarCaneY.ply');
sugarCane(3).MoveObj([ [0.03 -0.27 -0.02] zeros(1,3)]);

disp('Finished importing environment.')

end

