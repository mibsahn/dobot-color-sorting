classdef guiListener < handle
    properties
        e = 0;
    end
    methods
       function obj = guiListener(gui)
         addlistener(gui,'eStopPressed',@guiListener.handleEvnt);
       end
   end
   methods (Static)
      function handleEvnt(src,~)
         if src.eStop
             clc
            e = vertex
            disp('STOP')
         else
            e = 0
            disp('RESUME')
         end
      end
   end
end