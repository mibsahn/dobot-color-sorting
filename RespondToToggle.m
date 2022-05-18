classdef RespondToToggle < handle
   methods
      function obj = RespondToToggle(toggle_button_obj)
         addlistener(toggle_button_obj,'ToggledState',@RespondToToggle.handleEvnt);
      end
   end
   methods (Static)
      function handleEvnt(src,~)
         if src.State
            disp('ToggledState is true')
         else
            disp('ToggledState is false')
         end
      end
   end
end