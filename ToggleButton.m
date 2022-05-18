classdef ToggleButton < handle
   properties
      State = false
   end
   events
      ToggledState
   end
   methods
      function OnStateChange(obj,newState)
         if newState ~= obj.State
            obj.State = newState;
            notify(obj,'ToggledState');
         end
      end
   end
end