classdef WaterDrop % < handle % inherit from handle to allow pass by reference?
% Intelligent Water Drop Agents
% This class defines an agent for solving the multiple depot capacitated
% vehicle routing problem (MDVRP). It defines the static and dynamic 
% parameters held within each agent. Also defined are functions for updating
% the dynamic parameters.

   properties(SetObservable = true)
   % public properties
      soil = 0;
      vel = 0;
      node = 0;
      node_prev = 0;
   end
   
   properties(GetAccess = 'public', SetAccess = 'private')
   % read-only
      % velocity parameters
      a_v = 0;
      b_v = 0;
      c_v = 0;
      
      depot = 0;
      capacity = 0;
      
      epsilon = 0.001
   end
   
   methods
   % public by default
   
      function obj = WaterDrop(depot, capacity, a_v, b_v, c_v)
      % class constructor
         obj.depot = depot;
         obj.capacity = capacity;
         
         % velocity params
         obj.a_v = a_v;
         obj.b_v = b_v;
         obj.c_v = c_v;
      end

      function obj = updateLocation(obj, node)
         obj.node_prev = obj.node;
         obj.node = node;
      end
      function obj = updateSoil(obj, edge)
         obj.soil = obj.soil + 0.1;%edge.soil(node, node_prev);
      end
      function obj = updateVelocity(obj, edge)
         obj.vel = obj.vel + a_v/(b_v + c_v*0.01);%edge.soil(node, node_prev));
      end
   end
end
