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
      route = [];
   end
   
   properties(GetAccess = 'public', SetAccess = 'private')
   % read-only
      % velocity parameters
      a_v = 0;
      b_v = 0;
      c_v = 0;
      
      capacity = 0;
      
      epsilon = 0.001;
      epsilon_s = 0.01;
   end
   
   methods
   % public by default
   
      function obj = WaterDrop(depot, capacity, velocity, a_v, b_v, c_v)
      % class constructor
         obj.route = [obj.route; depot];
         obj.capacity = capacity;
         obj.vel = velocity;
         
         % velocity params
         obj.a_v = a_v;
         obj.b_v = b_v;
         obj.c_v = c_v;
      end

      function obj = updateSoil(obj, soil_mat)
         obj.soil = obj.soil + 0.1;%edge.soil(node, node_prev);
      end

      function obj = updateVelocity(obj, soil_mat)
         n_idx = length(obj.route);
         obj.vel = obj.vel + a_v/(b_v + c_v*soil_mat(route(n_idx), route(n_idx - 1)));
      end

      function obj = flow(obj, soil_mat, customers) % custs that haven't been visited
         probs = pathProbs(soil_mat, customers);
         node = selectPath(probs);

         obj.route = [obj.route; node];
      end
   end
   
   methods(Access = 'private')
   % private methods
      
      function path = selectPath(obj, probs)
      % select a path from probabilities of possible edges
         r = rand();
         [sorted idx] = sort(probs,'descend');
         runsum = 0;
         for i = 1:length(sorted)
            runsum = runsum + sorted(i);
            if r < runsum
               path = idx(i);
            end
         end
      end
      
      function probs = pathProbs(obj, soil_mat, customers)
      % calculate the probabilities of traversing possible paths
         invsoils = zeros(length(customers),1);
         for i = 1:length(customers)
            invsoils(i) = invSoil(customers(i), soil_mat, customers);
         end
         sumsoils = sum(invsoils);
         
         probs = zeros(length(customers,1));
         for i = 1:length(customers)
            f = invSoil(customers(i), soil_mat, customers);
            probs(i) = f/sumsoils;
         end
      end

      function f = invSoil(obj, customer, soil_mat, customers)
      % inverse term calculation
         f = 1/(epsilon_s + positiveSoil(customer, soil_mat, customers));
      end

      function g = positiveSoil(obj, customer, soil_mat, customers)
      % term calculation
         soils = zeros(length(customers),1);
         for i = 1:length(customers)
            soils(i) = soil_mat(customer,customers(i));
         end
         min_s = min(soils);
         
         if min_s >= 0
            g = soils(customer);
         else
            g = soils(customer) - min_s;
         end
      end
   end
end
