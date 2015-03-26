classdef WaterDrop % < handle % inherit from handle to allow pass by reference?
% Intelligent Water Drop Agents
% This class defines an agent for solving the multiple depot capacitated
% vehicle routing problem (MDVRP). It defines the static and dynamic 
% parameters held within each agent. Also defined are functions for updating
% the dynamic parameters.

   properties(SetObservable = true)
   % public properties
      soil = 0;
      vel = 100;
      route = [];
   end
   
   properties(GetAccess = 'public', SetAccess = 'private')
   % read-only
      % velocity parameters
      a_v = 1000;
      b_v = 0.01;
      c_v = 1;

      % soil parameters
      a_s = 1000;
      b_s = 0.01;
      c_s = 1;
      
      capacity = 0;
      
      epsilon = 0.001;
      epsilon_s = 0.01;
   end
   
   methods
   % public methods
   
      function obj = WaterDrop(depot, 
                               capacity, 
                               velocity, 
                               a_v, b_v, c_v,
                               a_s, b_s, c_s)
      % class constructor
         obj.route = [obj.route; depot];
         obj.capacity = capacity;
         obj.vel = velocity;
         
         % velocity params
         obj.a_v = a_v;
         obj.b_v = b_v;
         obj.c_v = c_v;
      end

      function obj = updateSoil(obj, dist_mat)
         obj.soil = obj.soil + deltaSoil(dist_mat);
      end

      function obj = updateVelocity(obj, soil_mat)
         n_idx = length(obj.route);
         obj.vel = obj.vel + a_v/(b_v + c_v*soil_mat(route(n_idx - 1), route(n_idx));
      end

      function obj = flow(obj, soil_mat, customers) % custs that haven't been visited
      % probabilistically move to a connected customer
         probs = pathProbs(soil_mat, customers);
         node = selectPath(probs);

         obj.route = [obj.route; node];
      end

      function soil = deltaSoil(obj, dist_mat)
      % soil picked up along most recent path
         soil = obj.a_s/(obj.b_s + obj.c_s*calcTime(dist_mat);
      end

      function cost = calcRouteCost(obj, dist_mat)
         cost = 0;
         for i = 2:length(route)
            cost = cost + dist_mat(route(i - 1), route(i))
         end
      end
   end
   
   methods(Access = 'private')
   % helper methods
      
      function time = calcTime(obj, dist_mat)
      % time required to traverse an edge
         n_idx = length(obj.route);
         time = dist_mat(route(n_idx - 1), route(n_idx))/max(obj.epsilon, obj.vel);
      end
      
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
