classdef WaterDrop %< handle % inherit from handle to allow pass by ref
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
      initcapacity = 0;
      
      epsilon = 0.001;
      epsilon_s = 0.01;
   end
   
   methods
   % public methods
   
      function obj = WaterDrop(depot, capacity, velocity, ...
                               vel_params, ...
                               soil_params)
      % class constructor
         obj.route = [obj.route; depot];
         obj.capacity = capacity;
         obj.initcapacity = capacity;
         obj.vel = velocity;
         
         % velocity params
         obj.a_v = vel_params(1);
         obj.b_v = vel_params(2);
         obj.c_v = vel_params(3);
         
         % soil params
         obj.a_s = soil_params(1);
         obj.b_s = soil_params(2);
         obj.c_s = soil_params(3);
      end

      function obj = updateSoil(obj, dist_mat)
         obj.soil = obj.soil + obj.deltaSoil(dist_mat);
      end

      function obj = updateVelocity(obj, soil_mat)
         n_idx = length(obj.route);
         obj.vel = obj.vel + obj.a_v / (obj.b_v + obj.c_v * ...
             soil_mat(obj.route(n_idx - 1), obj.route(n_idx)));
      end
      
      function obj = flow(obj, soil_mat, customers, demand) % custs that haven't been visited
      % probabilistically move to a connected customer
         probs = obj.pathProbs(soil_mat, customers);
         path = obj.selectPath(probs);
         newCust = customers(path);

         % capacitation
         newCustDemand = demand((demand(:, 1) == newCust), 2);
         if (obj.capacity < newCustDemand)
             obj = obj.returnHome();
         else
             obj.capacity = obj.capacity-newCustDemand;
             obj.route = [obj.route; newCust];
         end
         
      end

      function soil = deltaSoil(obj, dist_mat)
      % soil picked up along most recent path
         soil = obj.a_s/(obj.b_s + obj.c_s * obj.calcTime(dist_mat));
      end

      function cost = calcRouteCost(obj, dist_mat)
         cost = 0;
         for i = 2:length(obj.route)
            cost = cost + dist_mat(obj.route(i - 1), obj.route(i));
         end
      end
      
      function obj = returnHome(obj)
      % return to the depot
          if (obj.route(end) ~= obj.route(1))
            obj.route = [obj.route; obj.route(1)];
          end
          obj.capacity = obj.initcapacity;
      end
      
      function obj = reset(obj, velocity)
          obj.soil = 0;
          obj.capacity = obj.initcapacity;
          obj.vel = velocity;
          obj.route = [obj.route(1)];
      end
   end
   
   methods(Access = 'public')
   % helper methods
      
      function time = calcTime(obj, dist_mat)
      % time required to traverse an edge
         n_idx = length(obj.route);
         time = dist_mat(obj.route(n_idx - 1), obj.route(n_idx)) / ...
             max(obj.epsilon, obj.vel);
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
               return
            end
         end
      end
      
      function probs = pathProbs(obj, soil_mat, customers)
      % calculate the probabilities of traversing possible paths
         invsoils = zeros(length(customers),1);
         for i = 1:length(customers)
            invsoils(i) = obj.invSoil(customers(i), soil_mat, customers);
         end
         sumsoils = sum(invsoils);
         
         probs = zeros(length(customers),1);
         for i = 1:length(customers)
            f = invsoils(i);
            probs(i) = f/sumsoils;
         end
      end

      function f = invSoil(obj, customer, soil_mat, customers)
      % inverse term calculation
         f = 1/(obj.epsilon_s + obj.positiveSoil(customer, soil_mat, customers));
      end

      function g = positiveSoil(obj, customer, soil_mat, customers)
      % term calculation
         soils = zeros(length(customers),1);
         for i = 1:length(customers)
            soils(i) = soil_mat(obj.route(end),customers(i));
         end
         min_s = min(soils);
         
         c_idx = find(customers == customer);
         if min_s >= 0
            g = soils(c_idx);
         else
            g = soils(c_idx) - min_s;
         end
      end
   end
end
