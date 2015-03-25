function [desc, depot_desc, cust, depot] = parseProblemSet(filename)
    % Summary:  Function for parsing Cordeau problem sets and returning 
    %           arrays containing the data.
    % 
    % desc:         Prob descr (type, # vehicles, # cust, # dep)
    % depot_desc:   Descr of depots (max dur of route, max vehicle load)
    % cust:         Customers (id, x,y, serv dur, demand, visit freq, 
    %                           # visit combs, list visit combs)
    % depot:        Depots (same as customers, but list is not present)
    
    f = fopen(filename); % Open for read
    
    % Description of problem set
    desc = str2double(strsplit(strtrim(fgets(f)), ' '));
    if length(desc) ~= 4
        error('File not correct format')
    end
    
    % Pull out variables
    desc_cell = num2cell(desc);
    [ptype m n t] = desc_cell{:};
    
    % Depot descriptions
    depot_desc = zeros(t,2);
    for i = 1:t
        depot_desc(i,:) = str2double(strsplit(strtrim(fgets(f)), ' '));
    end
    
    % Customers
    first_cust = str2double(strsplit(strtrim(fgets(f)), ' '));
    cust = zeros(n,length(first_cust));
    cust(1,:) = first_cust;
    for i = 2:n
        cust(i,:) = str2double(strsplit(strtrim(fgets(f)), ' '));
    end
    
    % Depots
    first_depot = str2double(strsplit(strtrim(fgets(f)), ' '));
    depot = zeros(t,length(first_depot));
    depot(1,:) = first_depot;
    for i = 2:t
        depot(i,:) = str2double(strsplit(strtrim(fgets(f)), ' '));
    end
    
    fclose(f); % Close it up
    return
end