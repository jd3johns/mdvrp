function [cost, routes] = parseSolutionSet(filename)
    % Summary:  Function for parsing Cordeau solution sets and returning 
    %           arrays containing the data.
    % 
    % cost:     Total solution cost (excluding service time)
    % routes:    (depot, vehicle, route len, load, seq of stops (not impl))
    
    f = fopen(filename); % Open for read
    
    % How long is the file?
    fseek(f, 0, 'eof');
    filesize = ftell(f); frewind(f);
    data = fread(f, filesize, 'uint8');
    numlines = sum(data == 10); frewind(f);
    
    % Cost of the solution
    cost = str2double(strtrim(fgets(f)));
    if length(cost) ~= 1
        error('File not correct format')
    end
    
    % Vehicle routes
    routes = zeros(numlines - 1,4);
    for i = 1:numlines - 1
        route = str2double(strsplit(strtrim(fgets(f)), ' '));
        routes(i,:) = route(:,1:4);
    end
    
    fclose(f); % Close it up
    return
end