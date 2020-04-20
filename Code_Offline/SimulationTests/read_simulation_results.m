function [simulation_results] = read_simulation_results(name_str)

mat_file = load(name_str );
simulation_results = mat_file.MatOut;

end

