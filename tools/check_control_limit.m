function [val] = check_control_limit(current_val,lower_limit, upper_limit)
val = max(min(current_val, upper_limit), lower_limit);
end

