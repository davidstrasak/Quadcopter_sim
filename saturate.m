function [outputVal] = saturate_value(inputVal, limit)
%Function that saturates the value of the rolls of the quadcopter, so the
%quadcopter is always in hover mode
    if inputVal > limit
        outputVal = limit;
    elseif inputVal < -limit
        outputVal = -limit;
    else
        outputVal = inputVal;
    end
end