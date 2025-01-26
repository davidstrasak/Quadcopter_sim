function [outputVal] = saturate_value(inputVal, limit)
    if inputVal > limit
        outputVal = limit;
    elseif inputVal < -limit
        outputVal = -limit;
    else
        outputVal = inputVal;
    end
end