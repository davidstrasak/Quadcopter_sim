function [outputVal] = saturate_value(inputVal, limit)
    if inputVal > limit
        outputVal = limit;
        disp('saturated')
    elseif inputVal < -limit
        outputVal = -limit;
        disp('saturated')
    else
        outputVal = inputVal;
    end
end