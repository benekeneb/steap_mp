function [x_array, y_array] = values_to_array(values, steps)

    import gtsam.*
    import gpmp2.*
    
    x_array = zeros(200, 1);
    y_array = zeros(200, 1);
       
    i = 1;
    while i < steps
        try
            values.atPose2(symbol('x', i));
        catch
            break;
        end
       
        x_array(i) = values.atPose2(symbol('x', i)).x();
        y_array(i) = values.atPose2(symbol('x', i)).y();

        i = i+1;
    end
end
   