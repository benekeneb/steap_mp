function [x_array, y_array] = values_to_array(values)

    import gtsam.*
    import gpmp2.*
    
    x_array = zeros(200, 1);
    y_array = zeros(200, 1);
       
    end_reached = 0;
    i = 1;
    while end_reached == 0
        try
            values.atPose2(symbol('x', i))
        catch
            end_reached = 1;
            break;
        end
       
        x_array(i) = values.atPose2(symbol('x', i)).x();
        y_array(i) = values.atPose2(symbol('x', i)).y();

        i = i+1;
    end
end
   