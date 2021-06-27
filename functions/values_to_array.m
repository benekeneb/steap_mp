function [x_array, y_array] = values_to_array(values, steps)

    import gtsam.*
    import gpmp2.*
    
    x_array = zeros(200, 1);
    y_array = zeros(200, 1);
       
    i = 1;
    while i < steps
        try
            atPose2VectorValues(symbol('x', i), values).pose;
        catch
            break;
        end
       
        x_array(i) = atPose2VectorValues(symbol('x', i), values).pose.x();
        y_array(i) = atPose2VectorValues(symbol('x', i), values).pose.y();

        i = i+1;
    end
end
   