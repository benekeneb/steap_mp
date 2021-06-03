euler_val = 1.57079632679

euler_vector = zeros(1, 3);
euler_vector(1) = euler_val;
quaternion_vector = eul2quat(euler_vector);
orient_w = quaternion_vector(4);