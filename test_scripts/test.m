        figure;
        hold on;
        minimum = 10000;
        x_min = 0;
        y_min = 0;
        for x = 0:20
           for y = 0:20
               x_adj = x/2;
               y_adj = y/2;

               x_vector = [x_adj y_adj 0 0 0];

               m_gp1_x0_likelihood = m_gp1_x0(x_vector);
               m_gp0_x0_likelihood = m_gp0_x0(x_vector);
               m_o0_to_x0_likelihood = m_o0_to_x0(x_vector);
               m_prior_x1_likelihood = m_prior_x1(x_vector);

               plot3(x_adj, y_adj, m_gp1_x0_likelihood, '. r');
               plot3(x_adj, y_adj, m_gp0_x0_likelihood, '. g');
               plot3(x_adj, y_adj, m_o0_to_x0_likelihood, '. b');
               plot3(x_adj, y_adj, m_prior_x1_likelihood, '. y');
           end
        end
