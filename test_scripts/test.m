x = optimvar('x', 2)
func = @(x) x(1)^2/2 + x(2)^2 - x(1)*x(2) - 2*x(1) - 6*x(2)
objec = fcn2optimexpr(func, x)

% x = optimvar('x', 2)
% objec = x(1)^2/2 + x(2)^2 - x(1)*x(2) - 2*x(1) - 6*x(2)

prob = optimproblem('Objective',objec)
% 
options = optimoptions('quadprog')
% 
problem = prob2struct(prob)
problem.solver = 'quadprog'
problem.options = options
% 
% problem.lb = [0, 0]
% problem.ub = [0, 0]
% 
% 
% 
quadprog(problem)