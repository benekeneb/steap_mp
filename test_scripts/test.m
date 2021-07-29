% fun = @(x)100*(x(2)-x(1)^2)^2 + (1-x(1))^2;
% 
% x0 = [-1,2];
% A = [1,2];
% b = 1;
% x = fmincon(fun,x0,A,b)

A = eye(3)
b = [1 1 0]
% 
% A_const = [];
% b_const = [];
% Aeq_const = [];
% beq_const = [];

% function C = mycost(x)
% % created matrices A(150x50) & B(150x150)
% % C = norm(0.5*(eye(3) * x - [1 1 0]))^2
% C = 0.5
% end

fun = @(x) 0.5* norm(A * transpose([x(1) x(2) x(3)]) - transpose(b))^2;

x0 = [2, 10, 0];
A = [];
b = [];

Aeq = [];
beq = [];

lb = [10, -inf, 10];
ub = [10, inf, 10];
x = fmincon(fun,x0,A,b ,Aeq,beq,lb,ub)