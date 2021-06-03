x = 1:0.01:25;
y = sin(x);
n = numel(x);
figure
hold on

xlim([0 25])
ylim([-1.1 1.1])

plot(1,1, 'O r')
pause(5)
plot(1,-1, 'O')