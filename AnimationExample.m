t = linspace(0, 2*pi, 100);
f = sin(t);
figure;
axis([0 2*pi -0.5 0.5]);
axis square;
grid on;

for i = 1:numel(t);
   cla(gca);
   hold on
   plot(t(1:i), f(1:i));
   hold off
   drawnow;
   pause(1/60);
end