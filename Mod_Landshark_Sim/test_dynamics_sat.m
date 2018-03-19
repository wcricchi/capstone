u=1;
x(1,1)=0;
x(2,1)=0

for i=1:100
   x(:,i+1)=A*x(:,i)+B*u;
end

figure(1)
hold on
plot(x(1,:),'r-')
hold off