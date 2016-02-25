M = 200;

X = rand(M,2);

W = [1:M]./M;

for t = 1:200
    %Particle filter estimation
    for m=1:M
        seed = rand(1);
        X(m,:,t+1) = X(find(W>seed,1),:,t);
    end

    
end


figure(1); clf;
subplot(3,3,1)
plot(X(:,1,1),X(:,2,1),'ro')
axis([0 1 0 1]);
xlabel('t=1')
subplot(3,3,2)
plot(X(:,1,2),X(:,2,2),'ro')
axis([0 1 0 1]);
xlabel('t=2')
subplot(3,3,3)
plot(X(:,1,5),X(:,2,5),'ro')
axis([0 1 0 1]);
xlabel('t=5')
subplot(3,3,4)
plot(X(:,1,10),X(:,2,10),'ro')
axis([0 1 0 1]);
xlabel('t=10')
subplot(3,3,5)
plot(X(:,1,20),X(:,2,20),'ro')
axis([0 1 0 1]);
xlabel('t=20')
subplot(3,3,6)
plot(X(:,1,30),X(:,2,30),'ro')
axis([0 1 0 1]);
xlabel('t=30')
subplot(3,3,7)
plot(X(:,1,50),X(:,2,50),'ro')
axis([0 1 0 1]);
xlabel('t=50')
subplot(3,3,8)
plot(X(:,1,70),X(:,2,70),'ro')
axis([0 1 0 1]);
xlabel('t=70')
subplot(3,3,9)
plot(X(:,1,200),X(:,2,200),'ro')
axis([0 1 0 1]);
xlabel('t=200')
