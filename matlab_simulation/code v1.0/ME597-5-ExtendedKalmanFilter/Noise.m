function N=Noise(n,e)

 N=randn(n,1);
 N(3)=toRad(N(n));
 N=e*N;
end