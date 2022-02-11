function delta = MPC_Sim(vx,PsiDot,Beta,PsiDotRef,controller)

    %% Paramaters
    m = 1400;
    Iz = 1960;
    lf = 1.1770;
    lr = 1.3580;
    Cf = 84085;
    Cr = 87342;
    g = 9.8100;
    
    Ts = 0.01;
    %% Recomputing Ad & Bd for each vx

    A = [-(lf^2*Cf+lr^2*Cr)/(Iz*vx) (-lf*Cf+lr*Cr)/(Iz);
         -1+(-lf*Cf+lr*Cr)/(m*vx^2) -(Cf+Cr)/(m*vx)];
    B = [lf*Cf/Iz; Cf/(m*vx)];

    %Exact discretization
    sysd = c2d(ss(A,B,eye(2),0),Ts);
    Ad = sysd.A;  
    Bd = sysd.B; 
    
    %%
    x0 = [PsiDot;Beta];
    u = controller(x0,PsiDotRef,Ad,Bd);
    
    delta = u(1);

end


