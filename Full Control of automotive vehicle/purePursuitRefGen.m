function Ref = purePursuitRefGen(Xgl,Ygl,YawHead,Vx,Beta,tp,Track)
   
    Xpath = Track.X;
    Ypath = Track.Y;
    if abs(Vx)<0.05
        Vx = 0.05;
    end
    Vy = Vx*Beta; %Car body slip angle
    
    V = sqrt(Vx^2+Vy^2);
    L = tp*V; %Lookahead distance
    
    global pathindex
       
    % Find new Path Index
    pathindex = findMinDist(Xgl,Ygl,Xpath,Ypath,pathindex);
    
    % Find Pursuit Point
    XL = Xgl+L*cos(YawHead+Beta); %X position at lookahead distance if actual velocity is maintened
    YL = Ygl+L*sin(YawHead+Beta); %Y position at lookahead distance if actual velocity is maintened
    pursuitindex = findMinDist(XL,YL,Xpath,Ypath,pathindex);
    Xpursuit = Xpath(pursuitindex);
    Ypursuit = Ypath(pursuitindex);
    
    % Find yawrate       
    alpha = atan2(Ypursuit-Ygl,Xpursuit-Xgl)-YawHead; 
    k = 2*sin(alpha)/norm([Xgl,Ygl]-[Xpursuit,Ypursuit]);  %k = 2*sin(alpha)/L
    Yawrate_Ref = k*Vx;
    
    % Find ye
    sgn = sign(sin(alpha));
    ye = sgn*norm([XL,YL]-[Xpursuit,Ypursuit]);
    
    %Output References
    Ref = [ye;Yawrate_Ref];
end


function y = findMinDist(X,Y,Xpath,Ypath,index)
    DO = true;
    while DO
        di = norm([X,Y]-[Xpath(index),Ypath(index)]);
        index = index+1;
        dnew = norm([X,Y]-[Xpath(index),Ypath(index)]);
        DO = di>dnew;
    end
    y = index-1;
end

% function y = findPointL(L,X,Y,Xpath,Ypath,index)
%     DO = true;
%     while DO
%         di = norm([X,Y]-[Xpath(index),Ypath(index)]);
%         index = index+1;
%         DO = L>di;
%     end
%     y = index-1;
% end



