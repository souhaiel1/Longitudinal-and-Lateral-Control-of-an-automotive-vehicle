function w = MappingFtoW(Fx,Vx)

C_f = 66100;
R = 0.3;

if Fx>=0
    w = Vx/(R*(1-Fx/C_f));
else
    w =Vx*(Fx/C_f+1)/R;
end

end