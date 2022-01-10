function con_comm = fuzz(th,th_d,th_dd,Irot)
%fuzz provides fuzzy logic control for a cubesat given a single dimensional
%control axis.
%   Detailed explanation goes here
Vcc=12;
rate_final=1/(6000000*Irot);
rate_inner=1/(5000000*Irot);
rate_outer=1/(200000*Irot);
th_thresh=0.1;
th_thresh_outer=0.6;
th_thresh_inner=0.2;
if (th<th_thresh)&&(th>-th_thresh)
    theta=abs(th);
else
    theta=1;
end

if th>=th_thresh_outer
    if th_d>=-rate_outer
        con_comm=-Vcc;
    else
        if th_dd>0
            con_comm=-Vcc*theta;
        else
            con_comm=Vcc*theta;
        end
    end
elseif th<=-th_thresh_outer
    if th_d<=rate_outer
        con_comm=Vcc;
    else
        if th_dd<0
            con_comm=Vcc*theta;
        else
            con_comm=-Vcc*theta;
        end
    end
elseif th>=th_thresh_inner
    if th_d>=-rate_inner
        con_comm=-Vcc;
    else
        if th_dd>0
            con_comm=-Vcc*theta;
        else
            con_comm=Vcc*theta;
        end
    end
elseif th<=-th_thresh_inner
    if th_d<=rate_inner
        con_comm=Vcc;
    else
        if th_dd<0
            con_comm=Vcc*theta;
        else
            con_comm=-Vcc*theta;
        end
    end
elseif th>=th_thresh
    if th_d>=-rate_final
        con_comm=-Vcc;
    else
        if th_dd>0
            con_comm=-Vcc*theta;
        else
            con_comm=Vcc*theta;
        end
    end
elseif th<=-th_thresh
    if th_d<=rate_final
        con_comm=Vcc;
    else
        if th_dd<0
            con_comm=Vcc*theta;
        else
            con_comm=-Vcc*theta;
        end
    end
elseif (th<th_thresh)&&(th>=0)
    if th_d>=0
        con_comm=-Vcc*theta;
    else
        if th_dd>0
            con_comm=-Vcc*theta;
        else
            con_comm=Vcc*theta;
        end
    end
elseif (th<=0)&&(th>-th_thresh)
    if th_d<=0
        con_comm=Vcc;
    else
        if th_dd<0
            con_comm=Vcc*theta;
        else
            con_comm=-Vcc*theta;
        end
    end
end

if con_comm>Vcc
    con_comm=Vcc;
elseif con_comm<-Vcc
    con_comm=-Vcc;
end
end

