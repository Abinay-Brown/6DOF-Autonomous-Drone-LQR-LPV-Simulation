function [Xr,Yr,Zr,Xdr,Ydr,Zdr,Xddr,Yddr,Zddr, psiInt] = refTraj2(t)
r=5;
f=0.05;
height_i=0.5;
height_f=30;
d_height=height_f-height_i;
alpha=2*pi*f*t;
r_2=r/500;
pwr=2;
Xr=(r_2*t.^pwr+r).*cos(alpha);
Yr=(r_2*t.^pwr+r).*sin(alpha);
Zr=height_i+d_height/t(end).*t;

Xdr=pwr.*r_2.*t.^(pwr-1).*cos(alpha)-(r_2.*t.^(pwr)+r).*sin(alpha)*2*pi.*f;
Ydr=pwr.*r_2.*t.^(pwr-1).*sin(alpha)+(r_2.*t.^(pwr)+r).*cos(alpha)*2*pi.*f;
Zdr=d_height./(t(end)).*ones(length(t));

Xddr=(pwr.*(pwr-1).*r_2.*t.^(pwr-2).*cos(alpha)-pwr.*r_2.*t.^(pwr-1).*sin(alpha).*2*pi.*f)-(pwr.*r_2.*t.^(pwr-1).*sin(alpha)*2*pi.*f+(r_2.*t.^pwr+r_2).*cos(alpha).*(2*pi*f).^2);
Yddr=(pwr.*(pwr-1).*r_2.*t.^(pwr-2).*sin(alpha)+pwr.*r_2.*t.^(pwr-1).*cos(alpha).*2*pi.*f)+(pwr.*r_2.*t.^(pwr-1).*cos(alpha)*2*pi.*f-(r_2.*t.^pwr+r_2).*sin(alpha).*(2*pi*f).^2);
Zddr=0*ones(length(t));

dx=Xr(2:length(Xr))-Xr(1:length(Xr)-1);
dy=Yr(2:length(Yr))-Yr(1:length(Yr)-1);
dz=Zr(2:length(Zr))-Zr(1:length(Zr)-1);

dx=[dx(1), dx];
dy=[dy(1), dy];
dz=[dz(1), dz];

psi=zeros(length(Xr),1);
psiInt = psi;
psi(1) = atan2(Yr(1),Xr(1))+pi/2;
psi(2:length(psi))=atan2(dy(2:length(dy)),dx(2 :length(dx)));


dpsi=psi(2:length(psi))-psi(1:length(psi)-1);
psiInt(1)=psi(1);
for i =2:length(psiInt)-1
    if dpsi(i-1)<-pi
        psiInt(i)=psiInt(i-1)+(dpsi(i-1)+2*pi);
    elseif dpsi(i-1)>pi
        psiInt(i)=psiInt(i-1)+(dpsi(i-1)-2*pi);
    else
        psiInt(i)=psiInt(i-1)+dpsi(i-1);
    end
end
end
