function [y] = RK8(func, t, y, h, params)
k_1 = func(t,y, params);
k_2 = func(t+h*(4/27),y+(h*4/27)*k_1, params);
k_3 = func(t+h*(2/9),y+(h/18)*(k_1+3*k_2), params);
k_4 = func(t+h*(1/3),y+(h/12)*(k_1+3*k_3), params);
k_5 = func(t+h*(1/2),y+(h/8)*(k_1+3*k_4), params);
k_6 = func(t+h*(2/3),y+(h/54)*(13*k_1-27*k_3+42*k_4+8*k_5), params);
k_7 = func(t+h*(1/6),y+(h/4320)*(389*k_1-54*k_3+966*k_4-824*k_5+243*k_6), params);
k_8 = func(t+h,y+(h/20)*(-234*k_1+81*k_3-1164*k_4+656*k_5-122*k_6+800*k_7), params);
k_9 = func(t+h*(5/6),y+(h/288)*(-127*k_1+18*k_3-678*k_4+456*k_5-9*k_6+576*k_7+4*k_8), params);
k_10= func(t+h,y+(h/820)*(1481*k_1-81*k_3+7104*k_4-3376*k_5+72*k_6-5040*k_7-60*k_8+720*k_9), params);
y = y + h/840*(41*k_1+27*k_4+272*k_5+27*k_6+216*k_7+216*k_9+41*k_10);
end
