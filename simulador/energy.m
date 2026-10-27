function [energyConsummed] = energy(uu,P)

delta_e = uu(1);
delta_a = uu(2);
delta_r = uu(3);
delta_t = uu(4);
t       = uu(4)
energyConsummed = 20*delta_t*t;
end