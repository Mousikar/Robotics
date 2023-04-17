syms phi_1 phi_2 phi_3
Q=rotz(phi_1)*roty(phi_2)*rotz(phi_3)
Q=simplify(Q)
%%
p_0=[0 0 1].';
p=Q*p_0
%%
syms d_phi_1 d_phi_2 d_phi_3
% dp=diff(p,'phi_1')*d_phi_1+diff(p,'phi_2')*d_phi_2
dQ=diff(Q,'phi_1')*d_phi_1+diff(Q,'phi_2')*d_phi_2+diff(Q,'phi_3')*d_phi_3
%%
omega_s=dQ*(Q.')
omega_s=simplify(omega_s)
omega_x=omega_s(3,2)
omega_y=omega_s(1,3)
omega_z=omega_s(2,1)