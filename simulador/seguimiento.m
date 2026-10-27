function out = seguimiento(in,P)
  
  NN = 0;
  flag      = in(1+NN);
  Va_d      = in(2+NN);
  r_path    = [in(3+NN); in(4+NN); in(5+NN)];
  q_path    = [in(6+NN); in(7+NN); in(8+NN)];
  c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
  rho_orbit = in(12+NN);
  lam_orbit = in(13+NN);
  NN = NN + 13;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  phi       = in(7+NN);
  theta     = in(8+NN);
  chi       = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
   r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  NN = NN + 16;
  t         = in(1+NN);
  
  switch flag,
      case 1, % seguimiento de linea recta dada r and q
          h_d=-r_path(3)-sqrt((r_path(1)-pn)^2+(r_path(2)-pe)^2)*q_path(3)/sqrt(q_path(1)^2+q_path(2)^2);
          chi_q =atan2(q_path(2),q_path(1));
          while chi_q-chi < -pi,
              chi_q=chi_q+2*pi;
          end
          while chi_q-chi > pi,
              chi_q=chi_q-2*pi;
          end
            
          h_c = h_d;
          e_py=-sin(chi_q)*(pn-r_path(1))+cos(chi_q)*(pe-r_path(2));
          k_path=0.025;
          chi_infty=89*pi/180;
          chi_c=chi_q-chi_infty*2*atan(k_path*e_py)/pi;
          %phi_ff = 0;
          Va_c = Va_d;
          error=e_py; 
      case 2, % seguimiento de orbita dada c, rho, lam
          d=sqrt((pn-c_orbit(1))^2+(pe-c_orbit(2))^2);
          phi_path=atan2(pe-c_orbit(2),pn-c_orbit(1));
          while phi_path-chi>pi,
              phi_path=phi_path-2*pi;
          end
          while phi_path-chi<-pi
              phi_path=phi_path+2*pi;
          end
          e_orbit=(d-rho_orbit)/rho_orbit;
          k_orbit=2;

           chi_c = phi_path+lam_orbit*[(pi/2)+atan(k_orbit*e_orbit)];
          h_c = -c_orbit(3);
          Va_c = Va_d;
          %error=d-rho_orbit;
  end
  
  % out
  out = [Va_c; h_c; chi_c];
end