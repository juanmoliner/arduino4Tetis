#include <cstdio>


int main(int argc, char **argv)
{
    double p[3];
    
    double E2=52.55;
    double E3=320;
    double E4=225;
    double E5=167.25;
    double M3=55.775;
    double  M4=55.775;
    double M5=57;
    
    double t1 = input[0];
    double t2 = input[1];
    double t3 = input[2];
    double t4 = input[4];
    
    double c1 = cos(t1);
    double s1 = sin(t1);
    
    double c2 = cos(t2);
    double s2 = sin(t2);
    
    double c3 = cos(t3);
    double s3 = sin(t3);
    
    double c4 = cos(t4);
    double s4 = sin(t4);
    
    double c23 = cos(t2+t3);
    double s23 = sin(t2+t3);
    
    double c34 = cos(t3+t4);
    double s34 = sin(t3+t4);
    
    double c234 = cos(t2+t3+t4);
    double s234 = sin(t2+t3+t4);
    

    p[0] = -M5*s1+E4*c23*c1+E3*c1*c2+E5*c234*c1;
    p[1] =  M5*c1+E4*c23*s1+E3*c2*s1+E5*c234*s1;
    p[2] =  E4*s23+E3*s2+E5*s234;
    
   
    Je[0][0] = -M5*c234;
    Je[0][1] = E3*s34+E4*s4;
    Je[0][2] = E4*s4;
    Je[0][3] = 0.0;
    Je[1][0] = E4*c23+E3*c2+E5*c234;
    Je[1][1] = 0.0;
    Je[1][2] = 0.0;
    Je[1][3] = 0.0;
    Je[2][0] = M5*s23;
    Je[2][1] = E5+E3*c34+E4*c4;
    Je[2][2] = E5+E4*c4;
    Je[2][3] = E5;
    Je[3][0] = 0.0;
    Je[3][1] = 1.0;
    Je[3][2] = 1.0;
    Je[3][3] = 1.0;
    
    
    Jei=inverse(Je);
    
    output[0]= Jei[0][0] * inputp[0] + Jei[0][1] * inputp[1] + Jei[0][2] * inputp[2]; //+ Jei[0][3] * inputp[3];
    output[1]= Jei[1][0] * inputp[0] + Jei[1][1] * inputp[1] + Jei[1][2] * inputp[2]; //+ Jei[1][3] * inputp[3];
    output[2]= Jei[2][0] * inputp[0] + Jei[2][1] * inputp[1] + Jei[2][2] * inputp[2]; //+ Jei[2][3] * inputp[3];
    output[3]= Jei[3][0] * inputp[0] + Jei[3][1] * inputp[1] + Jei[3][2] * inputp[2]; //+ Jei[3][3] * inputp[3];
    

    
  return 0;
}
