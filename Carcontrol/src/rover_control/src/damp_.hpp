#include <iostream>
class SecondOrderDynamics{
    private:
        double xp;
        double y,yd;
        double k1,k2,k3;
        double dt;
    public:
        SecondOrderDynamics(double f,double z,double r,double x0,double dt){
            double pi = 3.141592657;
            this->k1 = z/pi/f;
            this->k2 = 1/((2*pi*f)*(2*pi*f));
            this->k3 = r*z/(2*pi*f);
            this->dt = dt;
            this->xp = x0;
            this->y = x0;
            this->yd = 0;
        }
        
        double Update(double x){
            double xd = (x-xp)/dt;
            y = y + yd*dt;
            yd = yd + dt*(x+k3*xd-y-k1*yd)/k2;
            return y;
        }

        double get_y(){
            return y;
        }
        double get_yd(){
            return yd;
        }
        ~SecondOrderDynamics(){
            std::cout<<"free system"<<std::endl;
        }
};