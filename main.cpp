#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <fstream>
using namespace std;


#define PI 3.14159265

//static parameters
double m = 4;
double weight = m*9.81;
double L_arm = 0.62;
double I_X = 0.06;
double I_Y = 0.06;
double dt = 0.1;
//350 is the default number of step
double Fstep = weight / (4*350);
double dF = 0;

//dynamic variables
double angle_X = 0;
double angle_Y = 0;
double omega_X = 0;
double omega_Y = 0;
double alpha_X = 0;
double alpha_Y = 0;
double pos_X = 0;
double pos_Y = 0;
double pos_Z = 0;
double vel_X = 0;
double vel_Y = 0;
double vel_Z = 0;
double acc_X = 0;
double acc_Y = 0;
double acc_Z = 0;

//: F1-F3 x axis. F2-F4 y axis
int F1_base = 350;
int F2_base = 350;
int F3_base = 350;
int F4_base = 350;
double F1 = 0;
double F2 = 0;
double F3 = 0;
double F4 = 0;
double Ftot_X = 0;
double Ftot_Y = 0;
double Ftot_Z = 0;

//output history
vector<map<string, double>> data;
map<string, double> * curr_data;
vector<double> angle_X_history ;
vector<double> angle_Y_history ;
vector<double> omega_X_history ;
vector<double> omega_Y_history ;
vector<double> alpha_X_history ;
vector<double> alpha_Y_history ;
vector<double> pos_X_hist;
vector<double> pos_Y_hist;
vector<double> pos_Z_hist;
vector<double> vel_X_hist;
vector<double> vel_Y_hist;
vector<double> vel_Z_hist;
vector<double> acc_X_hist;
vector<double> acc_Y_hist;
vector<double> acc_Z_hist;
vector<double> Ftot_X_hist;
vector<double> Ftot_Y_hist;
vector<double> Ftot_Z_hist;
vector<double> F1_hist;
vector<double> F2_hist;
vector<double> F3_hist;
vector<double> F4_hist;


double changeForcesOmega(){
    F1 = F1_base + floor(angle_X)*dF + dF* floor(omega_X/10) +1;
    F3 = F3_base - floor(angle_X)*dF - dF* floor(omega_X/10) +1 ;
    F2 = F2_base + floor(angle_Y)*dF + dF* floor(omega_Y/10) +1 ;
    F2 = F4_base - floor(angle_Y)*dF - dF* floor(omega_Y/10) +1 ;
}

double changeForcesNormal(){
    F1 = F1_base + floor(angle_X)*dF ;
    F3 = F3_base - floor(angle_X)*dF ;
    F2 = F2_base + floor(angle_Y)*dF ;
    F4 = F4_base - floor(angle_Y)*dF ;
}

double changeForcesWeight(){
    double deltaW = ((-Ftot_Z)/4) ;
    F1 = F1_base + floor(angle_X)*dF + deltaW ;
    F3 = F3_base - floor(angle_X)*dF + deltaW;
    F2 = F2_base + floor(angle_Y)*dF + deltaW;
    F4 = F4_base - floor(angle_Y)*dF + deltaW;
}

double changeForces(){
   // changeForcesNormal();
    //changeForcesOmega();
    changeForcesWeight();
    //saving real forces values
    (*curr_data)["F1"] = F1*Fstep;
    (*curr_data)["F2"] = F2*Fstep;
    (*curr_data)["F3"] = F3*Fstep;
    (*curr_data)["F4"] = F4*Fstep;
}

double calculateRotationalDynamic(){
    double m_X = (F3 - F1)* Fstep * L_arm;
    double m_Y = (F4 - F2)* Fstep * L_arm;
    alpha_X = m_X / I_X;
    alpha_Y = m_Y / I_Y;
    (*curr_data)["alphaX"] = alpha_X;
    (*curr_data)["alphaY"] = alpha_Y;

    omega_X += alpha_X * dt;
    omega_Y += alpha_Y * dt;
    (*curr_data)["omegaX"] = omega_X;
    (*curr_data)["omegaY"] = omega_Y;


    angle_X += omega_X*dt + 0.5 * alpha_X * dt * dt;
    angle_Y += omega_Y*dt + 0.5 * alpha_Y * dt * dt;
    (*curr_data)["angleX"] = angle_X;
    (*curr_data)["angleY"] = angle_Y;

}

double calculateCentreOfMassDynamic() {
    acc_X = Ftot_X / m;
    acc_Y = Ftot_Y / m;
    acc_Z = Ftot_Z / m;
    vel_X += acc_X * dt;
    vel_Y += acc_Y * dt;
    vel_Z += acc_Z * dt;
    pos_X += vel_X * dt + 0.5 * acc_X * dt * dt;
    pos_Y += vel_Y * dt + 0.5 * acc_Y * dt * dt;
    pos_Z += vel_Z * dt + 0.5 * acc_Z * dt * dt;
    (*curr_data)["posX"] = pos_X;
    (*curr_data)["posY"] = pos_Y;
    (*curr_data)["posZ"] = pos_Z;
    (*curr_data)["velX"] = vel_X;
    (*curr_data)["velY"] = vel_Y;
    (*curr_data)["velZ"] = vel_Z;
    (*curr_data)["accX"] = acc_X;
    (*curr_data)["accY"] = acc_Y;
    (*curr_data)["accZ"] = acc_Z;
}


double calculateTotalForce() {
    Ftot_X = (F1 + F2 + F3 + F4) * Fstep * sin(angle_X * PI / 180);
    Ftot_Y = (F1 + F2 + F3 + F4) * Fstep * sin(angle_Y * PI / 180);
    Ftot_Z = (F1 + F2 + F3 + F4) * Fstep * cos(angle_X * PI / 180) * cos(angle_Y * PI / 180) - weight;
    (*curr_data)["FtotX"] = Ftot_X;
    (*curr_data)["FtotY"] = Ftot_Y;
    (*curr_data)["FtotZ"] = Ftot_Z;
}


int main(int argc, char* argv[]){
    int seconds = 0;
    if (argc != 8){
        cout << "Insert: angle_X angle_Y omega_X omega_Y dF dT seconds" <<endl;
        return 1;
    }else{
        angle_X = atof(argv[1]);
        angle_Y = atof(argv[2]);
        omega_X = atof(argv[3]);
        omega_Y = atof(argv[4]);
        dF = atoi(argv[5]);
        dt = atof(argv[6]);
        seconds = atoi(argv[7]);
    }

    cout << "Starting process..." << endl;
    double time = 0;
    cout << "angle_X\t\tangle_Y" <<endl;
    while (time < seconds){
        curr_data = new map<string, double>();
        (*curr_data)["t"] = time;
        changeForces();
        calculateTotalForce();
        calculateCentreOfMassDynamic();
        calculateRotationalDynamic();
        time += dt;
        data.push_back(*curr_data);
    }
    cout << "\nWriting output..." << endl;
    ofstream out("data");
    for (map<string, double>::iterator it = curr_data->begin(); it!=curr_data->end(); it++){
        out << it->first << " ";
    }
    out << endl;
    for(vector<map<string, double>>::iterator vt = data.begin(); vt!=data.end(); vt++) {
        for (map<string, double>::iterator it = vt->begin(); it!=vt->end(); it++){
            out << it->second<< " ";
        }
        out << endl;
    }
    out.close();
}



