#include <iostream>
#include <cmath>
#include <vector>
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
double dF = 0.1;

//dynamic variables
double angle_X = 0;
double angle_Y = 0;
double omega_X = 0;
double omega_Y = 0;
double alpha_X = 0;
double alpha_Y = 0;
double pos_X, pos_Y, pos_Z = 0;
double vel_X, vel_Y, vel_Z = 0;
double acc_X, acc_Y, acc_Z = 0;

//: F1-F3 x axis. F2-F4 y axis
double F1_base = weight/4;
double F2_base = weight/4;
double F3_base = weight/4;
double F4_base = weight/4;
double F1 = 0;
double F2 = 0;
double F3 = 0;
double F4 = 0;
double Ftot_X, Ftot_Y, Ftot_Z = 0;

//output history
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
    F1 = F1_base + floor(angle_X)*dF + dF* omega_X/10;
    F3 = F3_base - floor(angle_X)*dF - dF* omega_X/10;
    F2 = F2_base + floor(angle_Y)*dF + dF* omega_Y/10;
    F2 = F4_base - floor(angle_Y)*dF - dF* omega_Y/10;
}

double changeForcesNormal(){
    F1 = F1_base + floor(angle_X)*dF ;
    F3 = F3_base - floor(angle_X)*dF ;
    F2 = F2_base + floor(angle_Y)*dF ;
    F4 = F4_base - floor(angle_Y)*dF ;
}

double changeForcesWeight(){
    double deltaW = ((-Ftot_Z)/4) + 4*dF;
    F1 = F1_base + floor(angle_X)*dF + deltaW ;
    F3 = F3_base - floor(angle_X)*dF + deltaW;
    F2 = F2_base + floor(angle_Y)*dF + deltaW;
    F4 = F4_base - floor(angle_Y)*dF + deltaW;
}

double changeForces(){
    changeForcesNormal();
    //changeForcesOmega();
    //changeForcesWeight();
    F1_hist.push_back(F1);
    F2_hist.push_back(F2);
    F3_hist.push_back(F3);
    F4_hist.push_back(F4);

}

double calculateRotationalDynamic(){
    double m_X = (F3 - F1) * L_arm;
    double m_Y = (F4 - F2) * L_arm;
    alpha_X = m_X / I_X;
    alpha_Y = m_Y / I_Y;
    alpha_X_history.push_back(alpha_X);
    alpha_Y_history.push_back(alpha_Y);

    omega_X += alpha_X * dt;
    omega_Y += alpha_Y * dt;
    omega_X_history.push_back(omega_X);
    omega_Y_history.push_back(omega_Y);

    angle_X += omega_X*dt + 0.5 * alpha_X * dt * dt;
    angle_Y += omega_Y*dt + 0.5 * alpha_Y * dt * dt;
    angle_X_history.push_back(angle_X);
    angle_Y_history.push_back(angle_Y);

    cout << angle_X  << "\t\t" << angle_Y <<endl;
}

double calculateCentreOfMassDynamic(){
    acc_X = Ftot_X / m;
    acc_Y = Ftot_Y / m;
    acc_Z = Ftot_Z / m;
    vel_X += acc_X * dt;
    vel_Y += acc_Y * dt;
    vel_Z += acc_Z * dt;
    pos_X+= vel_X * dt + 0.5 * acc_X * dt * dt;
    pos_Y+= vel_Y * dt + 0.5 * acc_Y * dt * dt;
    pos_Z+= vel_Z * dt + 0.5 * acc_Z * dt * dt;
    pos_X_hist.push_back(pos_X);
    pos_Y_hist.push_back(pos_Y);
    pos_Z_hist.push_back(pos_Z);
    vel_X_hist.push_back(vel_X);
    vel_Y_hist.push_back(vel_Y);
    vel_Z_hist.push_back(vel_Z);
    acc_X_hist.push_back(acc_X);
    acc_Y_hist.push_back(acc_Y);
    acc_Z_hist.push_back(acc_Z);
}

double calculateTotalForce() {
    Ftot_X = (F1 + F2 + F3 + F4) * sin(angle_X * PI / 180);
    Ftot_Y = (F1 + F2 + F3 + F4) * sin(angle_Y * PI / 180);
    Ftot_Z = (F1 + F2 + F3 + F4) * cos(angle_X * PI / 180) * cos(angle_Y * PI / 180) - weight;
    Ftot_X_hist.push_back(Ftot_X);
    Ftot_Y_hist.push_back(Ftot_Y);
    Ftot_Z_hist.push_back(Ftot_Z);
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
        dF = atof(argv[5]);
        dt = atof(argv[6]);
        seconds = atoi(argv[7]);
    }

    cout << "Starting process..." << endl;
    double time = 0;
    cout << "angle_X\t\tangle_Y" <<endl;
    while (time < seconds){
        changeForces();
        calculateTotalForce();
        calculateCentreOfMassDynamic();
        calculateRotationalDynamic();
        time += dt;
    }
    cout << "\nWriting output..." << endl;
    ofstream out_angleX("angleX");
    ofstream out_angleY("angleY");
    ofstream out_omegaX("omegaX");
    ofstream out_omegaY("omegaY");
    ofstream out_F1("F1");
    ofstream out_F2("F2");
    ofstream out_F3("F3");
    ofstream out_F4("F4");
    ofstream out_ftotX("FtotX");
    ofstream out_ftotY("FtotY");
    ofstream out_ftotZ("FtotZ");
    ofstream out_posX("posX");
    ofstream out_posY("posY");
    ofstream out_posZ("posZ");
    ofstream out_velX("velX");
    ofstream out_velY("velY");
    ofstream out_velZ("velZ");
    ofstream out_accX("accX");
    ofstream out_accY("accY");
    ofstream out_accZ("accZ");

    for (int i = 0; i< angle_Y_history.size(); i++ ){
        out_angleX << angle_X_history[i] << endl;
        out_angleY << angle_Y_history[i] << endl;
        out_omegaX << omega_X_history[i] << endl;
        out_omegaY << omega_Y_history[i] << endl;
        out_F1 << F1_hist[i] <<endl;
        out_F2 << F2_hist[i] << endl;
        out_F3 << F3_hist[i] <<endl;
        out_F4 << F4_hist[i] << endl;
        out_ftotX << Ftot_X_hist[i] << endl;
        out_ftotY << Ftot_Y_hist[i] << endl;
        out_ftotZ << Ftot_Z_hist[i] << endl;
        out_posX << pos_X_hist[i] << endl;
        out_posY << pos_Y_hist[i] << endl;
        out_posZ << pos_Z_hist[i] << endl;
        out_velX << vel_X_hist[i] << endl;
        out_velY << vel_Y_hist[i] << endl;
        out_velZ << vel_Z_hist[i] << endl;
        out_accX << acc_X_hist[i] << endl;
        out_accY << acc_Y_hist[i] << endl;
        out_accZ << acc_Z_hist[i] << endl;

    }
    out_angleX.close();
    out_angleY.close();
    out_omegaX.close();
    out_omegaY.close();
    out_ftotX.close();
    out_ftotY.close();
    out_ftotZ.close();
    out_F1.close();
    out_F2.close();
    out_F3.close();
    out_F4.close();
    out_posX.close();
    out_posY.close();
    out_posZ.close();
    out_velX.close();
    out_velY.close();
    out_velZ.close();
    out_accX.close();
    out_accY.close();
    out_accZ.close();
}



