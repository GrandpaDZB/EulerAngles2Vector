#include <iostream>
using namespace std;

class IsomorphismTransformer
{
private:
    /* data */
public:
    float IsomorphismMatrix[2][2], offset_vector[2];
    IsomorphismTransformer(float points_camera[6], float points_realworld[6]){
        float delta_x_c[2] = {  points_camera[0] - points_camera[4],
                                points_camera[2] - points_camera[4],}; 
        float delta_y_c[2] = {  points_camera[1] - points_camera[5],
                                points_camera[3] - points_camera[5],}; 
        float delta_x_r[2] = {  points_realworld[0] - points_realworld[4],
                                points_realworld[2] - points_realworld[4],}; 
        float delta_y_r[2] = {  points_realworld[1] - points_realworld[5],
                                points_realworld[3] - points_realworld[5],}; 
        this->IsomorphismMatrix[0][1] = (delta_x_c[0]*delta_x_r[1] - delta_x_c[1]*delta_x_r[0])/(delta_x_c[0]*delta_y_c[1] - delta_y_c[0]*delta_x_c[1]);
        this->IsomorphismMatrix[0][0] = (delta_x_r[1] - this->IsomorphismMatrix[0][1]*delta_y_c[1])/delta_x_c[1];
        this->IsomorphismMatrix[1][1] = (delta_x_c[0]*delta_y_r[1] - delta_x_c[1]*delta_y_r[0])/(delta_x_c[0]*delta_y_c[1] - delta_y_c[0]*delta_x_c[1]);
        this->IsomorphismMatrix[1][0] = (delta_y_r[1] - this->IsomorphismMatrix[1][1]*delta_y_c[1])/delta_x_c[1];

        this->offset_vector[0] = points_realworld[0] - this->IsomorphismMatrix[0][0]*points_camera[0] - this->IsomorphismMatrix[0][1]*points_camera[1];
        this->offset_vector[1] = points_realworld[1] - this->IsomorphismMatrix[1][0]*points_camera[0] - this->IsomorphismMatrix[1][1]*points_camera[1];
        return;
    };
    ~IsomorphismTransformer(){};
    void camera2realworld(float points_camera[2], float points_realworld[2]){
        points_realworld[0] = this->IsomorphismMatrix[0][0]*points_camera[0] + this->IsomorphismMatrix[0][1]*points_camera[1] + this->offset_vector[0];
        points_realworld[1] = this->IsomorphismMatrix[1][0]*points_camera[0] + this->IsomorphismMatrix[1][1]*points_camera[1] + this->offset_vector[1];
        return;
    }
    void show(){
        cout << "Isomorphism Matrix: " << endl;
        cout << this->IsomorphismMatrix[0][0] << " " << this->IsomorphismMatrix[0][1] << endl;
        cout << this->IsomorphismMatrix[1][0] << " " << this->IsomorphismMatrix[1][1] << endl;
        cout << "Offset Vector: " << endl;
        cout << this->offset_vector[0] << " " << this->offset_vector[1] << endl;
        return;
    }
};


int main(int argc, char** argv)
{
    float points_realworld[6] = {0,0,0,1,1,0};
    float points_camera[6] = {-1,0,-2,0,-1,1};
    IsomorphismTransformer IT(points_camera, points_realworld);
    IT.show();

    float v_c[2] = {0,0};
    float v_r[2] = {0,0};

    IT.camera2realworld(v_c, v_r);
    cout << v_r[0] << " " << v_r[1] << endl;

    return 0;
}