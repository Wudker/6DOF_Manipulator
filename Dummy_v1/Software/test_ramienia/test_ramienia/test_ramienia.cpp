#include <iostream>
const float PI = 3.1415;
void calculateAngles(float vectorLength, float armLength, float x, float z, float& alphaAngle, float& thetaAngle) {
    float initialAngle = (x == 0) ? 90 : atan2(z, x) * (180.0 / PI);

    thetaAngle = asin((vectorLength / 2) / armLength) * 2 * (180.0 / PI);
    alphaAngle = acos((vectorLength / 2) / armLength) * (180.0 / PI) + initialAngle;

    if (x == 0 && z != 0) {
        alphaAngle = 90;
    }
    else if (x != 0 && z == 0) {
        alphaAngle = 0;
    }
}
float calculateKappaAngle(float x, float y) {
    float angle = 0;
    if (x == 0 && y == 0) {
        angle = 0;
    }
    else if (x == 0) {
        angle = (y > 0) ? 90 : 270;
    }
    else if (y == 0) {
        angle = (x > 0) ? 0 : 180;
    }
    else {
        angle = atan2(y, x) * (180.0 / PI);
        if (x > 0 && y > 0) {
            angle = angle;
        }
        if (x < 0 && y > 0) {
            angle = 270 - angle;
        }
        if (x < 0 && y < 0) {
            angle = 180 + (180 + angle);
        }
        if (x > 0 && y < 0) {
            angle = 180 + (180 + angle);
        }
    }
    return angle;
}


int main()
{
    while (1 == 1) {
        float valueX, valueZ, valueY;
        std::cin >> valueX  >> valueY >> valueZ;
        float alphaAngle = 0, thetaAnle = 0;
        valueX = abs(valueX);
        valueZ = abs(valueZ);
        float valueXY = sqrt(valueX * valueX + valueY * valueY);
        float vectorLength = sqrt(valueXY * valueXY + valueZ * valueZ);
        calculateAngles(vectorLength, 15.00, valueXY, valueZ, alphaAngle, thetaAnle);
        float kappaAngle = calculateKappaAngle(valueX, valueY);
        float kappa_smooth = kappaAngle / 4;
        for (int i = 1; i <= 1000; i++) {
            float kappa = kappa_smooth * (i/250);
                std::cout << kappa << std::endl;
        }
    }
        return 0;
}
