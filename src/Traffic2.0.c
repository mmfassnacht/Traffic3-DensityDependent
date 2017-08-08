#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 10
#define lambda 3.0
#define L .5
#define tmax 20
#define dt .1

double f1(double v[N], int i){return (v[i]);}
double f2(double x[N], double v[N], int i) {return (lambda*(v[i-1] - v[i])/(x[i-1] - x[i]));}

void rungeKutta(double t, double x[N], double v[N]) {

    FILE *fptrV;
    FILE *fptrX;
    fptrV = fopen("trafficV.csv","w");
    fptrX = fopen("trafficX.csv","w");

    double x1[N], x2[N], x3[N], xNew[N];
    double v1[N], v2[N], v3[N], vNew[N];
    int i;

    fprintf(fptrX,"t,position\n");
    fprintf(fptrV,"t,velocity\n");

    for(t = dt; t <= tmax; t+=dt)
    {
        printf("t: %lf\n",t);

        //Special case -- first car. Velocity is constant.
        v1[0] = v[0];
        v2[0] = v[0];
        v3[0] = v[0];
        x1[0] = x[0] + ((dt/2.0)*(f1(v,0)));
        x2[0] = x[0] + ((dt/2.0)*(f1(v1,0)));
        x3[0] = x[0] + (dt*(f1(v2,0)));
        xNew[0] = x[0] + (dt/6.0)*(f1(v,0)+2.0*f1(v1,0)+2.0*f1(v2,0)+f1(v3,0));
        x[0] = xNew[0];
        printf("Car 0 position = %lf\tCar 0 velocity = %lf\n",x[0],v[0]);
        fprintf(fptrX,"%lf,", t);
        fprintf(fptrV,"%lf,", t);
        fprintf(fptrX,"%lf\n", x[0]);
        fprintf(fptrV,"%lf\n", v[0]);

        for(i = 1; i < N; i++)
        {
            x1[i] = x[i] + ((dt/2.0)*(f1(v,i)));
            v1[i] = v[i] + ((dt/2.0)*(f2(x,v,i)));

            x2[i] = x[i] + ((dt/2.0)*(f1(v1,i)));
            v2[i] = v[i] + ((dt/2.0)*(f2(x1,v1,i)));

            x3[i] = x[i] + (dt*(f1(v2,i)));
            v3[i] = v[i] + (dt*(f2(x2,v2,i)));

            xNew[i] = x[i] + (dt/6.0)*(f1(v,i)+2.0*f1(v1,i)+2.0*f1(v2,i)+f1(v3,i));
            vNew[i] = v[i] + (dt/6.0)*((f2(x,v,i)+2.0*f2(x1,v1,i))+2.0*f2(x2,v2,i)+f2(x3,v3,i));

            x[i] = xNew[i];
            v[i] = vNew[i];

            if(x[i]>x[i-1])
            {
                printf("Crash!");
                return;
            }
            printf("Car %d position = %lf\tCar %d velocity = %lf\n",i,x[i],i,v[i]);
            fprintf(fptrX,"%lf,", t);
            fprintf(fptrV,"%lf,", t);
            fprintf(fptrX,"%lf\n", x[i]);
            fprintf(fptrV,"%lf\n", v[i]);
        }
    }
}

int main() {

    double x[N];
    double v[N];
    double t = 0.0;
    int i;

    ///Initial conditions
    //Car 1
    x[0] = 0;
    v[0] = 15;

    ///Zero out arrays
    for(i = 1; i < N; i++) {
        x[i] = x[i-1] - 1.2;
        v[i] = 20;
    }

    rungeKutta(t,x,v);

    return 0;
}
