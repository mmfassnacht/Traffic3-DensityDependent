#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double f1(double v[], int i){return (v[i]);}
double f2(double lambda, double x[], double v[], int i) {return (lambda*(v[i-1] - v[i])/(x[i-1] - x[i]));}

void rungeKutta(double t, double x[], double v[], double N, double lambda, double tmax, double dt, double limit) {

    FILE *fptrV;
    FILE *fptrX;
    fptrV = fopen("trafficV.csv","w");
    fptrX = fopen("trafficX.csv","w");

    double *x1 = malloc(sizeof(double)*N), *x2 = malloc(sizeof(double)*N), *x3 = malloc(sizeof(double)*N), *xNew = malloc(sizeof(double)*N);
    double *v1 = malloc(sizeof(double)*N), *v2 = malloc(sizeof(double)*N), *v3 = malloc(sizeof(double)*N), *vNew = malloc(sizeof(double)*N);
    int i;

    fprintf(fptrX,"t,x,y\n");
    fprintf(fptrV,"t,velocity\n");

    for(t = dt; t <= tmax; t+=dt)
    {
        printf("t: %lf s\n",t);

        //Special case -- first car. Velocity increases linearly with time.
        v[0] = t*2.0;
        if(v[0] > limit)        //Imposing speed limit of 30m/s ~ 65mph
            v[0] = limit;
        v1[0] = v[0];
        v2[0] = v[0];
        v3[0] = v[0];

        x1[0] = x[0];
        x2[0] = x[0];
        x3[0] = x[0];

        //Updating position of leading car - to be used in next timestep.
        x1[0] = x[0] + ((dt/2.0)*(f1(v,0)));
        x2[0] = x[0] + ((dt/2.0)*(f1(v1,0)));
        x3[0] = x[0] + (dt*(f1(v2,0)));
        xNew[0] = x[0] + (dt/6.0)*(f1(v,0)+2.0*f1(v1,0)+2.0*f1(v2,0)+f1(v3,0));
        x[0] = xNew[0];

        printf("Car 0 position = %lf m\tCar 0 velocity = %lf m/s\n",x[0],v[0]);
        fprintf(fptrX,"%lf,", t);
        fprintf(fptrV,"%lf,", t);
        fprintf(fptrX,"%lf,", x[0]);
        fprintf(fptrV,"%lf\n", v[0]);
        fprintf(fptrX,"1\n");

        for(i = 1; i < N; i++)
        {
            x1[i] = x[i] + ((dt/2.0)*(f1(v,i)));
            v1[i] = v[i] + ((dt/2.0)*(f2(lambda,x,v,i)));

            x2[i] = x[i] + ((dt/2.0)*(f1(v1,i)));
            v2[i] = v[i] + ((dt/2.0)*(f2(lambda,x1,v1,i)));

            x3[i] = x[i] + (dt*(f1(v2,i)));
            v3[i] = v[i] + (dt*(f2(lambda,x2,v2,i)));

            xNew[i] = x[i] + (dt/6.0)*(f1(v,i)+2.0*f1(v1,i)+2.0*f1(v2,i)+f1(v3,i));
            vNew[i] = v[i] + (dt/6.0)*((f2(lambda,x,v,i)+2.0*f2(lambda,x1,v1,i))+2.0*f2(lambda,x2,v2,i)+f2(lambda,x3,v3,i));

            x[i] = xNew[i];
            v[i] = vNew[i];

            printf("Car %d position = %lf m\tCar %d velocity = %lf m/s\n",i,x[i],i,v[i]);
            fprintf(fptrX,"%lf,", t);
            fprintf(fptrV,"%lf,", t);
            fprintf(fptrX,"%lf,", x[i]);
            fprintf(fptrV,"%lf\n", v[i]);
            fprintf(fptrX,"1\n");
        }
    }
}

int main() {

    //User input
    double N = 10;
    double lambda = 5;
    double d = 2;
    double tmax = 50;
    double dt = .1;
    double limit = 30;

    double *x = malloc(sizeof(double)*N);
    double *v = malloc(sizeof(double)*N);
    double t = 0.0;
    int i;

    ///Setting initial conditions

    x[0] = 0;
    v[0] = 0;

    for(i = 1; i < N; i++)
    {
        x[i] = x[i-1] - d;
        v[i] = 0;
    }

    rungeKutta(t,x,v,N,lambda,tmax,dt,limit);

    return 0;
}
