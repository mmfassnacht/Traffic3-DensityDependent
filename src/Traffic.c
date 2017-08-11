#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double f1(double v[], int i){return (v[i]);}
double f2(double lambda, double x[], double v[], int i) {return (lambda*(v[i-1] - v[i])/(x[i-1] - x[i]));}

void rungeKutta(double t, double x[], double v[], int N, double lambda, double tmax, double dt, double limit, int trialFlag) {

    FILE *fptrEV;
    FILE *fptrV;
    FILE *fptrX;

    fptrEV = fopen("trafficExactV.csv","w");
    fptrV = fopen("trafficV.csv","w");
    fptrX = fopen("trafficX3.csv","w");

    double *x1 = malloc(sizeof(double)*N), *x2 = malloc(sizeof(double)*N), *x3 = malloc(sizeof(double)*N), *xNew = malloc(sizeof(double)*N);
    double *v1 = malloc(sizeof(double)*N), *v2 = malloc(sizeof(double)*N), *v3 = malloc(sizeof(double)*N), *vNew = malloc(sizeof(double)*N);
    double exactV;
    int i;

    for(t = dt; t <= tmax; t+=dt)
    {
       printf("t: %lf s\n",t);

        //Special case -- first car. Setting initial velocity conditions.
        if(trialFlag == 1)
        {
            v[0] = t*2.0;           //Accelerating at 2 m/s^2
            if(v[0] > limit)        //Speed limit of 30m/s ~ 65mph
                v[0] = limit;
        }

        if(trialFlag == 3)
        {
            if(t > 5)
            {
                v[0] = v[0]-1;
                if(v[0] < 0)
                    v[0] = 0;
            }
        }

        if(trialFlag == 3 || trialFlag == 2)
        {
                //Break loop once all cars have stopped
                if(v[N-1] < .00000001)
                {
                    for(i = 0; i < N-1; i++)
                        printf("Stopping distance between car %d and car %d: %lf m\n", i+1, i+2, x[i]-x[i+1]);
                    return;
                }
        }

        v1[0] = v[0];
        v2[0] = v[0];
        v3[0] = v[0];

        //Updating position of leading car - to be used in next timestep.
        x1[0] = x[0] + ((dt/2.0)*(f1(v,0)));
        x2[0] = x[0] + ((dt/2.0)*(f1(v1,0)));
        x3[0] = x[0] + (dt*(f1(v2,0)));
        xNew[0] = x[0] + (dt/6.0)*(f1(v,0)+2.0*f1(v1,0)+2.0*f1(v2,0)+f1(v3,0));
        x[0] = xNew[0];

        printf("Car 0 position = %lf m\tCar 0 velocity = %lf m/s\n",x[0],v[0]);
        fprintf(fptrX,"%lf,", x[0]);
        fprintf(fptrV,"%lf,", v[0]);

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

            //Testing for crash case
            if(v[i] < 0 || x[i] >= x[i-1])
            {
                v[i] = 0;
                v[i-1] = 0;
                x[i] = x[i-1]+.001;
                printf("Crash!\n");
            }

            printf("Car %d position = %lf m\tCar %d velocity = %lf m/s\n",i,x[i],i,v[i]);
            fprintf(fptrX,"%lf,", x[i]);
            fprintf(fptrV,"%lf,", v[i]);
            exactV = lambda*log(x[i-1]-x[i]);
            fprintf(fptrEV,"%lf,", exactV);
        }
            fprintf(fptrX,"\n");
            fprintf(fptrV,"\n");
            fprintf(fptrEV,"\n");
    }
}

int main() {

    FILE *r;
    r = fopen("Test.txt","r");

    //User input
    int N;
    double lambda;
    double d;
    double tmax;
    double dt;
    double limit;
    int trialFlag;

    fscanf(r,"%d",&N);
    fscanf(r,"%lf",&lambda);
    fscanf(r,"%lf",&d);
    fscanf(r,"%lf",&tmax);
    fscanf(r,"%lf",&dt);
    fscanf(r,"%lf",&limit);
    fscanf(r,"%d",&trialFlag);

    double *x = malloc(sizeof(double)*N);
    double *v = malloc(sizeof(double)*N);
    double t = 0.0;
    int i;
    fscanf(r,"%lf",&v[0]);

    ///Setting initial conditions
    //Car 1 is given initial velocity. All others cars start from rest.
    if(trialFlag == 0)
    {
        x[0] = 0;

        for(i = 1; i < N; i++)
        {
            x[i] = x[i-1] - d;
            v[i] = 0;
        }
    }
    //All cars start from rest. First car is given acceleration. User sets speed limit.
    if(trialFlag == 1)
    {
        x[0] = 0;
        v[0] = 0;

        for(i = 1; i < N; i++)
        {
            x[i] = x[i-1] - d;
            v[i] = 0;
        }
    }
    //User inputs initial velocity for lead car. Following cars begin at 30 m/s.
    if(trialFlag == 2)
    {
        x[0] = 0;

        for(i = 1; i < N; i++)
        {
            x[i] = x[i-1] - d;
            v[i] = 30;
        }
    }
    //User inputs initial velocity for cars. First car decelerates quickly.
    if(trialFlag == 3)
    {
        x[0] = 0;

        for(i = 1; i < N; i++)
        {
            x[i] = x[i-1] - d;
            v[i] = v[0];
        }
    }
    rungeKutta(t,x,v,N,lambda,tmax,dt,limit,trialFlag);
    return 0;
}

