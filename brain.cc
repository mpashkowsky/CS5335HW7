
#include <iostream>
#include <thread>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;
float frustration[60][60];
float heuristic[60][60];
float target[5][200];
float headtowards[2][200];
int headtowardscounter = 0;

void
callback(Robot* robot)
{
    int xpos = floor(robot->pos_x + 30);
    int ypos = floor(robot->pos_y + 30);
    for (int a=2;a<5;a++) {
    	if (robot->ranges[a].range < 2.0) {
    		frustration[xpos][ypos]=frustration[xpos][ypos]+1;
    	}
    }
    if (headtowards[0][headtowardscounter]==NULL) {

    float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float rgt = clamp(0.0, robot->ranges[4].range, 2.0);

    float spd = fwd - 1.0;
    float trn = clamp(-1.0, lft - rgt, 1.0);

    if (fwd < 1.2) {
      spd = 0;
      trn = 1;
    }

    robot->set_vel(spd + trn, spd - trn);
    } else {
    	double tarang = robot->pos_t;
    	float trn = 0;
    	float spd = 2.0f;

	if ((xpos == headtowards[0][headtowardscounter])&&(ypos == headtowards[1][headtowardscounter])){
		headtowardscounter = headtowardscounter + 1;
		return;
	}
	if (xpos != headtowards[0][headtowardscounter]){
		if (xpos > headtowards[0][headtowardscounter]){
			tarang = 3.14;
		} else {
			tarang = 0;
		}
	} else if (ypos != headtowards[1][headtowardscounter]) {
		if (ypos > headtowards[1][headtowardscounter]){
			tarang = 1.59;
		} else {
			tarang = 4.71;
		}
	}
	float angdif = fmod((tarang - robot->pos_t +6.28),6.28);
	if (abs(angdif)>0.5) {
		if (angdif > 0) {
			trn = 1;
		} else {
			trn = -1;
		}
	}
	if ((robot->ranges[2].range < 2)||(robot->ranges[4].range < 2)) {
		float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    		float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    		float rgt = clamp(0.0, robot->ranges[4].range, 2.0);
    		spd = fwd - 1.0;
    		float trn = clamp(-1.0, lft - rgt, 1.0);
    		if (fwd < 1.2) {
      			spd = 0;
      			trn = 1;
    		}
	} else if (trn == 0) {
		spd = 2.0f;
	} else {
		spd = 1.0f;
	}
	robot->set_vel(spd + trn, spd - trn);
    }
    
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

void
astar(Robot* robot)
{
	float xposition = floor(robot->pos_x + 30);
	float yposition = floor(robot->pos_y + 30);
	bool tested[60][60];
	for(int i = 0; i < 60; i++) {
		for(int j = 0; j < 60; j++) {
			float d = 100 - pow(100,(-(abs(i-50)+abs(j-30))/100));
			heuristic[i-1][j-1] = frustration[i-1][j-1]+d;
		}
	}
	target[0][0]=xposition;
	target[0][1]=yposition;
	int checked[60][60];
	while ((target[0][0]!=50)&&(target[0][1]!=30)) {
	  int test[4][3];
	  int targetx = target[0][0];
	  int targety = target[0][1];
	  checked[targetx][targety] = 1;
	  test[0][0] = targetx+1;
	  test[0][1] = targety;
	  test[0][2] = heuristic[targetx+1][targety];
	  test[1][0] = targetx-1;
	  test[1][1] = targety;
	  test[1][2] = heuristic[targetx-1][targety];
	  test[2][0] = targetx;
	  test[2][1] = targety+1;
	  test[2][2] = heuristic[targetx][targety+1];
	  test[3][0] = targetx;
	  test[3][1] = targety-1;
	  test[3][2] = heuristic[targetx][targety-1];
	  for (int i=0; i < 4; i++) {
	  	int testx = test[i][0];
	  	int testy = test[i][1];
	  	if (checked[testx][testy] != 1) {
	  	int a = 0;
	  	bool comparison = true;
	  	while (target[a][0] != NULL) {	
	  		if (test[i][0] == target[a][0]) {
	  			if (test[i][1] == target[a][1]) {
	  				comparison = false;
	  				if (test[i][0] < target[a][2]) {
	  					target[a][0] = test[i][0];
	  					target[a][1] = test[i][1];
	  					target[a][2] = test[i][2];
	  					target[a][3] = xposition;
	  					target[a][4] = yposition;	  				
	  				}
	  			}
	  		}
	  		a = a+1;
	  	}
	  	if (comparison) {
	  		int b = 0;
	  		while (target[b][0] != NULL) {
	  			b = b+1;
	  		}
	  		target[b][0] = test[i][0];
	  		target[b][1] = test[i][1];
	  		target[b][2] = test[i][2];
	  		target[b][3] = xposition;
	  		target[b][4] = yposition;
	  	}
	  	}  
	  }
	  bool cont = true;
	  int d = 0;
	  for (int c=0; c<200; c++) {
	  	if (cont) {
	  		if (target[c][0] == NULL) {
	  			cont = false;
	  			d = c;
	  		}
	  	}
	  }
	  float hold[5][200];
	  for (int h=0;h<5;h++) {
	  	for (int i=0;i<200;i++) {
	  		hold[h][i] = target[h][i];
	  	}
	  }
	  for (int f=0;f<d;f++) {
	  	float mincompare = hold[2][0];
	  	int minplace = 0;
	  	for (int g=1;g<d;g++) {
	  		if (mincompare>hold[2][g]) {
	  			mincompare = hold[2][g];
	  			minplace = g;
	  		}
	  	}
	  	target[0][f] = hold[0][minplace];
	  	target[1][f] = hold[1][minplace];
	  	target[2][f] = hold[2][minplace];
	  	target[3][f] = hold[3][minplace];
	  	target[4][f] = hold[4][minplace];
	  	hold[2][minplace] = 1000.0;
	  }
	  
	}
	headtowards[0][0] = 50;
	headtowards[1][0] = 30;
	while ((target[0][0]!=xposition)&&(target[1][0]!=yposition)){
		for(int a=0;a<200;a++) {
			if ((target[3][a]==headtowards[0][0])&&(target[4][a])) {
				int nexthead = a;
			}
		}
		float holdhead[2][200];
		for(int b=0;b<200;b++){
			if(headtowards[0][b]!=NULL) {
				holdhead[0][b+1]=headtowards[0][b];
				holdhead[1][b+1]=headtowards[1][b];
			}
		}
		for(int b=0;b<200;b++){
			if(holdhead[0][b]!=NULL) {
				headtowards[0][b]=holdhead[0][b];
				headtowards[1][b]=holdhead[1][b];
			}
		}
	}
	headtowardscounter = 0;		
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    rthr.detach();
    std::thread name(astar, &robot);
    name.detach();

}
