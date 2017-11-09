#include <SDL\SDL.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <math.h>
#include <time.h>
#include "constants.h"

const int SCREEN_WIDTH = MAX_X;
const int SCREEN_HEIGHT = MAX_Y;

//Starts up SDL and creates window
bool init();

//Loads media
bool loadMedia();

//Frees media and shuts down SDL
void close();

//Loads individual image as texture
SDL_Texture* loadTexture( std::string path );

//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//The window renderer
SDL_Renderer* gRenderer = NULL;

////////////////////////////////////initialization variable/////////////
float x,y,q;                    //estimated position variables
float dp,dq;                  //movement inputs from robot simulator code
float sx,sy,sq;                 //variances of position variables
int converged;
float x_rand,y_rand,q_rand;     //random variables corrected to fit in environment
float rand_value;
//int w;                          //particle weight variable
int neighborhood;

float tx, ty, tq;
int goal_x,goal_y;
int obs_num=0;
//array of particles with positions, headings, and Ws
float particle_map[VARS][NUM_PARTICLES];
int obstacles[2][NUM_OBSTACLES];
int sonar_angle[MAX_ANGLE*2/ANGLE_STEP];

//array of sonar distance readings
int sonar[MAX_ANGLE*2/ANGLE_STEP];

//function prototypes
void particle_filter(void);
void init_dist(void);
void resample(void);
void print_file(int num);
void print_obst(void);
float dist(float x,float y);
float random_num(void);
float gaussian(void);
void update_position();
void weight_particles(void);
void print_init(void);
void generate_outputs(void);
int obst_detect(float x,float y);
void hist(void);
void generateSonar();
void planPath();
void moveRobot();
void print_path(int num);
int sign(int v);
int getSonar(float x, float y, float q);
int getSonar2(float x, float y, float q);
float distance(float x0,float y0, float x1,float y1);
int getSonar3(float x , float y , float q);
int map[int32_t(MAX_X)][int32_t(MAX_Y)];
void dummy_sonar(void);
bool first_itrate = true; /// to initialize first iteration for sonar to cutoff for particles redistributuion 


///////////////////////////////////////////////////////////
bool init()
{
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
		success = false;
	}
	else
	{
		//Set texture filtering to linear
		if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) )
		{
			printf( "Warning: Linear texture filtering not enabled!" );
		}

		//Create window
		gWindow = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
		if( gWindow == NULL )
		{
			printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
			success = false;
		}
		else
		{
			//Create renderer for window
			gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED );
			if( gRenderer == NULL )
			{
				printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
				success = false;
			}
			else
			{
				//Initialize renderer color
				SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );

				//Initialize PNG loading
			}
		}
	}

	return success;
}

bool loadMedia()
{
	//Loading success flag
	bool success = true;

	//Nothing to load
	return success;
}

void close()
{
	//Destroy window	
	SDL_DestroyRenderer( gRenderer );
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	gRenderer = NULL;

	//Quit SDL subsystems
	SDL_Quit();
}

SDL_Texture* loadTexture( std::string path )
{
	//The final texture
	SDL_Texture* newTexture = NULL;

	//Load image at specified path

	return newTexture;
}

void draw_rect1(double x ,double y , int wi, int li , int angle)
{
	double x1, x2, x3, x4, y1, y2, y3, y4;
	x1 = x + wi * cos((angle) * (M_PI / 180));
	y1 = y + wi * sin((angle) * (M_PI / 180));
	x2 = x1 + li * cos((angle + 270) * (M_PI / 180));
	y2 = y1 + li * sin((angle + 270) * (M_PI / 180));
	x3 = x2 + wi * cos((angle + 180) * (M_PI / 180));
	y3 = y2 + wi * sin((angle + 180) * (M_PI / 180));
	x4 = x3 + li * cos((angle + 90) * (M_PI / 180));
	y4 = y3 + li * sin((angle + 90) * (M_PI / 180));
	SDL_SetRenderDrawColor( gRenderer,0,0,0,255);		
	SDL_RenderDrawLine( gRenderer,x1,y1,x2,y2);
	SDL_RenderDrawLine( gRenderer,x2,y2,x3,y3);


}
void draw_rect2(double x, double y, int wi, int li, int angle)
{

	double x1, x2, x3, x4, y1, y2, y3, y4;
	x1 = x + li * cos((angle+90) * (M_PI / 180));
	y1 = y + li * sin((angle+90) * (M_PI / 180));
	x2 = x1 + wi * cos((angle + 0) * (M_PI / 180));
	y2 = y1 + wi * sin((angle + 0) * (M_PI / 180));
	x3 = x2 + li * cos((angle + 270) * (M_PI / 180));
	y3 = y2 + li * sin((angle + 270) * (M_PI / 180));
	x4 = x3 + wi * cos((angle + 180) * (M_PI / 180));
	y4 = y3 + wi * sin((angle + 180) * (M_PI / 180));
	SDL_SetRenderDrawColor( gRenderer,0,0,0,255);		
	SDL_RenderDrawLine( gRenderer,x1,y1,x2,y2);
	SDL_RenderDrawLine( gRenderer,x2,y2,x3,y3);


}
void draw_rect3(double x, double y, int wi, int li, int angle)
{
	double x1, x2, x3, x4, y1, y2, y3, y4;
	x1 = x + wi * cos((angle+180) * (M_PI / 180));
	y1 = y + wi * sin((angle+180) * (M_PI / 180));
	x2 = x1 + li * cos((angle + 90) * (M_PI / 180));
	y2 = y1 + li * sin((angle + 90) * (M_PI / 180));
	x3 = x2 + wi * cos((angle + 0) * (M_PI / 180));
	y3 = y2 + wi * sin((angle + 0) * (M_PI / 180));
	x4 = x3 + li * cos((angle + 270) * (M_PI / 180));
	y4 = y3 + li * sin((angle + 270) * (M_PI / 180));
	SDL_SetRenderDrawColor( gRenderer,255,0,0,255);		
	SDL_RenderDrawLine( gRenderer,x1,y1,x2,y2);
	SDL_RenderDrawLine( gRenderer,x2,y2,x3,y3);
	
}
void draw_rect4(double x, double y, int wi, int li, int angle)
{
	double x1, x2, x3, x4, y1, y2, y3, y4;
	x1 = x + li * cos((angle + 270) * (M_PI / 180));
	y1 = y + li * sin((angle + 270) * (M_PI / 180));
	x2 = x1 + wi * cos((angle + 180) * (M_PI / 180));
	y2 = y1 + wi * sin((angle + 180) * (M_PI / 180));
	x3 = x2 + li * cos((angle + 90) * (M_PI / 180));
	y3 = y2 + li * sin((angle + 90) * (M_PI / 180));
	x4 = x3 + wi * cos((angle + 0) * (M_PI / 180));
	y4 = y3 + wi * sin((angle + 0) * (M_PI / 180));
	SDL_SetRenderDrawColor( gRenderer,255,0,0,255);		
	SDL_RenderDrawLine( gRenderer,x1,y1,x2,y2);
	SDL_RenderDrawLine( gRenderer,x2,y2,x3,y3);
}

void draw_rect(int x,int y, int wi,int li,int angle)
{
	draw_rect1(x,y,wi/2,li/2,angle);
	draw_rect2(x,y,wi/2,li/2,angle);
	draw_rect3(x,y,wi/2,li/2,angle);
	draw_rect4(x,y,wi/2,li/2,angle);
}
void draw_goal()
{
	SDL_Rect fillRect = { goal_x ,goal_y,10,10};
	SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
	SDL_RenderFillRect( gRenderer, &fillRect );
}
int main( int argc, char* args[] )
{
	///main init
	srand(time(NULL));    //seed random number generator     
	//Start up SDL and create window
	if( !init() )
	{
		printf( "Failed to initialize!\n" );
	}
	else
	{
		//Load media
		if( !loadMedia() )
		{
			printf( "Failed to load media!\n" );
		}
		else
		{	
			//Main loop flag
			bool quit = false;

			//Event handler
			SDL_Event e;
			/////////////////////////////////////map
			for(int mx=0;mx<MAX_X;mx++)
			{
				for(int my=0;my<MAX_Y;my++)
				{
					if(mx==0)
					{
						map[0][my]=1;
					}
					else if (mx > 0 && mx <MAX_X-1)
					{
						map[mx][0]=1;
						map[mx][int32_t(MAX_Y)-1]=1;
					}
					else if (mx==MAX_X-1)
					{
						map[int32_t(MAX_X)-1][my]=1;
					}
					else
					{
						map[mx][my]=0;
					}
				}
			}
			//////////////////initial//////////////
			int angle=0;
			int i=0;
			bool st =false,st1=false,st3=false;
			goal_x=320;
			goal_y=220;
			dp = dq = 0;
			tx = 50.0f;
			ty = 200.0f;
			tq = 0.0f;
			init_dist();
		
			/////////////////////////////////////////////////
			//While application is running
			while( !quit )
			{
				//Handle events on queue
				while( SDL_PollEvent( &e ) != 0 )
				{
					//User requests quit
					if( e.type == SDL_QUIT )
					{
						quit = true;
					}
					else if(e.type == SDL_KEYDOWN)
					{
						if(e.key.keysym.scancode == SDL_SCANCODE_Q)
						{
							SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
							SDL_RenderClear( gRenderer );
							draw_rect(tx,ty,60,40,tq);
							print_obst(); 
							SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
							generateSonar();
							particle_filter();
							print_init();
							dp=1;
							dq=-10;
							moveRobot();
							for (int t = 0 ; t<MAX_ANGLE*2/ANGLE_STEP;t++)
							{
								int new_x,new_y;
								new_x =tx + sonar[t]*cos((sonar_angle[t]) * (M_PI / 180));
								new_y = ty + sonar[t]*sin((sonar_angle[t]) * (M_PI / 180));
								SDL_RenderDrawLine( gRenderer,tx,ty,new_x,new_y);
							}

							//Update screen
							SDL_RenderPresent( gRenderer );
						}
						else if(e.key.keysym.scancode == SDL_SCANCODE_W)
						{
							SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
							SDL_RenderClear( gRenderer );
							//Draw blue horizontal line
							draw_rect(tx,ty,60,40,tq);
							print_obst(); 
							SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
							generateSonar();
							particle_filter();
							print_init();
							dp=10;
							dq=0;
							moveRobot();
							for (int t = 0 ; t<MAX_ANGLE*2/ANGLE_STEP;t++)
							{
								int new_x,new_y;
								new_x =tx + sonar[t]*cos((sonar_angle[t]) * (M_PI / 180));
								new_y = ty + sonar[t]*sin((sonar_angle[t]) * (M_PI / 180));
								SDL_RenderDrawLine( gRenderer,tx,ty,new_x,new_y);
							}

						//Update screen
						SDL_RenderPresent( gRenderer );
						}
						else if(e.key.keysym.scancode == SDL_SCANCODE_E)
						{
							SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
							SDL_RenderClear( gRenderer );
							//Draw blue horizontal line
							draw_rect(tx,ty,60,40,tq);
							print_obst(); 
							SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
							generateSonar();
							particle_filter();
							print_init();
							dq=10;
							dp=1;
							moveRobot();
							for (int t = 0 ;t<MAX_ANGLE*2/ANGLE_STEP;t++)
							{
								int new_x,new_y;
								new_x = tx + sonar[t]*cos((sonar_angle[t]) * (M_PI / 180));
								new_y = ty + sonar[t]*sin((sonar_angle[t]) * (M_PI / 180));
								SDL_RenderDrawLine( gRenderer,tx,ty,new_x,new_y);
							}

						//Update screen
						SDL_RenderPresent( gRenderer );
						}
					}
					if(e.type == SDL_MOUSEBUTTONDOWN && st1==true)
					{
						if(st1==true)
						{
							obstacles[X][obs_num] = e.motion.x;
							obstacles[Y][obs_num] = e.motion.y;
							obs_num++;
						}
						if (obs_num == NUM_OBSTACLES)
						{
							st1=false;
							st3=true;
						}
					}
				}
				if (st1 && !st3)
				{
					//Clear screen
					SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
					SDL_RenderClear( gRenderer );
					SDL_Rect fillRect = { e.motion.x,e.motion.y,OBS_SIZE_X,OBS_SIZE_Y};
					SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, 0xFF );
					SDL_RenderFillRect( gRenderer, &fillRect );
					print_obst();   
					//Draw blue horizontal line
					draw_rect(tx,ty,60,40,angle);
					//Update screen
					SDL_RenderPresent( gRenderer );
				}
				else if(st3 && !st1)
				{
						
				}
			}
		}
	}

	//Free resources and close SDL
	close();

	return 0;
}

//generate simulated noisy sonar readings
void generateSonar()
{
	int angle;
	for(int i = 0; i < MAX_ANGLE*2/ANGLE_STEP; ++i)
	{
		sonar_angle[i] = i*ANGLE_STEP-MAX_ANGLE+tq;
		sonar[i] = (getSonar2(tx,ty,sonar_angle[i]) + gaussian()*SONAR_SIGMA);
		if( sonar[i]>CUTOFF_RANGE )
		{
			sonar[i] = CUTOFF_RANGE;
		}
	}
}
void dummy_sonar()
{
	for(int i = 0; i < MAX_ANGLE*2/ANGLE_STEP; ++i)
	{
			sonar[i] = CUTOFF_RANGE;
	}
}
//move robot position
void moveRobot()
{
	tq += dq;
	if(tq > MAX_Q_INT)
	{
		tq -= 360;
	}
	if(tq < -MAX_Q_INT)
	{
		tq += 360;
	}
	tx += dp*cos(tq*PI/180);
	ty += dp*sin(tq*PI/180);
}


//main particle filtering function
void particle_filter(void)
{
	update_position();
	weight_particles();
	resample();
}

//generate initial uniform distribution
void init_dist(void)
{

	int i;

	//set all particle weights identically
	for(i=0;i<NUM_PARTICLES;i++)
	{
		particle_map[W][i]=1/NUM_PARTICLES_F;

		//gen X          
		particle_map[X][i]=random_num()*MAX_X;

		//gen Y
		particle_map[Y][i]=random_num()*MAX_Y;

		//gen Q
		particle_map[Q][i]= random_num()*MAX_Q*2;

		//check for obstacles
		if(obst_detect(particle_map[X][i],particle_map[Y][i]) )
		{
			--i;
			continue;
		}
	}       
}

void resample(void)
{

	int j;
	float new_x[NUM_PARTICLES];         //init new x position array   
	float new_y[NUM_PARTICLES];         //init new y position array
	float new_q[NUM_PARTICLES];         //init new heading array
	float cdf[NUM_PARTICLES];           //init CDF array
	float ran[NUM_PARTICLES];           //init random number array

	//initialize CDF
	cdf[0] = particle_map[W][0];        
	for(int i = 1; i < NUM_PARTICLES; ++i)
	{  
		cdf[i] = cdf[i-1] + particle_map[W][i];
	}

	//initialize random array 
	for(int i = 0; i < NUM_PARTICLES; ++i)
	{
		ran[i] = random_num()*cdf[NUM_PARTICLES-1];
	}

	//make copies as many times as numbers occur in CDF
	for(int i = 0; i < NUM_PARTICLES; ++i)
	{
		j = 0;
		while(cdf[j] < ran[i])
		{
			++j;
		}
		new_x[i] = particle_map[X][j];
		new_y[i] = particle_map[Y][j];
		new_q[i] = particle_map[Q][j];
	}

	for(int i = 0; i < NUM_PARTICLES; ++i)
	{
		particle_map[X][i] = new_x[i];
		particle_map[Y][i] = new_y[i];
		particle_map[Q][i] = new_q[i];
		particle_map[W][i] = 1.0/NUM_PARTICLES_F;
	}


}

//generate random number scaled between 0 and 1
float random_num(void)
{

	return (rand()/(float)RAND_MAX);

}

//generate gaussian number for measurement noise     
float gaussian(void)
{
	float sum = 0.0;
	int it = 10;
	for (int i = 0; i < it; ++i)
	{
		sum += random_num()-.5;
	}
	//return sqrt(-2.0 * log( (float)random_num()) ) * cos(2.0 * PI * random_num());
	return sum*1.0871;
}

//update positions of particles based on movement model that includes noise
//if particles leave the map, or a heading is less than 0 or greater than 2pi,
//generate a new random particle
void update_position()
{
	int i;
	for(i=0;i<NUM_PARTICLES;i++)
	{
		particle_map[Q][i] += dq*(1 + gaussian()*ENCODER_NOISE);
		if(particle_map[Q][i] > MAX_Q_INT)
		{
			particle_map[Q][i] -= 360;
		}
		if(particle_map[Q][i] < -MAX_Q_INT)
		{
			particle_map[Q][i] += 360;
		}
		particle_map[X][i] += dp*cos(particle_map[Q][i]*PI/180)*(1 + gaussian()*ENCODER_NOISE);  
		particle_map[Y][i] += dp*sin(particle_map[Q][i]*PI/180)*(1 + gaussian()*ENCODER_NOISE);
	}
}

void weight_particles(void)
{
	//weight particles based on sonar matches          
	int i=0,j=0,diff=0;
	double discrep=0;
	double sm1,sm2,sm3,sm4,sm5;
	int s_part[MAX_ANGLE*2/ANGLE_STEP];

	if (first_itrate)
	{
		dummy_sonar();
		first_itrate=false;
	}
	else
	{
		generateSonar();
	}

	for( i=0;i<NUM_PARTICLES;i++)
	{
		if( obst_detect(particle_map[X][i],particle_map[Y][i])||( (particle_map[X][i]>MAX_X) || (particle_map[X][i]<0.0) ) ||( (particle_map[Y][i]>MAX_Y) || (particle_map[Y][i]<0.0) )  )
		{
			particle_map[W][i] = 0.0;
		}
		else
		{
			for(j=0;j<MAX_ANGLE*2/ANGLE_STEP;j++)
			{
				//s_part[j]=getSonar(particle_map[X][i],particle_map[Y][i],particle_map[Q][i]+j*ANGLE_STEP-MAX_ANGLE);
				s_part[j]=getSonar2(particle_map[X][i],particle_map[Y][i],particle_map[Q][i]+j*ANGLE_STEP-MAX_ANGLE);
				diff =  abs( sonar[j] - s_part[j] );
				/*if( diff >=50)
				discrep += 50;
				else discrep += diff;*/
				discrep += diff;
				
			}
			
			particle_map[W][i] = 1 - discrep/(CUTOFF_RANGE/1.5*(MAX_ANGLE*2/ANGLE_STEP));

			discrep=0;
		}
	}
}    

//print particle positions at each step 
void print_file()
{
	int i;

	for(i=0;i<NUM_PARTICLES;i++)
		SDL_RenderDrawPoint(gRenderer,particle_map[X][i],particle_map[Y][i]);

}

//print robot location at each step
void print_path(int num){  
	FILE *outp;
	char name[30];
	sprintf(name,"output/particle%d.txt",num);
	outp = fopen(name,"w");

	fprintf(outp,"%.2f\t %.2f\n",tx,ty);

	fclose(outp);
}

//print initial distribution  
void print_init(void)
{
	int i;
	for(i=0;i<NUM_PARTICLES;i++)
		SDL_RenderDrawPoint(gRenderer,particle_map[X][i],particle_map[Y][i]);
}

//generate obstacle file for visualization
void print_obst(void)
{
	int x,y,obst[50];

	for(int i = 0; i <obs_num; ++i)
	{
		SDL_Rect fillRect = { obstacles[X][i],obstacles[Y][i],OBS_SIZE_X,OBS_SIZE_Y};
		SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, 0xFF );
		SDL_RenderFillRect( gRenderer, &fillRect );
		for(x=obstacles[X][i];x<obstacles[X][i]+OBS_SIZE_X;x++)
		{
			for(y=obstacles[Y][i];y<obstacles[Y][i]+OBS_SIZE_Y;y++)
			{
				map[x][y]=1;
			}
		}

	}          
}

//detect obstacles at x,y
int obst_detect(float x,float y)
{
	int obstacle=0,i;
	for(i = 0; i < NUM_OBSTACLES; ++i)
	{
		if(x <= obstacles[X][i] + OBS_SIZE_X && x >= obstacles[X][i])
		{
			if(y <= obstacles[Y][i] + OBS_SIZE_Y && y >= obstacles[Y][i])
			{
				obstacle = 1;
				break;
			}
		}
	}   
	return obstacle;    
}

//compute mean and standard deviation of x,y,q
void generate_outputs(void)
{
	int i = 0;
	float sum = 0.0;
	//Calculate the mean of the particles
	for(i = 0; i < NUM_PARTICLES; ++i)
	{
		sum+= particle_map[X][i];
	}
	x = sum/NUM_PARTICLES;

	sum = 0.0;
	for(i = 0; i < NUM_PARTICLES; ++i)
	{
		sum+= particle_map[Y][i];
	}
	y = sum/NUM_PARTICLES;

	sum = 0.0;
	for(i = 0; i < NUM_PARTICLES; ++i)
	{
		sum+= particle_map[Q][i];
	}
	q = sum/NUM_PARTICLES;

	//Calculate the sigma of the particles
	sum = 0.0;
	for(i = 0; i < NUM_PARTICLES; ++i)
	{
		sum+= particle_map[X][i]*particle_map[X][i];
	}
	sx = sqrt(sum/NUM_PARTICLES - x*x);

	sum = 0.0;
	for(i = 0; i < NUM_PARTICLES; ++i)
	{
		sum+= particle_map[Y][i]*particle_map[Y][i];
	}
	sy = sqrt(sum/NUM_PARTICLES - y*y);

	sum = 0.0;
	for(i = 0; i < NUM_PARTICLES; ++i)
	{
		sum+= particle_map[Q][i]*particle_map[Q][i];
	}
	sq = sqrt(sum/NUM_PARTICLES - q*q);

	if( (sx < CONVERGED) && (sy < CONVERGED) && (sq < CONVERGED) )
	{
		converged = 1;
	}
	else
	{
		converged = 0;
	}


	print_init();
}

int sign(int v)
{
	return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

//unused sonar generation function
int getSonar(float x, float y, float q)
{

	int ret = MAX_RANGE;
	double a = q*PI/180; 
	for (int i = 0; i < MAX_RANGE; ++i)
	{
		if( obst_detect( (x+i*cos(a)), (y+i*sin(a)) ) )
		{
			ret = i;
			break;
		}
	}
	return ret;
}

//computer sonar readings at a given pose x,y,q
int getSonar2(float x, float y, float q)
{
	int ret = 20;
	int temp = 0;
	int sx,sy;
	double a = (q)*PI/180;
	while( ret < CUTOFF_RANGE)
	{
		sx =x + ret*cos(a);
		sy =y + ret*sin(a);	
		if(sx>=MAX_X)
		{
			sx=MAX_X-1;
		}
		else if(sx<0)
		{
			sx=0;
		}
		if(sy>=MAX_Y)
		{
			sy=MAX_Y-1;
		}
		else if(sy<0)
		{
			sy=0;
		}
		if(map[sx][sy] == 1)
		{
			while(1)
			{
				ret--;
				sx =x + ret*cos(a);
				sy =y + ret*sin(a);
				if(map[sx][sy]==0)
				{
					break;
				}
			}
			break;
		}
		else if(map[sx][sy] == 0)
		{
			ret += 10;
		}
	}
	
	return ret;
}



//calculate distance between two points
float distance(float x0,float y0, float x1,float y1)
{
	float xdiff = x1 - x0;
	float ydiff = y1 - y0;
	return sqrt(xdiff*xdiff + ydiff*ydiff);
}