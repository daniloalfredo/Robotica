#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <unistd.h>
 
extern "C" {
#include "extApi.h"
/* #include "extApiCustom.h" if you wanna use custom remote API functions! */
}

#define PI M_PI

//Obter sinal do número
float fsignal(float v);

//Números aleatórios
float rand_beetween_0_and_1();
float rand_signal();

//Funções de Ângulos
float to_pi_range(float radians);
float to_2pi_range(float radians);
float to_rad(float degrees);
float to_deg(float radians);
float to_pos_deg(float radians);
float smallestAngleDiff(float target, float source);

//Funções de Tempo
float GetSimulationTimeInSecs(simxInt clientID);
float GetTimeSinceLastCommandInSecs(simxInt clientID, float lastCommandTime);

//Distribuição de probabilidade
double normalDistribution(double x);

//Matrizes
class Matrix
{
	public:
		std::vector<std::vector<float> > mat;
		
		int Rows()
		{
			return (int) mat.size();
		}
		
		int Cols()
		{
			if(this->Rows() > 0)
				return (int) mat[0].size();
			return 0;
		}
		
		Matrix() 
		{
		}
		
		//Cria uma Matriz MxN
		Matrix(int m, int n)
		{
			mat.resize(m);
			
			for(int i = 0; i < mat.size(); i++)
		        mat[i].resize(n);
		
			for(int i = 0; i < mat.size(); i++)
		    { 
		        for(int j = 0; j < mat[i].size(); j++) 
		            mat[i][j] = 0.0;
		    }
		}
		
		Matrix Transpose()
		{
			Matrix t(Cols(), Rows());
			
			for(int i = 0; i < t.mat.size(); i++)
		    { 
		        for(int j = 0; j < t.mat[i].size(); j++) 
		            t.mat[i][j] = mat[j][i];
		    }
			
			return t;
		}
		
		Matrix& operator =(Matrix& o)
    	{
		    mat.resize(o.mat.size());
		    
		    for(int i = 0; i < mat.size(); i++)
		        mat[i].resize(o.mat[i].size());
		    
		    for(int i = 0; i < mat.size(); i++)
		    { 
		        for(int j = 0; j < mat[i].size(); j++) 
		            mat[i][j] = o.mat[i][j];
		    }
		        
		    return *this;
		}
		
		Matrix& operator *(Matrix& o)
		{
			if(mat[0].size() != o.mat.size())
				return *this;

			Matrix tm;
			tm.mat.resize(mat.size());
        
			for(int i = 0; i < tm.mat.size(); i++)
				tm.mat[i].resize(o.mat[0].size());

			for(int i = 0; i < tm.mat.size(); i++)
			{ 
				for(int j = 0; j < tm.mat[i].size(); j++) 
				{
		            tm.mat[i][j] = 0;
		            for(int c = 0; c < mat[i].size(); c++) 
		                tm.mat[i][j] += mat[i][c] * o.mat[c][j];
				}
			}
			
			*this = tm;
			return *this;
		}
		
		void Print()
		{
			if(Rows() <= 0 || Cols() <= 0)
			{	
				printf("[]\n");
				return;
			}
			
		
			printf("[\n");
			for(int i = 0; i < (int) mat.size(); i++)
			{
				for(int j = 0; j < (int) mat[i].size(); j++)
					printf("%.2f ", mat[i][j]);
				printf("\n");
			}
			printf("]\n");
		}
};

/*
	//---------------------------
	EXEMPLO DE USO DAS MATRIZES

	Matrix a(2, 2);
	Matrix b(2, 1);
	
	a.mat[0][0] = 1;
	a.mat[0][1] = 0;
	a.mat[1][0] = 0;
	a.mat[1][1] = 1;
	
	b.mat[0][0] = 5;
	b.mat[1][0] = 8;
	
	Matrix c = a*b;
		
	c.Print();
	c.Transpose().Print();
*/

#endif
