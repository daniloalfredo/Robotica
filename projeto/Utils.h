#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <cstdio>
#include <vector>

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

//Distribuição de probabilidade
double normalDistribution(double x);

//Matrizes
class Matrix
{
	public:
		std::vector<std::vector<float> > mat;
		
		//Cria uma matriz nula (0x0)
		Matrix() 
		{
		}
		
		//Cria uma Matriz MxN
		Matrix(int m, int n)
		{
			ResizeAndNulify(m, n);
		}
		
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
		
		void Resize(int m, int n)
		{
			mat.resize(m);
			
			for(int i = 0; i < mat.size(); i++)
		        mat[i].resize(n);
		}
		
		void ResizeAndNulify(int m, int n)
		{
			Resize(m, n);
		
			for(int i = 0; i < mat.size(); i++)
		    { 
		        for(int j = 0; j < mat[i].size(); j++) 
		            mat[i][j] = 0.0;
		    }
		}
		
		void Print()
		{
			printf("[");
			for(int i = 0; i < (int) mat.size(); i++)
			{
				for(int j = 0; j < (int) mat[i].size(); j++)
					printf("%.2f ", mat[i][j]);
				
				if(i < (int) mat.size()-1)
					printf("\n");
			}
			printf("]\n");
		}
		
		//Operador de atribuição
		Matrix& operator=(Matrix& o)
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
		
		//Multiplicar por constante (a constante tem que ficar depois ex: M*5.0)
		Matrix& operator*(float k)
		{
			for(int i = 0; i < Rows(); i++)
			{ 
				for(int j = 0; j < Cols(); j++) 
		            mat[i][j] *= k;
			}
			
			return *this;
		}
};

Matrix operator+(Matrix A, Matrix B);
Matrix operator-(Matrix A, Matrix B);
Matrix operator*(Matrix A, Matrix B);
Matrix Transpose(Matrix A);
Matrix Invert3x3(Matrix A);

#endif
