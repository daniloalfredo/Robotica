#include "ProbMap.h"

ProbMap::ProbMap()
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++){
			this->covar[i][j] = 0;
		}
	}
}

void ProbMap::UpdateMap()
{
	this->ErrorPropagationOdometry();
	for (int i = 0; i < CELLS_X; i++)
	{
		for (int j = 0; j < CELLS_Y; j++)
		{
			for (int k = 0; k < CELLS_DEG; k++)
			{
				this->ActionUpdate(i, j, k);
				this->PerceptionUpdate(i, j, k);
			}
		}
	}
}

void ProbMap::ActionUpdate(int x, int y, int deg) //Atualiza a célula [x,y, theta]
{
	
}

void ProbMap::ErrorPropagationOdometry(simxFloat dPhiL, simxFloat dPhiR, float* varX, float* varY)
{
	
}
