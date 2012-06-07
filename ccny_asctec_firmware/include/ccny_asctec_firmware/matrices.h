#ifndef MATRICES_H
#define MATRICES_H

#include <math.h>
#include <stdlib.h>

typedef float** matrix;  // designate a matrix as an array (2D) of floatsdef Row* Matrix;

void addMatrix2 (float A[2][2], float B[2][2], float C[2][2]);
void subMatrix2 (float A[2][2], float B[2][2], float C[2][2]);
void multMatrix2(float A[2][2], float B[2][2], float C[2][2]);
void subVector2 (float A[2], float B[2], float C[2]);
void multMatVec2(float A[2][2], float B[2], float C[2]);
void invert2(float A[2][2], float B[2][2]);
void VectmultSc2(float V[2], float scl, float V1[2]);
void addVector2(float V[2], float V1[2], float V2[2]);
//void allocMatrix(matrix mat, int nrows, int ncolumns);
void addMatrix (matrix A, matrix B, matrix C, int m, int n);
void addVector (float* A, float* B, float* C, int m);
void subMatrix (matrix A, matrix B, matrix C, int m, int n);
void subVector (float* A, float* B, float* C, int m);
void negMatrix(matrix A, int m, int n);
void multMatrix(matrix A, matrix B, matrix C, int Am, int An, int Bn);
void multMatVec(matrix A, float* B, float* C, int Am, int Bn);
void transpose(matrix A, matrix B, int Am, int An);
void invert3(matrix A, matrix B);
float determinant(matrix A,matrix tmp, int n);
void getMinor(matrix A, matrix B,int An, int m, int n);
void invt(matrix A, matrix B, matrix tmp1, matrix tmp2, int An);
float BhQhBhT(matrix Qh, float* Bh, float* tmp);
void VectdivSc(float* V, float scl);

/*void allocMatrix(float** mat, int nrows, int ncolumns)
{
	mat = (float **)malloc(nrows * ncolumns * sizeof(float));

	float* temp = (float*) malloc(nrows * ncolumns * sizeof(float));

	for(int i = 0; i < nrows; i++)
		mat[i] = temp + (i * ncolumns);

}*/

/*void allocMatrix(float** mat, int nrows, int ncolumns)
{
	mat = malloc(ncolumns * sizeof(float*));
  for(int i = 0; i < ncolumns; i++)
   mat[i] = malloc(nrows * sizeof(float));
}*/


#endif /*MATRICES_H_*/


