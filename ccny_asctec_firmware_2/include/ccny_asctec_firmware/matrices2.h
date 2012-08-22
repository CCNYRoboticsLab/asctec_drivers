#ifndef MATRICES_H
#define MATRICES_H
#include <math.h>

typedef float** matrix;  // designate a matrix as an array (2D) of floats

void addMatrix (matrix A, matrix B, matrix C, int m, int n);
void addVector (float* A, float* B, float* C, int m);
void subMatrix (matrix A, matrix B, matrix C, int m, int n);
void subVector (float* A, float* B, float* C, int m);
void negMatrix(matrix A, int m, int n);
void multMatrix(matrix A, matrix B, matrix C, int Am, int An, int Bn);
void multMatVec(matrix A, float* B, float* C, int Am, int Bn);
void transpose(matrix A, matrix B, int Am, int An);
void invert3(matrix A, matrix B);
void invert2(matrix A, matrix B);
float determinant(matrix A,matrix tmp, int n);
void getMinor(matrix A, matrix B,int An, int m, int n);
void invt(matrix A, matrix B, matrix tmp1, matrix tmp2, int An);
float BhQhBhT(matrix Qh, float* Bh, float* tmp);
void VectdivSc(float* V, float scl);


void addMatrix (matrix A, matrix B, matrix C, int m, int n){
	/*C= A+B;
	 * A,B and C need to be the same size (m by n); function does not check*/
	int i, j;
	for(i=0; i<m; ++i){
		for(j=0; j<n; ++j){
			C[i][j] = A[i][j] + B[i][j];
		}//inner for
	}//outer for
}//add matrices

void addVector (float* A, float* B, float* C, int m){
	/*C= A+B;
	 *  A,B and C need to be the same size (m); function does not check*/
	int i;
	for(i=0; i<m; ++i){
		C[i] = A[i] + B[i];
	}//for
}//add Vectors

void subMatrix (matrix A, matrix B, matrix C, int m, int n){
	/*C= A-B;
	 * A,B and C need to be the same size (m by n); function does not check*/
	int i, j;
	for(i=0; i<m; ++i){
			for(j=0; j<n; ++j){
				C[i][j] = A[i][j] - B[i][j];
			}//inner for
	}//outer for
}//sub matrices

void subVector (float* A, float* B, float* C, int m){
	/*C= A-B;
	 * A,B and C need to be the same size (m); function does not check*/
	int i;
	for(i=0; i<m; ++i){
		C[i] = A[i] - B[i];
	}//for
}//add Vectors



void negMatrix(matrix A, int m, int n){
	/*negate matrix*/
	int i,j;
	for(i=0; i<m; ++i){
		for(j=0; j<n; ++j){
			A[i][j] = -A[i][j];
		}//inner for
	}//outer for
}//negMAtrix


void multMatrix(matrix A, matrix B, matrix C, int Am, int An, int Bn){
	//C = A*B
	//A,B and C need to be of correct dimensions, function does not check
    int i,j,k;
    float sum;

    for(i=0; i < Bn; i++){
    	for(j=0; j < Am; j++){
    		sum=0;
    		for(k=0; k < An; k++)
    			sum += A[j][k] * B[k][i];
    		C[j][i] = sum;
    	}//inner for
    }//outer for
}

void multMatVec(matrix A, float* B, float* C, int Am, int Bn){
	/*C = A*B
	 *A,B and C need to be of correct dimensions, function does not check */


	int j,k;
	float sum;
  	for(j=0; j < Am; j++){
  		sum=0;
   		for(k=0; k < Bn; k++)
   			sum += A[j][k] * B[k];
   		C[j] = sum;
  		}//for
}//multMatVec


void transpose(matrix A, matrix B, int Am, int An){
	/*B = A'*/
	int i,j;
	 for(i=0; i < Am; i++)
		for(j=0; j < An; j++)
		    B[j][i] = A[i][j];

}

void invert3(matrix A, matrix B){
/* B = A^-1 for 3x3 matrices*/
	float det = (A[0][0]*A[1][1]*A[2][2]-A[0][0]*A[1][2]*A[2][1]-A[1][0]*A[0][1]*A[2][2]+A[1][0]*A[0][2]*A[2][1]+A[2][0]*A[0][1]*A[1][2]-A[2][0]*A[0][2]*A[1][1]);

	B[0][0]=(A[1][1]*A[2][2]-A[1][2]*A[2][1])/det;
	B[0][1]=(-A[0][1]*A[2][2]+A[0][2]*A[2][1])/det;
	B[0][2]=(A[0][1]*A[1][2]-A[0][2]*A[1][1])/det;
	B[1][0]=(-A[1][0]*A[2][2]+A[1][2]*A[2][0])/det;
	B[1][1]=(A[0][0]*A[2][2]-A[0][2]*A[2][0])/det;
	B[1][2]=(-A[0][0]*A[1][2]+A[0][2]*A[1][0])/det;
	B[2][0]=(A[1][0]*A[2][1]-A[1][1]*A[2][0])/det;
	B[2][1]=(-A[0][0]*A[2][1]+A[0][1]*A[2][0])/det;
	B[2][2]=(A[0][0]*A[1][1]-A[0][1]*A[1][0])/det;
}//invert3

void invert2(matrix A, matrix B){
/* B = A^-1 for 2x2 matrices*/
	float det = (A[0][0]*A[1][1]-A[0][1]*A[1][0]);

	B[0][0]=(A[1][1])/det;
	B[0][1]=(-A[0][1])/det;
	B[1][0]=(-A[1][0])/det;
	B[1][1]=(A[0][0])/det;
}//invert2

float determinant(matrix A,matrix tmp, int n){
	/*returns determinant of n x n matrix*/
	int i,j,k;
	float q;
	float dtr=1;
	/* copy A to tmp*/
	for(i=0; i < n; i++)
		for(j=0; j < n; j++)
		    tmp[i][j] = A[j][i];

	/*compute determinant*/
	for(i=0;i < n;i++){
		for(j=0;j < n;j++){
			q=tmp[j][i]/tmp[i][i];
			for(k=0;k < n;k++){
				if(i==j) break;
				tmp[j][k]=(tmp[j][k]-tmp[i][k])*q;
			}//innermost for
		}//middle for
	}//outer for
	for(i=0;i < n;i++){
		dtr=dtr*tmp[i][i];
	}//for
	return(dtr);
}//det


void getMinor(matrix A, matrix B,int An, int m, int n){
	/*	use this function to generate a matrix B that is the mn minor of A; A must be square
	 *  m>=1, n>=1, An is the size of A*/
	int i,j;
	int k=0;
	for(i=0; i < An; i++){
		if(i!=(m-1)){
			for(j=0; j < (An-1); j++){
				if(j<(n-1)){
					B[k][j] = A[i][j];
				}
				else{
					B[k][j] = A[i][j+1];
				}
			}//inner for (column)
			k++;
		}//if
	}//outer for
}//get minor


void invt(matrix A, matrix B, matrix tmp1, matrix tmp2, int An){
	/*B =  A^-1 ; tmp1 & tmp2 must be at least of size (An-1)x(An-1)
	 * function does not check if matrix is singular, tried, but even if det(A)==0, it did not escape,
	 * just reutrned "nan" for some elements, and garbage for others
	 *  */
	int i,j;
	float detA = determinant(A,tmp2,An);
	for(i=0; i < An; i++){
		for(j=0; j < An; j++){
			getMinor(A,tmp1,An,i+1,j+1);
			if(((i+j)%2)==0){
				B[j][i]= determinant(tmp1,tmp2,(An-1))/detA;
			}
			else{
				B[j][i]= -determinant(tmp1,tmp2,(An-1))/detA;
			}
			 //problem if an element is zero (returns "nan" for zero element)
		}//inner for
	}//outer for
}//invert

float BhQhBhT(matrix Qh, float* Bh, float* tmp){
	//function returns the value for Bh*Qh*Bh', where Qh is an  3x3 matrix, and Bh is a vector of length 3



	//Bh*Qh
	int i,j;
	float sum;
	for(i=0; i < 3; i++){
		sum=0;
		for(j=0; j < 3; j++)
			sum += Bh[j]*Qh[j][i];
		tmp[i] = sum;
	 }//outer for

	//tmp*BhT
	sum = 0;
	for(i=0; i < 3; i++)
		sum += tmp[i]*Bh[i];



	return sum;
}

void VectdivSc(float* V, float scl){
	//function returns the value of Vector/scalar where vector is 3x1

	int i;
	for(i=0; i < 3; i++){
		V[i] = V[i]/scl;
	 }// for
}

#endif /*MATRICES_H_*/


