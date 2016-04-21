/* Matrix Library
 * (c) Srikanth Saripalli
 * srik@robotics.usc.edu
 */

#ifndef _MATRIX_H_
#define _MATRIX_H_

#define MAXSIZE 6

int inverse(double A[MAXSIZE][MAXSIZE], double B[MAXSIZE][MAXSIZE], int n);

int Mcol(double A[MAXSIZE][MAXSIZE], double a[MAXSIZE], int n, int m);

int Vcol(double a[MAXSIZE], double A[MAXSIZE][MAXSIZE], int n, int m);

int solveupper(double A[MAXSIZE][MAXSIZE], double b[MAXSIZE], 
	       double x[MAXSIZE], int n);

int solvelower(double A[MAXSIZE][MAXSIZE], double b[MAXSIZE], 
	       double x[MAXSIZE], int n);

int LU(double A[MAXSIZE][MAXSIZE], double L[MAXSIZE][MAXSIZE], 
       double U[MAXSIZE][MAXSIZE], int n);

int matrix_init(double A[MAXSIZE][MAXSIZE], int n, int m);

int eye(double I[MAXSIZE][MAXSIZE], int n);

int matrix_print(double A[MAXSIZE][MAXSIZE], int n, int m);

int vector_print(double A[MAXSIZE], int n);
#endif

