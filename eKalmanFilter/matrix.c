/* 
 * Matrix inverse finding routines
 * (c) Srikanth Saripalli 
 * srik@robotics.usc.edu
 * 
 */

#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "utils.h"

int inverse(double A[MAXSIZE][MAXSIZE], double B[MAXSIZE][MAXSIZE], int n)
{
    double identCol[MAXSIZE];
    double ident[MAXSIZE][MAXSIZE];
    double L[MAXSIZE][MAXSIZE];
    double U[MAXSIZE][MAXSIZE];
    double invUcol[MAXSIZE];
    double invLcol[MAXSIZE];
    double invU[MAXSIZE][MAXSIZE];
    double invL[MAXSIZE][MAXSIZE];
    double detA;

    int i;
    if( n == 1)
        B[0][0] = 1.0/A[0][0];
    else if( n == 2 ){
        detA = A[0][0]*A[1][1] - A[0][1]*A[1][0];
        B[0][0] = A[1][1]/detA;
        B[0][1] = -A[0][1]/detA;
        B[1][0] = -A[1][0]/detA;
        B[1][1] = A[0][0]/detA;
    }
    else{
        /* construct an identity matrix */
        eye(ident,n);
        /* perform LU decomp on A */
        LU(A,L,U,n);
        for(i=0; i<n; ++i){
            Mcol(ident,identCol,i,n);   // separates the ith column
            solveupper(U,identCol,invUcol,n);
            solvelower(L,identCol,invLcol,n);
            Vcol(invUcol,invU,i,n);     // places invUcol in ith column of invU
            Vcol(invLcol,invL,i,n);     // places invLcol in ith column of invL
        }
        /* inv(A) = inv(U)*inv(L) */
        //matrix_multiply(invU,invL,B,n,n,n);
        mulNxM(B, invU, invL, n,n,n,0,0, 0);
    }
    return 0;
}



/* This will take column n from matrix A place in vector a
* A(:,n) = a(m,1)
* m = number of rows
*/

int Mcol(double A[MAXSIZE][MAXSIZE], double a[MAXSIZE], int n, int m)
{
    int i;
    for(i=0; i<m; ++i){
        a[i] = A[i][n];
    }
    return 0;
}


/*This will take vector a and place into column of matrix A
 * a(:,1) = A(m,n)
 * m = number of rows
 */

int Vcol(double a[MAXSIZE], double A[MAXSIZE][MAXSIZE], int n, int m)
{
    int i;
    for(i=0; i<m; ++i){
        A[i][n] = a[i];
    }
    return 0;
}

/* This will solve A*x = b, where matrix A is upper triangular
(n,n)*x(n,1) = b(n,1)
*/

int solveupper(double A[MAXSIZE][MAXSIZE], double b[MAXSIZE], 
                                            double x[MAXSIZE], int n)
{
    int p,i,j;
    p=n+1;
    for(i=1; i<=n; ++i){
        x[p-i-1] = b[p-i-1];
        for(j=(p+1-i); j<=n; ++j){
            x[p-i-1] = x[p-i-1] - A[p-i-1][j-1]*x[j-1];
        }
        x[p-i-1] = x[p-i-1]/A[p-i-1][p-i-1];
    }
    return 0;
}


/* This will solve A*x = b, where matrix A is lower triangular
 * A(n,n)*x(n,1) = b(n,1)
 * */

int solvelower(double A[MAXSIZE][MAXSIZE], double b[MAXSIZE], 
                                            double x[MAXSIZE], int n)
{
    int i,j;
    for(i=1; i<=n; ++i){
        x[i-1] = b[i-1];
        for(j=1; j<=i-1; ++j){
            x[i-1] = x[i-1] - A[i-1][j-1]*x[j-1];
        }
        x[i-1] = x[i-1]/A[i-1][i-1];
    }
    return 0;
}


/* This will perform LU decomp on matrix A return matrix L, matrix U
 * lu(A(n,n)) => L(n,n) and U(n,n)
 * */

int LU(double A[MAXSIZE][MAXSIZE], double L[MAXSIZE][MAXSIZE], 
       double U[MAXSIZE][MAXSIZE], int n)
{
    int k,i,j;
    double Acopy[MAXSIZE][MAXSIZE];
    /* copy A matrix */
    for(i=0; i<n; ++i){
        for(j=0; j<n; ++j){
            Acopy[i][j] = A[i][j];
        }
    }
    for(k=0; k<n-1; ++k){
        for(i=k+1; i<n; ++i){
            Acopy[i][k] = Acopy[i][k]/Acopy[k][k];
            for(j=k+1; j<n; ++j){
                Acopy[i][j] = Acopy[i][j] - Acopy[i][k]*Acopy[k][j];
            }
        }
    }
    /* make an identity matrix */
    eye(L,n);
    /* separate the L matrix */
    for(j=0; j<n-1; ++j){
        for(i=j+1; i<n; ++i){
            L[i][j] = Acopy[i][j];
        }
    }
    /* separate out the U matrix */
    matrix_init(U,n,n);
    for(i=0; i<n; ++i){
        for(j=i; j<n; ++j){
            U[i][j] = Acopy[i][j];
        }
    }
    return 0;
}



/* This will zero out the matrix A
 * zeros(n,m) = A
 * */

int matrix_init(double A[MAXSIZE][MAXSIZE], int n, int m)
{
    int i,j;
    for(i=0; i<n; ++i){
        for(j=0; j<m; ++j){
            A[i][j] = 0.0;
        }
    }
    return 0;
}


/* This will generate an identity matrix I
 * eye(n) = A(n,n)
 * */

int eye(double I[MAXSIZE][MAXSIZE], int n)
{
    int i;
    matrix_init(I,n,n);
    for(i=0; i<n; ++i){
        I[i][i] = 1.0;
    }
    return 0;
}



/* Prints an nxm matrix*/
int matrix_print(double A[MAXSIZE][MAXSIZE], int n, int m)
{
    int i,j;
    for(i = 0; i < n; i++){
    	for(j = 0; j < m; j++){
	    fprintf(stderr, "%7.5f ", A[i][j]);
	}
	fprintf(stderr,"\n");
    }			
    fprintf(stderr, "\n");
    return 0;
}

/* Prints a vector nx1 */
int vector_print(double A[MAXSIZE], int n)
{
    int i;
    for(i = 0; i < n; i++){
	fprintf(stderr, "%7.5f ", A[i]);
    }			
    fprintf(stderr, "\n");
    return 0;
}
