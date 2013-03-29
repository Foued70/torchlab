/* copy to C of Laurent Kneip's solveQuartic code.  Original copyright
   below. */

/*
 * Copyright (c) 2011, Laurent Kneip, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * P3p.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: Laurent Kneip
 * Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences
 *   Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *              Absolute Camera Position and Orientation
 *
 *       Input: featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
 *              worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
 *              solutions: 3x16 matrix that will contain the solutions
 *                         form: [ 3x1 position(solution1) 3x3 orientation(solution1) 3x1 position(solution2) 3x3 orientation(solution2) ... ]
 *                         the obtained orientation matrices are defined as transforming points from the cam to the world frame
 *      Output: int: 0 if correct execution
 *                  -1 if world points aligned
 */

#include <stdio.h>
#include <math.h>
#include <complex.h>

int computeFactors(double *factors, double *pointData) 
{
  // Definition of temporary variables for avoiding multiple computation

  double d_12 = pointData[0];
  double f_1  = pointData[1];
  double f_2  = pointData[2];
  double p_1  = pointData[3];
  double p_2  = pointData[4];
  double b    = pointData[5];

  double d_12_pw2 = pow(d_12,2);
  double f_1_pw2  = pow(f_1,2);
  double f_2_pw2  = pow(f_2,2);
  double p_1_pw2  = pow(p_1,2);
  double p_1_pw3  = p_1_pw2 * p_1;
  double p_1_pw4  = p_1_pw3 * p_1;
  double p_2_pw2  = pow(p_2,2);
  double p_2_pw3  = p_2_pw2 * p_2;
  double p_2_pw4  = p_2_pw3 * p_2;
  double b_pw2    = pow(b,2);

  printf("d_12: %f d_12^2: %f\n", d_12, d_12_pw2);
  printf("f_1: %f f_1^2: %f\n", f_1, f_1_pw2);
  printf("f_2: %f f_2^2: %f\n", f_2, f_2_pw2);
  printf("p_1: %f p_1^2: %f\n", p_1, p_1_pw2);
  printf("p_2: %f p_2^2: %f\n", p_2, p_2_pw2);
  printf("b: %f b^2: %f\n", b, b_pw2);

  // Computation of factors of 4th degree polynomial

  factors[0] = -f_2_pw2*p_2_pw4
    -p_2_pw4*f_1_pw2
    -p_2_pw4;

  factors[1] = 2*p_2_pw3*d_12*b
    +2*f_2_pw2*p_2_pw3*d_12*b
    -2*f_2*p_2_pw3*f_1*d_12;
  
  factors[2] = -f_2_pw2*p_2_pw2*p_1_pw2
    -f_2_pw2*p_2_pw2*d_12_pw2*b_pw2
    -f_2_pw2*p_2_pw2*d_12_pw2
    +f_2_pw2*p_2_pw4
    +p_2_pw4*f_1_pw2
    +2*p_1*p_2_pw2*d_12
    +2*f_1*f_2*p_1*p_2_pw2*d_12*b
    -p_2_pw2*p_1_pw2*f_1_pw2
    +2*p_1*p_2_pw2*f_2_pw2*d_12
    -p_2_pw2*d_12_pw2*b_pw2
    -2*p_1_pw2*p_2_pw2;

  factors[3] = 2*p_1_pw2*p_2*d_12*b
    +2*f_2*p_2_pw3*f_1*d_12
    -2*f_2_pw2*p_2_pw3*d_12*b
    -2*p_1*p_2*d_12_pw2*b;

  factors[4] = -2*f_2*p_2_pw2*f_1*p_1*d_12*b
    +f_2_pw2*p_2_pw2*d_12_pw2
    +2*p_1_pw3*d_12
    -p_1_pw2*d_12_pw2
    +f_2_pw2*p_2_pw2*p_1_pw2
    -p_1_pw4
    -2*f_2_pw2*p_2_pw2*p_1*d_12
    +p_2_pw2*f_1_pw2*p_1_pw2
    +f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;

  return 0;
}
int solveQuartic( double *realRoots, double *factors)
{
  double A = factors[0];
  double B = factors[1];
  double C = factors[2];
  double D = factors[3];
  double E = factors[4];

  double A_pw2 = A*A;
  double B_pw2 = B*B;
  double A_pw3 = A_pw2*A;
  double B_pw3 = B_pw2*B;
  double A_pw4 = A_pw3*A;
  double B_pw4 = B_pw3*B;

  double alpha = -3*B_pw2/(8*A_pw2)+C/A;
  double beta  =  B_pw3/(8*A_pw3)-B*C/(2*A_pw2)+D/A;
  double gamma = -3*B_pw4/(256*A_pw4)+B_pw2*C/(16*A_pw3)-B*D/(4*A_pw2)+E/A;

  double alpha_pw2 = alpha*alpha;
  double alpha_pw3 = alpha_pw2*alpha;

  double complex P = -alpha_pw2/12-gamma;
  double complex Q = -alpha_pw3/108+alpha*gamma/3-cpow(beta,2)/8;
  double complex R = -Q/2.0+csqrt(cpow(Q,2.0)/4.0+cpow(P,3.0)/27.0);

  double complex U = cpow(R,(1.0/3.0));
  double complex y;

  if (creal(U) == 0)
    y = -5.0*alpha/6.0-cpow(Q,(1.0/3.0));
  else
    y = -5.0*alpha/6.0-P/(3.0*U)+U;

  double complex w = csqrt(alpha+2.0*y);

  double complex temp;

  temp = -B/(4.0*A) + 0.5*(w+csqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
  realRoots[0] = creal(temp);
  temp = -B/(4.0*A) + 0.5*(w-csqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
  realRoots[1] = creal(temp);
  temp = -B/(4.0*A) + 0.5*(-w+csqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
  realRoots[2] = creal(temp);
  temp = -B/(4.0*A) + 0.5*(-w-csqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
  realRoots[3] = creal(temp);

  return 0;
}


int backSubstitute (double *solutions, double *pointData, double *rr)
{
  double d_12 = pointData[0];
  double f_1  = pointData[1];
  double f_2  = pointData[2];
  double p_1  = pointData[3];
  double p_2  = pointData[4];
  double b    = pointData[5];

  int i;  
  for(i=0; i<4; i++){

    double cot_alpha = (-f_1*p_1/f_2-rr[i]*p_2+d_12*b)/(-f_1*rr[i]*p_2/f_2+p_1-d_12);

    double cos_theta = rr[i];
    double sin_theta = sqrt(1-pow(rr[i],2));
    double sin_alpha = sqrt(1/(pow(cot_alpha,2)+1));
    double cos_alpha = sqrt(1-pow(sin_alpha,2));
    
    if (cot_alpha < 0)
      cos_alpha = -cos_alpha;

    // Translation 
    *solutions = d_12*cos_alpha*(sin_alpha*b+cos_alpha);
    solutions++;
    *solutions = cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha);
    solutions++;
    *solutions = sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha);
    // Rotation: R[0]
    *solutions = -cos_alpha;
    solutions++;
    *solutions = -sin_alpha*cos_theta; 
    solutions++;
    *solutions = -sin_alpha*sin_theta;
    solutions++;
    // Rotation: R[1] 
    *solutions = sin_alpha; 
    solutions++;
    *solutions = -cos_alpha*cos_theta;
    solutions++;
    *solutions = -cos_alpha*sin_theta;
    solutions++;
    // Rotation: R[2] 
    *solutions = 0;
    solutions++;
    *solutions = -sin_theta;
    solutions++;
    *solutions = cos_theta ;
    solutions++;

  }
  return 0;
}
