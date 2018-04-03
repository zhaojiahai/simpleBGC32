/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Constrain
///////////////////////////////////////////////////////////////////////////////

float constrain(float input, float minValue, float maxValue)
{
	if (input < minValue)
		return minValue;

	else if (input > maxValue)
		return maxValue;

	else
		return input;
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixMultiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, int16_t matrixC[], int16_t matrixA[], int16_t matrixB[])
{
	uint8_t i, j, k;

	for (i = 0; i < aRows * bCols; i++)
	{
		matrixC[i] = 0.0;
	}

	for (i = 0; i < aRows; i++)
	{
		for (j = 0; j < aCols_bRows; j++)
		{
			for (k = 0;  k < bCols; k++)
			{
				matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Round
///////////////////////////////////////////////////////////////////////////////

float Round(float x)
{
	if (x >= 0)
	{
		return x + 0.5F;
	}

	else
	{
		return x - 0.5F;
	}
}

///////////////////////////////////////////////////////////////////////////////
// _sbrk
///////////////////////////////////////////////////////////////////////////////

/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */

/*caddr_t*/int _sbrk(int incr)
{
	// extern char _ebss; // Defined by the linker
	static char *heap_end;
	//    char *prev_heap_end;

	char *stack;

	if (heap_end == 0) {}

	//   heap_end = &_ebss;

	// prev_heap_end = heap_end;

	stack = (char *) __get_MSP();

	if (heap_end + incr >  stack)
	{
		errno = ENOMEM;
		// return (caddr_t) - 1;
	}

	heap_end += incr;
	return 0;// (caddr_t) prev_heap_end;
}

///////////////////////////////////////////////////////////////////////////////
//  Least Squares Fit a Sphere to 3D Data
////////////////////////////////////////////////////////////////////////////////

// Least squares fit a sphere to 3D data, ImaginaryZ's blog,
// Miscellaneous banter, Useful mathematics, game programming
// tools and the occasional kink or two.
// 22 April 2011.
// http: imaginaryz.blogspot.com.au/2011/04/least-squares-fit-sphere-to-3d-data.html

// Substantially rewritten for UAVXArm by Prof. G.K. Egan (C) 2012.

// Incorporated into aq32Plus by J. Ihlein (C) 2012.

uint16_t sphereFit(float    d[][3],
                   uint16_t N,
                   uint16_t MaxIterations,
                   float    Err,
                   uint16_t Population[][3],
                   float    SphereOrigin[],
                   float     *SphereRadius)
{
	uint8_t  c;
	uint16_t i, Iterations;
	float    s[3], s2[3], s3[3], sum[3], sum2[3], sum3[3];
	float    x2sum[3], y2sum[3], z2sum[3];
	float    xy_sum, xz_sum, yz_sum;
	float    XY, XZ, YZ, X2Z, Y2X, Y2Z, Z2X, X2Y, Z2Y;
	float    QS, QB, Q0, Q1, Q2;
	float    R2, C[3], C2[3], Delta[3], Denom[3];
	float    F0, F1, F2, F3, F4;
	float    di2[3];
	float    SizeR;

	for (c = XAXIS; c <= ZAXIS; c++)
	{
		s[c] = s2[c] = s3[c] = sum[c] = x2sum[c] = y2sum[c] = z2sum[c] = 0.0f;

		Population[0][c] = Population[1][c] = 0;
	}

	xy_sum = xz_sum = yz_sum = 0.0f;

	for (i = 0; i < N; i++)
	{
		for (c = XAXIS; c <= ZAXIS; c++)
		{
			di2[c] = SQR(d[i][c]);

			s[c]  += d[i][c];
			s2[c] += di2[c];
			s3[c] += di2[c] * d[i][c];

			Population[d[i][c] > 0.0f][c]++;
		}

		xy_sum += d[i][XAXIS] * d[i][YAXIS];
		xz_sum += d[i][XAXIS] * d[i][ZAXIS];
		yz_sum += d[i][YAXIS] * d[i][ZAXIS];

		x2sum[YAXIS] += di2[XAXIS] * d[i][YAXIS];
		x2sum[ZAXIS] += di2[XAXIS] * d[i][ZAXIS];

		y2sum[XAXIS] += di2[YAXIS] * d[i][XAXIS];
		y2sum[ZAXIS] += di2[YAXIS] * d[i][ZAXIS];

		z2sum[XAXIS] += di2[ZAXIS] * d[i][XAXIS];
		z2sum[YAXIS] += di2[ZAXIS] * d[i][YAXIS];
	}

	SizeR = 1.0f / (float) N;

	for (c = XAXIS; c <= ZAXIS; c++)
	{
		sum[c]  = s[c]  * SizeR; //sum( X[n]   )
		sum2[c] = s2[c] * SizeR; //sum( X[n]^2 )
		sum3[c] = s3[c] * SizeR; //sum( X[n]^3 )
	}

	XY = xy_sum * SizeR;         //sum( X[n] * Y[n] )
	XZ = xz_sum * SizeR;         //sum( X[n] * Z[n] )
	YZ = yz_sum * SizeR;         //sum( Y[n] * Z[n] )

	X2Y = x2sum[YAXIS] * SizeR;  //sum( X[n]^2 * Y[n] )
	X2Z = x2sum[ZAXIS] * SizeR;  //sum( X[n]^2 * Z[n] )
	Y2X = y2sum[XAXIS] * SizeR;  //sum( Y[n]^2 * X[n] )
	Y2Z = y2sum[ZAXIS] * SizeR;  //sum( Y[n]^2 * Z[n] )
	Z2X = z2sum[XAXIS] * SizeR;  //sum( Z[n]^2 * X[n] )
	Z2Y = z2sum[YAXIS] * SizeR;  //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	F0 = sum2[XAXIS] + sum2[YAXIS] + sum2[ZAXIS];
	F1 = 0.5f * F0;
	F2 = -8.0f * (sum3[XAXIS] + Y2X + Z2X);
	F3 = -8.0f * (X2Y + sum3[YAXIS] + Z2Y);
	F4 = -8.0f * (X2Z + Y2Z + sum3[ZAXIS]);

	for (c = XAXIS; c <= ZAXIS; c++)
	{
		C[c]  = sum[c];
		C2[c] = SQR(C[c]);
	}

	QS = C2[XAXIS] + C2[YAXIS] + C2[ZAXIS];
	QB = -2.0f * (SQR(C[XAXIS]) + SQR(C[YAXIS]) + SQR(C[ZAXIS]));
	R2 = F0 + QB + QS;
	Q0 = 0.5f * (QS - R2);
	Q1 = F1 + Q0;
	Q2 = 8.0f * (QS - R2 + QB + F0);

	Iterations = 0;

	do
	{
		for (c = XAXIS; c <= ZAXIS; c++)
		{
			Denom[c] = Q2 + 16.0f * (C2[c] - 2.0f * C[c] * sum[c] + sum2[c]);

			if (Denom[c] == 0.0f)
				Denom[c] = 1.0f;
		}

		Delta[XAXIS] = -((F2 + 16.0f * (C[YAXIS] * XY + C[ZAXIS] * XZ + sum[XAXIS] * (-C2[XAXIS] - Q0)
		                                + C[XAXIS] * (sum2[XAXIS] + Q1 - C[ZAXIS] * sum[ZAXIS] - C[YAXIS] * sum[YAXIS]))) / Denom[XAXIS]);

		Delta[YAXIS] = -((F3 + 16.0f * (C[XAXIS] * XY + C[ZAXIS] * YZ + sum[YAXIS] * (-C2[YAXIS] - Q0)
		                                + C[YAXIS] * (sum2[YAXIS] + Q1 - C[XAXIS] * sum[XAXIS] - C[ZAXIS] * sum[ZAXIS]))) / Denom[YAXIS]);

		Delta[ZAXIS] = -((F4 + 16.0f * (C[XAXIS] * XZ + C[YAXIS] * YZ + sum[ZAXIS] * (-C2[ZAXIS] - Q0)
		                                + C[ZAXIS] * (sum2[ZAXIS] + Q1 - C[XAXIS] * sum[XAXIS] - C[YAXIS] * sum[YAXIS]))) / Denom[ZAXIS]);

		for (c = XAXIS; c <= ZAXIS; c++)
		{
			C[c] += Delta[c];
			C2[c] = SQR(C[c]);
		}

		QS = C2[XAXIS] + C2[YAXIS] + C2[ZAXIS];
		QB = -2.0f * (C[XAXIS] * sum[XAXIS] + C[YAXIS] * sum[YAXIS] + C[ZAXIS] * sum[ZAXIS]);
		R2 = F0 + QB + QS;
		Q0 = 0.5f * (QS - R2);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - R2 + QB + F0);

		Iterations++;
	}
	while ((Iterations < 50) || ((Iterations < MaxIterations) && ((SQR(Delta[XAXIS]) + SQR(Delta[YAXIS]) + SQR(Delta[ZAXIS])) > Err)));

	for (c = XAXIS; c <= ZAXIS; c++)
		SphereOrigin[c] = C[c];

	*SphereRadius = sqrt(R2);

	return (Iterations);
}

///////////////////////////////////////////////////////////////////////////////
//  Standard Radian Format Limiter
////////////////////////////////////////////////////////////////////////////////

float standardRadianFormat(float angle)
{
	if (angle >= PI)
		return (angle - 2 * PI);

	else if (angle < -PI)
		return (angle + 2 * PI);

	else
		return (angle);
}

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////

// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//

#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')

float stringToFloat(const uint8_t *p)
{
	int frac = 0;
	double sign, value, scale;

	// Skip leading white space, if any.

	while (white_space(*p))
	{
		p += 1;
	}

	// Get sign, if any.

	sign = 1.0;

	if (*p == '-')
	{
		sign = -1.0;
		p += 1;

	}

	else if (*p == '+')
	{
		p += 1;
	}

	// Get digits before decimal point or exponent, if any.

	value = 0.0;

	while (valid_digit(*p))
	{
		value = value * 10.0 + (*p - '0');
		p += 1;
	}

	// Get digits after decimal point, if any.

	if (*p == '.')
	{
		double pow10 = 10.0;
		p += 1;

		while (valid_digit(*p))
		{
			value += (*p - '0') / pow10;
			pow10 *= 10.0;
			p += 1;
		}
	}

	// Handle exponent, if any.

	scale = 1.0;

	if ((*p == 'e') || (*p == 'E'))
	{
		unsigned int expon;
		p += 1;

		// Get sign of exponent, if any.

		frac = 0;

		if (*p == '-')
		{
			frac = 1;
			p += 1;

		}

		else if (*p == '+')
		{
			p += 1;
		}

		// Get digits of exponent, if any.

		expon = 0;

		while (valid_digit(*p))
		{
			expon = expon * 10 + (*p - '0');
			p += 1;
		}

		if (expon > 308) expon = 308;

		// Calculate scaling factor.

		while (expon >= 37)
		{
			scale *= 1E37;
			expon -= 37;
		}

		while (expon >=  8)
		{
			scale *= 1E8;
			expon -=  8;
		}

		while (expon >   0)
		{
			scale *= 10.0;
			expon -=  1;
		}
	}

	// Return signed and scaled floating point result.

	return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
