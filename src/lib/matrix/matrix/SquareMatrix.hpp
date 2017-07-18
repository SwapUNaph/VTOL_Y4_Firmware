/**
 * @file SquareMatrix.hpp
 *
 * A square matrix
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"
#include "helper_functions.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template <typename Type, size_t M>
class Vector;

template<typename Type, size_t  M>
class SquareMatrix : public Matrix<Type, M, M>
{
public:
    SquareMatrix() :
        Matrix<Type, M, M>()
    {
    }

    SquareMatrix(const Type *data_) :
        Matrix<Type, M, M>(data_)
    {
    }

    SquareMatrix(const Matrix<Type, M, M> &other) :
        Matrix<Type, M, M>(other)
    {
    }

    // inverse alias
    inline SquareMatrix<Type, M> I() const
    {
        SquareMatrix<Type, M> i;
        if(inv(*this, i)) {
            return i;
        } else {
            i.setZero();
            return i;
        }
    }


    // inverse alias
    inline bool I(SquareMatrix<Type, M> &i) const
    {
        return inv(*this, i);
    }


    Vector<Type, M> diag() const
    {
        Vector<Type, M> res;
        const SquareMatrix<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i, i);
        }
        return res;
    }

    Type trace() const
    {
        Type res = 0;
        const SquareMatrix<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res += self(i, i);
        }
        return res;
    }

};

typedef SquareMatrix<float, 3> SquareMatrix3f;

template<typename Type, size_t M>
SquareMatrix<Type, M> eye() {
    SquareMatrix<Type, M> m;
    m.setIdentity();
    return m;
}

template<typename Type, size_t M>
SquareMatrix<Type, M> diag(Vector<Type, M> d) {
    SquareMatrix<Type, M> m;
    for (size_t i=0; i<M; i++) {
        m(i,i) = d(i);
    }
    return m;
}

template<typename Type, size_t M>
SquareMatrix<Type, M> expm(const Matrix<Type, M, M> & A, size_t order=5)
{
    SquareMatrix<Type, M> res;
    SquareMatrix<Type, M> A_pow = A;
    res.setIdentity();
    size_t i_factorial = 1;
    for (size_t i=1; i<=order; i++) {
        i_factorial *= i;
        res += A_pow / Type(i_factorial);
        A_pow *= A_pow;
    }

    return res;
}


/**
 * inverse based on LU factorization with partial pivotting
 */
template<typename Type, size_t M>
bool inv(const SquareMatrix<Type, M> & A, SquareMatrix<Type, M> & inv)
{
    SquareMatrix<Type, M> L;
    L.setIdentity();
    SquareMatrix<Type, M> U = A;
    SquareMatrix<Type, M> P;
    P.setIdentity();

    //printf("A:\n"); A.print();

    // for all diagonal elements
    for (size_t n = 0; n < M; n++) {

        // if diagonal is zero, swap with row below
        if (fabs(static_cast<float>(U(n, n))) < 1e-8f) {
            //printf("trying pivot for row %d\n",n);
            for (size_t i = n + 1; i < M; i++) {

                //printf("\ttrying row %d\n",i);
                if (fabs(static_cast<float>(U(i, n))) > 1e-8f) {
                    //printf("swapped %d\n",i);
                    U.swapRows(i, n);
                    P.swapRows(i, n);
                    L.swapRows(i, n);
                    L.swapCols(i, n);
                    break;
                }
            }
        }

#ifdef MATRIX_ASSERT
        //printf("A:\n"); A.print();
        //printf("U:\n"); U.print();
        //printf("P:\n"); P.print();
        //fflush(stdout);
        //ASSERT(fabs(U(n, n)) > 1e-8f);
#endif

        // failsafe, return zero matrix
        if (fabs(static_cast<float>(U(n, n))) < 1e-8f) {
            return false;
        }

        // for all rows below diagonal
        for (size_t i = (n + 1); i < M; i++) {
            L(i, n) = U(i, n) / U(n, n);

            // add i-th row and n-th row
            // multiplied by: -a(i,n)/a(n,n)
            for (size_t k = n; k < M; k++) {
                U(i, k) -= L(i, n) * U(n, k);
            }
        }
    }

    //printf("L:\n"); L.print();
    //printf("U:\n"); U.print();

    // solve LY=P*I for Y by forward subst
    //SquareMatrix<Type, M> Y = P;

    // for all columns of Y
    for (size_t c = 0; c < M; c++) {
        // for all rows of L
        for (size_t i = 0; i < M; i++) {
            // for all columns of L
            for (size_t j = 0; j < i; j++) {
                // for all existing y
                // subtract the component they
                // contribute to the solution
                P(i, c) -= L(i, j) * P(j, c);
            }

            // divide by the factor
            // on current
            // term to be solved
            // Y(i,c) /= L(i,i);
            // but L(i,i) = 1.0
        }
    }

    //printf("Y:\n"); Y.print();

    // solve Ux=y for x by back subst
    //SquareMatrix<Type, M> X = Y;

    // for all columns of X
    for (size_t c = 0; c < M; c++) {
        // for all rows of U
        for (size_t k = 0; k < M; k++) {
            // have to go in reverse order
            size_t i = M - 1 - k;

            // for all columns of U
            for (size_t j = i + 1; j < M; j++) {
                // for all existing x
                // subtract the component they
                // contribute to the solution
                P(i, c) -= U(i, j) * P(j, c);
            }

            // divide by the factor
            // on current
            // term to be solved
            //
            // we know that U(i, i) != 0 from above
            P(i, c) /= U(i, i);
        }
    }

    //check sanity of results
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < M; j++) {
            if (!is_finite(P(i,j))) {
                return false;
            }
        }
    }
    //printf("X:\n"); X.print();
    inv = P;
    return true;
}

/**
 * inverse based on LU factorization with partial pivotting
 */
template<typename Type, size_t M>
SquareMatrix<Type, M> inv(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> i;
    if(inv(A, i)) {
        return i;
    } else {
        i.setZero();
        return i;
    }
}

/**
 * cholesky decomposition
 *
 * Note: A must be positive definite
 */
template<typename Type, size_t M>
SquareMatrix <Type, M> cholesky(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> L;
    for (size_t j = 0; j < M; j++) {
        for (size_t i = j; i < M; i++) {
            if (i==j) {
                float sum = 0;
                for (size_t k = 0; k < j; k++) {
                    sum += L(j, k)*L(j, k);
                }
                Type res = A(j, j) - sum;
                if (res <= 0) {
                    L(j, j) = 0;
                } else {
                    L(j, j) = sqrt(res);
                }
            } else {
                float sum = 0;
                for (size_t k = 0; k < j; k++) {
                    sum += L(i, k)*L(j, k);
                }
                if (L(j, j) <= 0) {
                    L(i, j) = 0;
                } else {
                    L(i, j) = (A(i, j) - sum)/L(j, j);
                }
            }
        }
    }
    return L;
}

/**
 * cholesky inverse
 *
 * TODO: Check if gaussian elimination jumps straight to back-substitution
 * for L or we need to do it manually. Will impact speed otherwise.
 */
template<typename Type, size_t M>
SquareMatrix <Type, M> choleskyInv(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> L_inv = inv(cholesky(A));
    return L_inv.T()*L_inv;
}

typedef SquareMatrix<float, 3> Matrix3f;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
