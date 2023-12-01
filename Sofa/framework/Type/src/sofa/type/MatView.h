/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/type/Mat.h>

namespace sofa::type
{

template <sofa::Size SubSizeL, sofa::Size SubSizeC, sofa::Size OffsetL, sofa::Size OffsetC, class TMatrix>
class SubMatView final
{
public:

    // The type of the original matrix
    using Matrix = TMatrix;

    static constexpr Size nbInitialLines = Matrix::nbLines;
    static constexpr Size nbInitialCols = Matrix::nbCols;

    static constexpr Size nbLines = SubSizeL;
    static constexpr Size nbCols  = SubSizeC;

    static_assert(OffsetL + nbLines <= nbInitialLines);
    static_assert(OffsetC + nbCols <= nbInitialCols);

    static constexpr sofa::Size N = nbLines * nbCols;

    using Real = typename Matrix::Real;
    using Line = Vec<nbCols, Real>;
    using LineNoInit = VecNoInit<nbCols, Real>;
    using Col = Vec<nbLines, Real>;

    explicit SubMatView(Matrix* matrixPointer)
        : m_ptr(matrixPointer)
    {}

    SubMatView() = delete;

    /// number of lines
    Size getNbLines() const
    {
        return nbLines;
    }

    /// number of colums
    Size getNbCols() const
    {
        return nbCols;
    }

    void clear() noexcept
    {
        static_assert(!std::is_const_v<TMatrix>, "The initial matrix is const");
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                operator()(i, j) = {};
            }
        }
    }

    void fill(Real r) noexcept
    {
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                operator()(i, j) = r;
            }
        }
    }

    void identity() noexcept
    {
        static_assert(!std::is_const_v<TMatrix>, "The initial matrix is const");
        clear();
        for (Size i = 0; i < nbLines; ++i)
        {
            operator()(i, i) = 1;
        }
    }

    Real& operator()(Size i, Size j) noexcept
    {
        static_assert(!std::is_const_v<TMatrix>, "The initial matrix is const");
        return m_ptr->operator()(i + OffsetL, j + OffsetC);
    }

    const Real& operator()(Size i, Size j) const noexcept
    {
        return m_ptr->operator()(i + OffsetL, j + OffsetC);
    }

    [[nodiscard]] bool isSymmetric() const
    {
        if constexpr (nbLines == nbCols)
        {
            for (Size i = 0; i < nbLines; i++)
            {
                for (Size j = i + 1; j < nbCols; j++)
                {
                    if (rabs(operator()(i, j) - operator()(j, i)) > EQUALITY_THRESHOLD)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    [[nodiscard]] bool isDiagonal() const noexcept
    {
        for (Size i = 0; i < nbLines; i++)
        {
            for (Size j = 0; j < i - 1; j++)
            {
                if (rabs(operator()(i, j)) > EQUALITY_THRESHOLD)
                {
                    return false;
                }
            }
            for (Size j = i + 1; j < nbCols; j++)
            {
                if (rabs(operator()(i, j)) > EQUALITY_THRESHOLD)
                {
                    return false;
                }
            }
        }
        return true;
    }

    Mat<nbLines, nbCols, Real> operator+(const Mat<nbLines, nbCols, Real>& m) const noexcept
    {
        Mat<nbLines, nbCols, Real> r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r(i, j) = operator()(i, j) + m(i, j);
            }
        }
        return r;
    }

    template <sofa::Size OffsetL2, sofa::Size OffsetC2, class TMatrix2>
    Mat<nbLines, nbCols, Real> operator+(const SubMatView<SubSizeL, SubSizeC, OffsetL2, OffsetC2, TMatrix2>& m) const noexcept
    {
        Mat<nbLines, nbCols, Real> r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r(i, j) = operator()(i, j) + m(i, j);
            }
        }
        return r;
    }

    Mat<nbLines, nbCols, Real> operator-(const Mat<nbLines, nbCols, Real>& m) const noexcept
    {
        Mat<nbLines, nbCols, Real> r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r(i, j) = operator()(i, j) - m(i, j);
            }
        }
        return r;
    }

    template <sofa::Size OffsetL2, sofa::Size OffsetC2, class TMatrix2>
    Mat<nbLines, nbCols, Real> operator-(const SubMatView<SubSizeL, SubSizeC, OffsetL2, OffsetC2, TMatrix2>& m) const noexcept
    {
        Mat<nbLines, nbCols, Real> r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r(i, j) = operator()(i, j) - m(i, j);
            }
        }
        return r;
    }

    Mat<nbLines, nbCols, Real> operator-() const noexcept
    {
        Mat<nbLines, nbCols, Real> r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r(i, j) = -operator()(i, j);
            }
        }
        return r;
    }

    Col operator*(const Line& v) const noexcept
    {
        Col r(NOINIT);
        for (Size i = 0; i < nbLines; i++)
        {
            r[i] = operator()(i, 0) * v[0];
            for (Size j = 1; j < nbCols; j++)
            {
                r[i] += operator()(i, j) * v[j];
            }
        }
        return r;
    }

    Matrix operator*(Real f) const noexcept
    {
        Matrix r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r[i][j] = operator()(i, j) * f;
            }
        }
        return r;
    }

    Matrix operator/(Real f) const noexcept
    {
        Matrix r(NOINIT);
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                r[i][j] = operator()(i, j) / f;
            }
        }
        return r;
    }

    Matrix& operator*=(Real f) noexcept
    {
        static_assert(!std::is_const_v<TMatrix>, "The initial matrix is const");
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                operator()(i, j) *= f;
            }
        }
        return *this;
    }

    Matrix& operator/=(Real f) noexcept
    {
        static_assert(!std::is_const_v<TMatrix>, "The initial matrix is const");
        for (Size i = 0; i < nbLines; ++i)
        {
            for (Size j = 0; j < nbCols; ++j)
            {
                operator()(i, j) /= f;
            }
        }
        return *this;
    }

private:

    Matrix* m_ptr { nullptr };

    sofa::Size m_offsetRow {};
    sofa::Size m_offsetColumn {};
};

template <class TMatrix>
SubMatView(TMatrix*) -> SubMatView<TMatrix::nbLines, TMatrix::nbCols, 0, 0, TMatrix>;

template <sofa::Size SubSizeL, sofa::Size SubSizeC, sofa::Size OffsetL, sofa::Size OffsetC, class TMatrix>
SubMatView<SubSizeL, SubSizeC, OffsetL, OffsetC, TMatrix> makeSubMatView(TMatrix* matrix)
{
    return SubMatView<SubSizeL, SubSizeC, OffsetL, OffsetC, TMatrix>{matrix};
}

}
