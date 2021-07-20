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

#include <sofa/core/config.h>
#include <sofa/type/vector.h>
#include <sofa/type/fwd.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa::core
{

class SOFA_CORE_API MatrixAccumulatorInterface
{
public:
    virtual ~MatrixAccumulatorInterface() = default;

    virtual void add(sofa::SignedIndex /*row*/, sofa::SignedIndex /*col*/, float /*value*/) {}
    virtual void add(sofa::SignedIndex /*row*/, sofa::SignedIndex /*col*/, double /*value*/) {}

    virtual void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value)
    {
        for (sofa::SignedIndex i = 0; i < 3; ++i)
        {
            for (sofa::SignedIndex j = 0; j < 3; ++j)
            {
                add(row + i, col + j, value(i, j));
            }
        }
    }
    virtual void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value)
    {
        for (sofa::SignedIndex i = 0; i < 3; ++i)
        {
            for (sofa::SignedIndex j = 0; j < 3; ++j)
            {
                add(row + i, col + j, value(i, j));
            }
        }
    }

    virtual void clear() {}
};

namespace matrixaccumulator
{
class no_check_policy {};
inline constexpr no_check_policy no_check {};


struct IndexVerificationStrategy
{
    virtual ~IndexVerificationStrategy() = default;
    using verify_index = std::true_type;

    virtual void checkRowIndex(sofa::SignedIndex row) = 0;
    virtual void checkColIndex(sofa::SignedIndex col) = 0;
};

struct NoIndexVerification : IndexVerificationStrategy
{
    using verify_index = std::false_type;
private:
    void checkRowIndex(sofa::SignedIndex row) override {}
    void checkColIndex(sofa::SignedIndex col) override {}
};

struct RangeVerification : IndexVerificationStrategy
{
    using verify_index = std::true_type;

    sofa::SignedIndex minRowIndex { 0 };
    sofa::SignedIndex maxRowIndex { std::numeric_limits<sofa::SignedIndex>::max() };

    sofa::SignedIndex minColIndex { 0 };
    sofa::SignedIndex maxColIndex { std::numeric_limits<sofa::SignedIndex>::max() };

    sofa::core::objectmodel::BaseObject* m_messageComponent { nullptr };

    [[nodiscard]]
    helper::logging::MessageDispatcher::LoggerStream logger() const
    {
        return m_messageComponent
                ? msg_error(m_messageComponent)
                : msg_error("RangeVerification");
    }

    void checkRowIndex(sofa::SignedIndex row) override
    {
        if (row < minRowIndex)
        {
            logger() << "Trying to accumulate a matrix entry out of the allowed submatrix: minimum "
                        "row index is " << minRowIndex << " while " << row << " was provided";
        }
        if (row > maxRowIndex)
        {
            logger() << "Trying to accumulate a matrix entry out of the allowed submatrix: maximum "
                        "row index is " << maxRowIndex << " while " << row << " was provided";
        }
    }

    void checkColIndex(sofa::SignedIndex col) override
    {
        if (col < minColIndex)
        {
            logger() << "Trying to accumulate a matrix entry out of the allowed submatrix: minimum "
                        "column index is " << minColIndex << " while " << col << " was provided";
        }
        if (col > maxColIndex)
        {
            logger() << "Trying to accumulate a matrix entry out of the allowed submatrix: maximum "
                        "column index is " << maxColIndex << " while " << col << " was provided";
        }
    }
};

}

/**
 * Decorator allowing to check the row and column indices before the matrix accumulation
 */
template<class TBaseMatrixAccumulator, class TStrategy>
class MatrixAccumulatorIndexChecker : public TBaseMatrixAccumulator
{
public:
    static_assert(std::is_base_of_v<MatrixAccumulatorInterface, TBaseMatrixAccumulator>, "Template argument must be a MatrixAccumulatorInterface");
    static_assert(std::is_base_of_v<objectmodel::BaseObject, TBaseMatrixAccumulator>, "Template argument must be a BaseObject");
    static_assert(std::is_base_of_v<matrixaccumulator::IndexVerificationStrategy, TStrategy>, "Template argument must be a IndexVerificationStrategy");

    SOFA_ABSTRACT_CLASS(MatrixAccumulatorIndexChecker, TBaseMatrixAccumulator)

    [[maybe_unused]]
    std::shared_ptr<TStrategy> indexVerificationStrategy;

    void add(sofa::SignedIndex row, sofa::SignedIndex col, float value) override final
    {
        if constexpr (TStrategy::verify_index::value)
        {
            if (indexVerificationStrategy)
            {
                indexVerificationStrategy->checkRowIndex(row);
                indexVerificationStrategy->checkColIndex(row);
            }
        }
        add(matrixaccumulator::no_check, row, col, value);
    }

    void add(sofa::SignedIndex row, sofa::SignedIndex col, double value) override final
    {
        if constexpr (TStrategy::verify_index::value)
        {
            if (indexVerificationStrategy)
            {
                indexVerificationStrategy->checkRowIndex(row);
                indexVerificationStrategy->checkColIndex(row);
            }
        }
        add(matrixaccumulator::no_check, row, col, value);
    }

    void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value) override final
    {
        if constexpr (TStrategy::verify_index::value)
        {
            if (indexVerificationStrategy)
            {
                indexVerificationStrategy->checkRowIndex(row);
                indexVerificationStrategy->checkColIndex(row);
            }
        }
        add(matrixaccumulator::no_check, row, col, value);
    }

    void add(sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value) override final
    {
        if constexpr (TStrategy::verify_index::value)
        {
            if (indexVerificationStrategy)
            {
                indexVerificationStrategy->checkRowIndex(row);
                indexVerificationStrategy->checkColIndex(row);
            }
        }
        add(matrixaccumulator::no_check, row, col, value);
    }

protected:

    virtual void add(const matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, float value)
    {
        TBaseMatrixAccumulator::add(row, col, value);
    }
    virtual void add(const matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, double value)
    {
        TBaseMatrixAccumulator::add(row, col, value);
    }

    virtual void add(const matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value)
    {
        TBaseMatrixAccumulator::add(row, col, value);
    }
    virtual void add(const matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value)
    {
        TBaseMatrixAccumulator::add(row, col, value);
    }
};

/**
 * Composite class of MatrixAccumulatorInterface
 */
template<class TMatrixAccumulator>
class ListMatrixAccumulator : public TMatrixAccumulator
{
    static_assert(std::is_base_of_v<MatrixAccumulatorInterface, TMatrixAccumulator>, "Invalid template argument");
    using InternalListMatrixAccumulator = sofa::type::vector<TMatrixAccumulator*>;

public:
    void push_back(TMatrixAccumulator* m)
    {
        m_list.push_back(m);
    }

    void remove(TMatrixAccumulator* m)
    {
        m_list.erase(std::remove(m_list.begin(), m_list.end(), m), m_list.end());
    }

    [[nodiscard]]
    bool empty() const
    {
        return m_list.empty();
    }

    void clear()
    {
        for (auto* mat : m_list)
        {
            mat->clear();
        }
    }

    [[nodiscard]]
    typename InternalListMatrixAccumulator::size_type size() const
    {
        return m_list.size();
    }

    void add(sofa::SignedIndex i, sofa::SignedIndex j, float value) override
    {
        for (auto* mat : m_list)
        {
            mat->add(i, j, value);
        }
    }
    void add(sofa::SignedIndex i, sofa::SignedIndex j, double value) override
    {
        for (auto* mat : m_list)
        {
            mat->add(i, j, value);
        }
    }
    void add(sofa::SignedIndex i, sofa::SignedIndex j, const sofa::type::Mat3x3f& value) override
    {
        for (auto* mat : m_list)
        {
            mat->add(i, j, value);
        }
    }
    void add(sofa::SignedIndex i, sofa::SignedIndex j, const sofa::type::Mat3x3d& value) override
    {
        for (auto* mat : m_list)
        {
            mat->add(i, j, value);
        }
    }

    [[nodiscard]]
    const InternalListMatrixAccumulator& getAccumulators() const
    {
        return m_list;
    }

private:
    InternalListMatrixAccumulator m_list;
};

namespace matrixaccumulator
{
    /**
     * Provides member typedef type for known TComponentType using SFINAE
     *
     * Typedef type is an abstract strong type derived from MatrixAccumulator and depending on TComponentType
     */
    template<class TComponentType>
    struct get_abstract_strong
    {
        using ComponentType = TComponentType;
    };

    /// Helper alias
    template<class TComponentType>
    using get_abstract_strong_type = typename get_abstract_strong<TComponentType>::type;

    /**
     * Provides member typedef type for known TComponentType using SFINAE
     *
     * Typedef type is an abstract strong type derived from MatrixAccumulator and BaseObject, and depending on TComponentType
     */
    template<class TComponentType>
    struct get_base_object_strong
    {
        using ComponentType = TComponentType;
    };

    /// Helper alias
    template<class TComponentType>
    using get_base_object_strong_type = typename get_base_object_strong<TComponentType>::type;

    /**
     * Provides member typedef type for known TComponentType using SFINAE
     *
     * Typedef type is an abstract strong type derived from ListMatrixAccumulator and depending on TComponentType
     */
    template<class TComponentType>
    struct get_list_abstract_strong
    {
        using ComponentType = TComponentType;
    };

    template<class TComponentType>
    using get_list_abstract_strong_type = typename get_list_abstract_strong<TComponentType>::type;
}

} //namespace sofa::core
