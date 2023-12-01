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
#include <sofa/type/MatView.h>
#include <gtest/gtest.h>

namespace sofa
{

TEST(MatView, basic)
{
    const sofa::type::Mat<3, 3, SReal> mat{
        {1, 2, 3}, {4, 5, 6}, {7, 8, 9}
    };

    {
        auto matCopy = mat;
        auto view_3200 = sofa::type::makeSubMatView<3, 2, 0, 0>(&matCopy);
        static_assert(decltype(view_3200)::nbCols == 2);
        static_assert(decltype(view_3200)::nbLines == 3);

        EXPECT_EQ(view_3200(0, 0), mat(0, 0));

        view_3200.clear();

        EXPECT_EQ(view_3200(0, 0), 0);
    }

    {
        const auto view_2211 = sofa::type::makeSubMatView<2, 2, 1, 1>(&mat);
        static_assert(decltype(view_2211)::nbCols == 2);
        static_assert(decltype(view_2211)::nbLines == 2);

        EXPECT_EQ(view_2211(0, 0), mat(1, 1));
    }
}

TEST(MatView, sum)
{
    const sofa::type::Mat<3, 3, SReal> mat{
            {1, 2, 3}, {4, 5, 6}, {7, 8, 9}
    };
    const auto view_2211 = sofa::type::makeSubMatView<2, 2, 1, 1>(&mat);

    const auto sum = view_2211 + view_2211;

    EXPECT_EQ(sum(0, 0), 2 * mat(1, 1));
}

TEST(MatView, product)
{
    const sofa::type::Mat<3, 3, SReal> mat{
                {1, 2, 3}, {4, 5, 6}, {7, 8, 9}
    };
    const auto view_2211 = sofa::type::makeSubMatView<2, 2, 1, 1>(&mat);

    const auto product = view_2211 * type::Vec2{1, 2};
    EXPECT_EQ(product[0], 17);
    EXPECT_EQ(product[1], 26);
}

}
