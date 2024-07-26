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
#include <sofa/helper/YoungModulus.h>
#include <gtest/gtest.h>
#include <sofa/testing/TestMessageHandler.h>


namespace sofa
{

TEST(YoungModulus, setToLegitValue)
{
    // required to be able to use EXPECT_MSG_NOEMIT and EXPECT_MSG_EMIT
    sofa::helper::logging::MessageDispatcher::addHandler(sofa::testing::MainGtestMessageHandler::getInstance() ) ;

    EXPECT_MSG_NOEMIT(Warning) ;
    EXPECT_MSG_NOEMIT(Error) ;

    const helper::YoungModulus<double> y {100};

    EXPECT_DOUBLE_EQ(y, 100);
}

TEST(YoungModulus, setToIllicitValue)
{
    // required to be able to use EXPECT_MSG_NOEMIT and EXPECT_MSG_EMIT
    sofa::helper::logging::MessageDispatcher::addHandler(sofa::testing::MainGtestMessageHandler::getInstance() ) ;

    EXPECT_MSG_NOEMIT(Warning) ;
    EXPECT_MSG_EMIT(Error) ;

    const helper::YoungModulus<double> y {-123456};

    EXPECT_DOUBLE_EQ(y, 1000);
}

TEST(YoungModulus, setToZero)
{
    // required to be able to use EXPECT_MSG_NOEMIT and EXPECT_MSG_EMIT
    sofa::helper::logging::MessageDispatcher::addHandler(sofa::testing::MainGtestMessageHandler::getInstance() ) ;

    EXPECT_MSG_NOEMIT(Warning) ;
    EXPECT_MSG_NOEMIT(Error) ;

    const helper::YoungModulus<double> y {0};

    EXPECT_DOUBLE_EQ(y, 0);
}

TEST(YoungModulus, operationsWithOtherScalars)
{
    const helper::YoungModulus<double> y {1000};

    const auto result = 2.5 * y + 80;

    EXPECT_DOUBLE_EQ(result, 2580);
}

}
