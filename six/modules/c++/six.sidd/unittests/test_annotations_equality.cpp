/* =========================================================================
* This file is part of six.sidd-c++
* =========================================================================
*
* (C) Copyright 2004 - 2016, MDA Information Systems LLC
*
* six-c++ is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this program; If not,
* see <http://www.gnu.org/licenses/>.
*
*/

#include <iostream>

#include <six/sidd/Annotations.h>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("SFAPoint")
{
    six::sidd::SFATyped* pointTyped = new six::sidd::SFAPoint(1, 2, 3, 4);
    six::sidd::SFATyped* secondPointTyped = new six::sidd::SFAPoint(1, 2, 3, 4);

    six::sidd::SFAPoint rawPointOne(1, 2, 3, 4);
    six::sidd::SFAPoint rawPointTwo(1, 2, 3, 4);

    six::sidd::SFAPoint* pointPointerOne = new six::sidd::SFAPoint(1, 2, 3, 4);
    six::sidd::SFAPoint* pointPointerTwo = new six::sidd::SFAPoint(1, 2, 3, 4);

    CHECK(*pointTyped == *secondPointTyped);
    CHECK(rawPointOne == rawPointTwo);
    CHECK(rawPointOne == static_cast<six::sidd::SFATyped&>(rawPointTwo));
    CHECK(static_cast<six::sidd::SFATyped&>(rawPointOne) == rawPointTwo);
    CHECK(*pointPointerOne == *pointPointerTwo);
    CHECK(*pointTyped == rawPointTwo);
    CHECK(*pointTyped == *pointPointerOne);
    CHECK(rawPointOne == *pointPointerOne);

    six::sidd::SFAPoint different(2, 4, 6, 8);
    CHECK(different != *pointTyped);
    CHECK(different != rawPointOne);
    CHECK(different != *pointPointerOne);
}

TEST_CASE("SFALineString")
{
    six::sidd::SFALineString stringOne;
    stringOne.vertices.resize(2);
    stringOne.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    stringOne.vertices[1].reset(new six::sidd::SFAPoint(2, 4, 6, 8));

    six::sidd::SFALineString stringTwo;
    stringTwo.vertices.resize(2);
    stringTwo.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    stringTwo.vertices[1].reset(new six::sidd::SFAPoint(3, 4, 5, 6));


    six::sidd::SFALineString stringThree;
    stringThree.vertices.resize(2);
    stringThree.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    stringThree.vertices[1].reset(new six::sidd::SFAPoint(2, 4, 6, 8));

    CHECK(stringOne == stringThree);
    CHECK(stringOne != stringTwo);
}

TEST_CASE("SFALine")
{
    six::sidd::SFALine lineOne;
    lineOne.vertices.resize(2);
    lineOne.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    lineOne.vertices[1].reset(new six::sidd::SFAPoint(2, 4, 6, 8));

    six::sidd::SFALine lineTwo;
    lineTwo.vertices.resize(2);
    lineTwo.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    lineTwo.vertices[1].reset(new six::sidd::SFAPoint(3, 4, 5, 6));
    CHECK(lineOne != lineTwo);

    six::sidd::SFALine lineThree;
    lineThree.vertices.resize(2);
    lineThree.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    lineThree.vertices[1].reset(new six::sidd::SFAPoint(2, 4, 6, 8));
    CHECK(lineOne == lineThree);

    // Make sure different types aren't equal
    six::sidd::SFALineString stringOne;
    stringOne.vertices.resize(2);
    stringOne.vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    stringOne.vertices[1].reset(new six::sidd::SFAPoint(2, 4, 6, 8));
    CHECK(lineOne != stringOne);

    // Make sure pointers to different types aren't equal
    six::sidd::SFALineString* stringTwo = new six::sidd::SFALineString();
    stringTwo->vertices.resize(2);
    stringTwo->vertices[0].reset(new six::sidd::SFAPoint(1, 2, 3, 4));
    stringTwo->vertices[1].reset(new six::sidd::SFAPoint(2, 4, 6, 8));
    CHECK(lineOne != *stringTwo);
}
