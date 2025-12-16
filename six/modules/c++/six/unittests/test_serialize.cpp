/* =========================================================================
* This file is part of six-c++
* =========================================================================
*
* (C) Copyright 2004 - 2018, MDA Information Systems LLC
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
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <six/Serialize.h>
#include <catch2/catch_test_macros.hpp>

template<typename T> T getRandomScalar()
{
    return static_cast<T>(rand() / static_cast<T>(RAND_MAX));
}

template<> int getRandomScalar<int>()
{
    return static_cast<int>(rand());
}

template<> size_t getRandomScalar<size_t>()
{
    return static_cast<size_t>(rand());
}

template<> sys::byte getRandomScalar<sys::byte>()
{
    return static_cast<sys::byte>(rand());
}

template<typename T> std::vector<T> getRandomVector(size_t length)
{
    std::vector<T> values(length);
    for (size_t ii = 0; ii < length; ++ii)
    {
        values[ii] = getRandomScalar<T>();
    }
    return values;
}

template<typename T>
bool testScalar(bool byteSwap)
{
    const T val = getRandomScalar<T>();
    std::vector<sys::byte> serializedData;
    six::serialize<T>(val, byteSwap, serializedData);
    const sys::byte* buffer = serializedData.data();
    T valCopy;
    six::deserialize<T>(buffer, byteSwap, valCopy);
    return val == valCopy;
}

template<typename T>
bool testVector(size_t length, bool byteSwap)
{
    const std::vector<T> val = getRandomVector<T>(length);
    std::vector<sys::byte> serializedData;
    six::serialize<std::vector<T> >(val, byteSwap, serializedData);
    const sys::byte* buffer = serializedData.data();
    std::vector<T> valCopy;
    six::deserialize<std::vector<T> >(buffer, byteSwap, valCopy);
    return val == valCopy;
}

bool testString(const std::string& str, bool byteSwap)
{
    std::vector<sys::byte> serializedData;
    six::serialize<std::string>(str, byteSwap, serializedData);

    const sys::byte* buffer = serializedData.data();
    std::string strCopy;
    six::deserialize<std::string>(buffer, byteSwap, strCopy);
    return str == strCopy;
}

TEST_CASE("ScalarSerialize")
{
    // Test with no byte swapping
    CHECK(testScalar<int>(false));
    CHECK(testScalar<size_t>(false));
    CHECK(testScalar<sys::byte>(false));
    CHECK(testScalar<float>(false));
    CHECK(testScalar<double>(false));

    // Test with byte swapping (do not test sys::byte)
    CHECK(testScalar<int>(true));
    CHECK(testScalar<size_t>(true));
    CHECK(testScalar<float>(true));
    CHECK(testScalar<double>(true));
}

TEST_CASE("StringSerialize")
{
    const std::string strNonEmpty = "TEST_STRING";
    CHECK(testString(strNonEmpty, false));
    CHECK(testString(strNonEmpty, true));

    const std::string strEmpty = "";
    CHECK(testString(strEmpty, false));
    CHECK(testString(strEmpty, true));
}

TEST_CASE("VectorSerialize")
{
    const size_t length = 213;

    // Test with no byte swapping
    CHECK(testVector<int>(length, false));
    CHECK(testVector<size_t>(length, false));
    CHECK(testVector<float>(length, false));
    CHECK(testVector<double>(length, false));
    CHECK(testVector<sys::byte>(length, false));

    // Test with byte swapping (do not test sys::byte)
    CHECK(testVector<int>(length, true));
    CHECK(testVector<size_t>(length, true));
    CHECK(testVector<float>(length, true));
    CHECK(testVector<double>(length, true));
}

