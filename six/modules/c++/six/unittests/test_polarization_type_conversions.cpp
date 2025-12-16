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

#include <vector>

#include <import/str.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#define TEST_ASSERT_EQ(X, Y) CHECK(X == Y);
#define TEST_ASSERT_ALMOST_EQ(X, Y) CHECK_THAT(X, Catch::Matchers::WithinAbs(Y, 0.0001));

#include <six/Enums.h>
#include <six/Utilities.h>

template<typename TSixEnum>
static void test_EnumConstructor(const std::string& strType, TSixEnum type)
{
    const auto pType = TSixEnum::toType(strType);
    CHECK(pType.toString() == strType);
    TEST_ASSERT_EQ(pType, type);
}
TEST_CASE("EnumConstructor")
{
    test_EnumConstructor<six::DualPolarizationType>("OTHER", six::DualPolarizationType::OTHER);
    test_EnumConstructor<six::DualPolarizationType>("UNKNOWN", six::DualPolarizationType::UNKNOWN);
    test_EnumConstructor<six::DualPolarizationType>("V_V", six::DualPolarizationType::V_V);
    test_EnumConstructor<six::DualPolarizationType>("E_V", six::DualPolarizationType::E_V); // SICD 1.3

    test_EnumConstructor<six::PolarizationType>("OTHER", six::PolarizationType::OTHER);
    test_EnumConstructor<six::PolarizationType>("UNKNOWN", six::PolarizationType::UNKNOWN);
    test_EnumConstructor<six::PolarizationType>("X", six::PolarizationType::X); // SICD 1.3

    test_EnumConstructor<six::PolarizationSequenceType>("OTHER", six::PolarizationSequenceType::OTHER);
    test_EnumConstructor<six::PolarizationSequenceType>("UNKNOWN", six::PolarizationSequenceType::UNKNOWN);
    test_EnumConstructor<six::PolarizationSequenceType>("SEQUENCE", six::PolarizationSequenceType::SEQUENCE);
    test_EnumConstructor<six::PolarizationSequenceType>("X", six::PolarizationSequenceType::X); // SICD 1.3
}

template<typename TSixEnum>
static void test_toType_(const std::string& strType, TSixEnum type)
{
    const auto fromToType = TSixEnum::toType(strType);
    TEST_ASSERT_EQ(fromToType, type);
    auto str = fromToType.toString();
    TEST_ASSERT_EQ(str, strType);

    const auto fromCtor = TSixEnum::toType(strType);
    TEST_ASSERT_EQ(fromToType, fromCtor);
    str = fromCtor.toString();
    TEST_ASSERT_EQ(str, strType);
}
template<typename TSixEnum>
static void test_toType(size_t sz)
{
    test_toType_<TSixEnum>("UNKNOWN", TSixEnum::UNKNOWN);
    test_toType_<TSixEnum>("OTHER", TSixEnum::OTHER);

    auto&& map = TSixEnum::string_to_value_();
    TEST_ASSERT_EQ(map.size(), sz);
    for (auto&& kv : map)
    {
        const TSixEnum fromInt(kv.second);
        const auto toType = TSixEnum::toType(kv.first);
        TEST_ASSERT_EQ(toType, fromInt);

        if (fromInt != TSixEnum::NOT_SET)
        {
            test_toType_<TSixEnum >(kv.first, fromInt);
        }
    }
}
TEST_CASE("ToType")
{
    test_toType_<six::DualPolarizationType>("V_V", six::DualPolarizationType::V_V);
    test_toType_<six::DualPolarizationType>("E_V", six::DualPolarizationType::E_V); // SICD 1.3
    test_toType<six::DualPolarizationType>(85);

    test_toType_<six::PolarizationType>("X", six::PolarizationType::X);  // SICD 1.3
    test_toType<six::PolarizationType>(12);

    test_toType_<six::PolarizationSequenceType>("X", six::PolarizationSequenceType::X);  // SICD 1.3
    test_toType_<six::PolarizationSequenceType>("SEQUENCE", six::PolarizationSequenceType::SEQUENCE);
    test_toType<six::PolarizationSequenceType>(13);
}

template<typename TSixEnum>
static void test_toType_OTHER()
{
    const TSixEnum not_set;
    TEST_ASSERT_EQ(not_set, TSixEnum::NOT_SET);

    auto fromToType = TSixEnum::toType("OTHER");
    TEST_ASSERT_EQ(fromToType, TSixEnum::OTHER);

    CHECK_THROWS(TSixEnum::toType("OTHER:abc"));
}
TEST_CASE("ToType_OTHER")
{
    test_toType_OTHER<six::PolarizationType>();
    test_toType_OTHER<six::PolarizationSequenceType>();
}
TEST_CASE("DualPolarizationType_ToType_OTHER")
{
    test_toType_OTHER<six::DualPolarizationType>();

    auto toTypeDual = six::DualPolarizationType::toType("V_OTHER");
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::V_OTHER);
    toTypeDual = six::DualPolarizationType::toType("OTHER_V");
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_V);
    toTypeDual = six::DualPolarizationType::toType("OTHER_OTHER");
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_OTHER);

    CHECK_THROWS(six::DualPolarizationType::toType("OTHER_xyz")); // no "top level" OTHER.*
}

template<typename TSixEnum>
static void test_six_toType_(const std::string& strType, TSixEnum type)
{
    const auto fromToType = six::toType<TSixEnum>(strType);
    TEST_ASSERT_EQ(fromToType, type);
    const auto str = six::toString<TSixEnum>(fromToType);
    TEST_ASSERT_EQ(str, strType);
    TEST_ASSERT_EQ(str, six::toString(fromToType)); // no template parameter
}
template<typename TSixEnum>
static void test_six_toType(size_t sz)
{
    test_six_toType_<TSixEnum>("OTHER", TSixEnum::OTHER);

    auto&& map = TSixEnum::string_to_value_();
    TEST_ASSERT_EQ(map.size(), sz);
    for (auto&& kv : map)
    {
        const TSixEnum fromInt(kv.second);
        if (fromInt != TSixEnum::NOT_SET)
        {
            auto s = kv.first;
            str::replace(s, "_", ":");
            const auto toType = six::toType<TSixEnum>(s);
            TEST_ASSERT_EQ(toType, fromInt);

            test_six_toType_<TSixEnum>(s, fromInt);
        }
    }
}
TEST_CASE("SixToType")
{
    test_six_toType_<six::DualPolarizationType>("V:V", six::DualPolarizationType::V_V);
    test_six_toType_<six::DualPolarizationType>("E:V", six::DualPolarizationType::E_V); // SICD 1.3
    test_six_toType_<six::DualPolarizationType>("UNKNOWN", six::DualPolarizationType::UNKNOWN);
    test_six_toType<six::DualPolarizationType>(85);

    test_six_toType_<six::PolarizationType>("X", six::PolarizationType::X);  // SICD 1.3
    test_six_toType_<six::PolarizationType>("OTHER_abc", six::PolarizationType::OTHER); // SIDD 3.0/SICD 1.3
    test_six_toType<six::PolarizationType>(12);

    test_six_toType_<six::PolarizationSequenceType>("X", six::PolarizationSequenceType::X);  // SICD 1.3
    test_six_toType_<six::PolarizationSequenceType>("SEQUENCE", six::PolarizationSequenceType::SEQUENCE);
    test_six_toType_<six::PolarizationSequenceType>("UNKNOWN", six::PolarizationSequenceType::UNKNOWN);
    test_six_toType_<six::PolarizationSequenceType>("OTHER_abc", six::PolarizationSequenceType::OTHER); // SIDD 3.0/SICD 1.3
    test_six_toType<six::PolarizationSequenceType>(13);

    auto toTypeDual = six::DualPolarizationType::toType("V_OTHER");
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::V_OTHER);
    toTypeDual = six::toType<six::DualPolarizationType>("V:OTHER_xyz"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::V_OTHER);
    toTypeDual = six::DualPolarizationType::toType("OTHER_V");
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_V);
    toTypeDual = six::toType<six::DualPolarizationType>("OTHER_xyz:V"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_V);
    toTypeDual = six::toType<six::DualPolarizationType>("OTHER_abc:OTHER"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_OTHER);
    toTypeDual = six::toType<six::DualPolarizationType>("OTHER:OTHER_xyz"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_OTHER);
    toTypeDual = six::toType<six::DualPolarizationType>("OTHER_abc:OTHER_xyz"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toTypeDual, six::DualPolarizationType::OTHER_OTHER);
}

template<typename TSixEnum>
static void test_ToString_(const std::string& strType, TSixEnum type)
{
    {
        const auto polarizationString = type.toString();
        TEST_ASSERT_EQ(strType, polarizationString);
        auto pType = TSixEnum::toType(polarizationString);
        TEST_ASSERT_EQ(pType, type);
    }
    {
        const auto polarizationString = type.toString();
        TEST_ASSERT_EQ(strType, polarizationString);
        auto pType = TSixEnum::toType(polarizationString);
        TEST_ASSERT_EQ(pType, type);
    }
}
template<typename TSixEnum>
static void test_ToString()
{
    test_ToString_<TSixEnum>("UNKNOWN", TSixEnum::UNKNOWN);
    test_ToString_<TSixEnum>("OTHER", TSixEnum::OTHER);
}
TEST_CASE("ToString")
{
    test_ToString_<six::DualPolarizationType>("V_V", six::DualPolarizationType::V_V);
    test_ToString_<six::DualPolarizationType>("E_V", six::DualPolarizationType::E_V); // SICD 1.3
    test_ToString<six::DualPolarizationType>();

    test_ToString_<six::PolarizationType>("V", six::PolarizationType::V);
    test_ToString_<six::PolarizationType>("X", six::PolarizationType::X); // SICD 1.3
    test_ToString<six::PolarizationType>();

    test_ToString_<six::PolarizationSequenceType>("V", six::PolarizationSequenceType::V);
    test_ToString_<six::PolarizationSequenceType>("X", six::PolarizationSequenceType::X); // SICD 1.3
    test_ToString_<six::PolarizationSequenceType>("SEQUENCE", six::PolarizationSequenceType::SEQUENCE);
    test_ToString<six::PolarizationSequenceType>();
}

TEST_CASE("DualPolarizationType_ToString_OTHER")
{
    const six::DualPolarizationType not_set;
    TEST_ASSERT_EQ(not_set, six::DualPolarizationType::NOT_SET);
    TEST_ASSERT_EQ("NOT_SET", not_set.toString(false /*throw_if_not_set*/));
    CHECK_THROWS(not_set.toString(true /*throw_if_not_set*/));
    CHECK(not_set.toString() == "NOT_SET");

    auto toType_DualPolarization = six::DualPolarizationType::toType("OTHER");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER);
    TEST_ASSERT_EQ("OTHER", toType_DualPolarization.toString());

    toType_DualPolarization = six::DualPolarizationType::toType("V_OTHER");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::V_OTHER);
    TEST_ASSERT_EQ("V_OTHER", toType_DualPolarization.toString());

    toType_DualPolarization = six::DualPolarizationType::toType("OTHER_V");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER_V);
    TEST_ASSERT_EQ("OTHER_V", toType_DualPolarization.toString());

    toType_DualPolarization = six::DualPolarizationType::toType("OTHER_OTHER");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER_OTHER);
    TEST_ASSERT_EQ("OTHER_OTHER", toType_DualPolarization.toString());
}
TEST_CASE("DualPolarizationType_SixToType_OTHER")
{
    auto toType_DualPolarization = six::toType<six::DualPolarizationType>("OTHER");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER);
    TEST_ASSERT_EQ("OTHER", toType_DualPolarization.toString());

    CHECK_THROWS(six::toType<six::DualPolarizationType>("OTHER_abc")); // no "top level" OTHER.*

    toType_DualPolarization = six::toType<six::DualPolarizationType>("OTHER:V");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER_V);
    TEST_ASSERT_EQ("OTHER_V", toType_DualPolarization.toString());
    TEST_ASSERT_EQ("OTHER:V", six::toString(toType_DualPolarization));

    toType_DualPolarization = six::toType<six::DualPolarizationType>("V:OTHER_xyz"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::V_OTHER);
    TEST_ASSERT_EQ("V_OTHER", toType_DualPolarization.toString());
    TEST_ASSERT_EQ("V:OTHER_xyz", six::toString(toType_DualPolarization));

    toType_DualPolarization = six::toType<six::DualPolarizationType>("OTHER_xyz:V"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER_V);
    TEST_ASSERT_EQ("OTHER_V", toType_DualPolarization.toString());
    TEST_ASSERT_EQ("OTHER_xyz:V", six::toString(toType_DualPolarization));

    toType_DualPolarization = six::toType<six::DualPolarizationType>("OTHER:OTHER");
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER_OTHER);
    TEST_ASSERT_EQ("OTHER_OTHER", toType_DualPolarization.toString());
    TEST_ASSERT_EQ("OTHER:OTHER", six::toString(toType_DualPolarization));

    toType_DualPolarization = six::toType<six::DualPolarizationType>("OTHER_abc:OTHER_xyz"); // SICD 1.3 "OTHER.*"   
    TEST_ASSERT_EQ(toType_DualPolarization, six::DualPolarizationType::OTHER_OTHER);
    TEST_ASSERT_EQ("OTHER_OTHER", toType_DualPolarization.toString());
    TEST_ASSERT_EQ("OTHER_abc:OTHER_xyz", six::toString(toType_DualPolarization));

    toType_DualPolarization.other_ = "abc";
    CHECK_THROWS(six::toString(toType_DualPolarization));

    toType_DualPolarization.other_ = "OTHER_xyz:V";
    CHECK_THROWS(six::toString(toType_DualPolarization)); // OTHER_OTHER, not OTHER_V
}

template<typename TSixEnum>
static void test_six_toString_(const std::string& strType, TSixEnum type)
{
    const std::string polarizationString = six::toString<TSixEnum>(type);
    if (type != TSixEnum::OTHER)
    {
        TEST_ASSERT_EQ(strType, polarizationString);
    }
    TEST_ASSERT_EQ(polarizationString, six::toString(type)); // no template parameter

    auto pType = six::toType<TSixEnum>(polarizationString);
    TEST_ASSERT_EQ(pType, type);

    pType.other_ = "abc";
    CHECK_THROWS(six::toString(pType));
}
template<typename TSixEnum>
static void test_six_toString()
{
    test_six_toString_<TSixEnum>("UNKNOWN", TSixEnum::UNKNOWN);
    test_six_toString_<TSixEnum>("OTHER", TSixEnum::OTHER);
}
TEST_CASE("SixToString")
{
    test_six_toString_<six::DualPolarizationType>("V:V", six::DualPolarizationType::V_V);
    test_six_toString_<six::DualPolarizationType>("E:V", six::DualPolarizationType::E_V); // SICD 1.3
    test_six_toString<six::DualPolarizationType>();

    test_six_toString_<six::PolarizationType>("V", six::PolarizationType::V);
    test_six_toString_<six::PolarizationType>("X", six::PolarizationType::X); // SICD 1.3
    test_six_toString_<six::PolarizationType>("OTHER_abc", six::PolarizationType::OTHER); // SICD 1.3

    test_six_toString_<six::PolarizationSequenceType>("V", six::PolarizationSequenceType::V);
    test_six_toString_<six::PolarizationSequenceType>("X", six::PolarizationSequenceType::X); // SICD 1.3
    test_six_toString_<six::PolarizationSequenceType>("SEQUENCE", six::PolarizationSequenceType::SEQUENCE);
    test_six_toString_<six::PolarizationSequenceType>("OTHER_abc", six::PolarizationSequenceType::OTHER); // SICD 1.3
    test_six_toString<six::PolarizationSequenceType>();
}

template<typename TSixEnum>
static void test_NotSet()
{
    TSixEnum pType;
    TEST_ASSERT_EQ(pType, TSixEnum::NOT_SET);
    auto polarizationString = pType.toString();
    TEST_ASSERT_EQ(polarizationString, "NOT_SET");
    CHECK_THROWS(polarizationString = pType.toString(true /*throw_if_not_set*/));
    polarizationString = pType.toString(false /*throw_if_not_set*/);
    TEST_ASSERT_EQ(polarizationString, "NOT_SET");

    pType = TSixEnum::toType("NOT SET");
    TEST_ASSERT_EQ(pType, TSixEnum::NOT_SET);
    pType = TSixEnum::toType("NOT_SET");
    TEST_ASSERT_EQ(pType, TSixEnum::NOT_SET);

    CHECK_THROWS(six::toType<TSixEnum>("NOT_SET"));

    CHECK_THROWS(six::toString(pType));
}
TEST_CASE("NotSet")
{
    test_NotSet<six::DualPolarizationType>();
    test_NotSet<six::PolarizationType>();
    test_NotSet<six::PolarizationSequenceType>();

    SUCCEED();
}

template<typename TSixEnum>
static void test_EqInt_(const std::string& strType, TSixEnum type, int enumValue)
{
    const auto fromStrCtor = TSixEnum::toType(strType);
    TEST_ASSERT_EQ(enumValue, fromStrCtor);
    const int value = fromStrCtor;
    TEST_ASSERT_EQ(enumValue, value);

    const decltype(type)fromIntCtor(six::Enum::cast<TSixEnum>(value));
    TEST_ASSERT_EQ(enumValue, fromIntCtor);
    CHECK(fromIntCtor.toString() == strType);
    TEST_ASSERT_EQ(fromIntCtor, type);

    TEST_ASSERT_EQ(fromStrCtor, fromIntCtor);
};
template<typename TSixEnum>
static void test_EqInt(int unknownEnumValue)
{
    test_EqInt_<TSixEnum>("UNKNOWN", TSixEnum::UNKNOWN, unknownEnumValue);
    test_EqInt_<TSixEnum>("OTHER", TSixEnum::OTHER, 1);
    test_EqInt_<TSixEnum>("NOT_SET", TSixEnum::NOT_SET, six::NOT_SET_VALUE);

    // These will throw, at least for the polarization types
    static const std::vector<int> invalidIntValuesToTest{ -1, 0, six::NOT_SET_VALUE - 1 };
    for (auto&& v : invalidIntValuesToTest)
    {
        CHECK_THROWS(six::Enum::cast<TSixEnum>(v));
    }
}
TEST_CASE("EqInt")
{
    test_EqInt_<six::PolarizationType>("V", six::PolarizationType::V, 2);
    test_EqInt_<six::PolarizationType>("X", six::PolarizationType::X, 7); // SICD 1.3
    test_EqInt<six::PolarizationType>(6 /*unknownEnumValue*/);

    test_EqInt_<six::PolarizationSequenceType>("V", six::PolarizationSequenceType::V, 2);
    test_EqInt_<six::PolarizationSequenceType>("X", six::PolarizationSequenceType::X, 8); // SICD 1.3
    test_EqInt_<six::PolarizationSequenceType>("SEQUENCE", six::PolarizationSequenceType::SEQUENCE, 7);
    test_EqInt<six::PolarizationSequenceType>(6 /*unknownEnumValue*/);
}
TEST_CASE("DualPolarizationType_EqInt")
{
    test_EqInt_<six::DualPolarizationType>("V_V", six::DualPolarizationType::V_V, 2);
    test_EqInt_<six::DualPolarizationType>("E_V", six::DualPolarizationType::E_V, 56); // SICD 1.3
    test_EqInt_<six::DualPolarizationType>("OTHER_V", six::DualPolarizationType::OTHER_V, 75);
    test_EqInt_<six::DualPolarizationType>("OTHER_OTHER", six::DualPolarizationType::OTHER_OTHER, 83);
    test_EqInt<six::DualPolarizationType>(18 /*unknownEnumValue*/);
}

TEST_CASE("DualPolarization")
{
    // https://pcf-om-mil-bb5cb050-f7c0-44fc-b114-b886abb80450.s3.us-east-1.amazonaws.com/doc/Document/NGA.STND.0024-1_1.3.0.pdf?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAWDCVGY5THCA2IKOI%2F20220531%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20220531T154720Z&X-Amz-Expires=900&X-Amz-SignedHeaders=host&response-cache-control=900&response-content-disposition=NGA.STND.0024-1_1.3.0.pdf&X-Amz-Signature=dc2de9c048244ce338da927ccaa26567ca8c5afc9c36aa384c21dc2e8faeec04
    // Allowed values include the form TX:RCV that is formed from one  TX value and one RCV value.
    // Allowed TX values: �V�, �H�, �X�, �Y�, �S�, �E�, �RHC�, �LHC�, �OTHER*�
    // Allowed RCV values:  �V�, �H�, �X�, �Y�, �S�, �E�, �RHC�, �LHC�, �OTHER*�,    

    auto&& map = six::PolarizationType::string_to_value_();
    for (auto&& tx : map)
    {
        const auto txType = six::PolarizationType::toType(tx.first);
        TEST_ASSERT_EQ(tx.second, txType);
        if ((txType == six::PolarizationType::NOT_SET) || (txType == six::PolarizationType::UNKNOWN))
        {
            continue;
        }
        for (auto&& rcv : map)
        {
            const auto rcvType = six::PolarizationType::toType(rcv.first);
            TEST_ASSERT_EQ(rcv.second, rcvType);
            if ((rcvType == six::PolarizationType::NOT_SET) || (rcvType == six::PolarizationType::UNKNOWN))
            {
                continue;
            }
            auto strType = txType.toString() + "_" + rcvType.toString();

            auto fromToType = six::DualPolarizationType::toType(strType);
            auto str = fromToType.toString();
            TEST_ASSERT_EQ(str, strType);
            test_toType_(strType, fromToType);

            str::replace(strType, "_", ":");
            fromToType = six::toType<six::DualPolarizationType>(strType);
            str = six::toString(fromToType);
            TEST_ASSERT_EQ(str, strType);
            test_six_toType_(strType, fromToType);
        }
    }
}
