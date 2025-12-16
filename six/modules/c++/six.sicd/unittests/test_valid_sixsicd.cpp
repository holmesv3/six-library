/* =========================================================================
* This file is part of six.sicd-c++
* =========================================================================
*
* (C) Copyright 2004 - 2016, MDA Information Systems LLC
* (C) Copyright 2021, Maxar Technologies, Inc.
*
* six.sicd-c++ is free software; you can redistribute it and/or modify
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

#include <stdlib.h>

#include <string>
#include <filesystem>
#include <span>

#include <io/FileInputStream.h>
#include <logging/NullLogger.h>
#include <import/sys.h>

#include <six/Utilities.h>
#include <import/six/sicd.h>

#include <catch2/catch_test_macros.hpp>

#define TEST_ASSERT_EQ(X, Y) CHECK(X == Y);
#define TEST_ASSERT_NULL(X) CHECK(X == nullptr);

static std::unique_ptr<six::sicd::ComplexData> test_assert_round_trip(const six::sicd::ComplexData& complexData, const std::vector<std::filesystem::path>* pSchemaPaths)
{
    auto strXML = six::sicd::Utilities::toXMLString(complexData, pSchemaPaths);
    CHECK_FALSE(strXML.empty());
    return six::sicd::Utilities::parseDataFromString(strXML, pSchemaPaths);
}

inline static const six::Unmodeled* get_Unmodeled(const six::sicd::ComplexData& complexData, const std::string& strVersion)
{
    if (strVersion != "1.3.0") // Unmodeled added in SICD 1.3
    {
        return nullptr;
    }

    if (has_value(complexData.errorStatistics->unmodeled))
    {
        return &value(complexData.errorStatistics->unmodeled);
    }

    return nullptr;
}

static void test_createFakeComplexData_(const std::string& strVersion)
{
    const auto pFakeComplexData = six::sicd::Utilities::createFakeComplexData(strVersion, six::PixelType::RE32F_IM32F, false /*makeAmplitudeTable*/);
    auto Unmodeled = get_Unmodeled(*pFakeComplexData, strVersion);
    TEST_ASSERT_NULL(Unmodeled); // not part of the fake data, only added in SICD 1.3

    // NULL schemaPaths, no validation
    auto pComplexData = test_assert_round_trip(*pFakeComplexData, nullptr /*pSchemaPaths*/);
    Unmodeled = get_Unmodeled(*pComplexData, strVersion);
    TEST_ASSERT_NULL(Unmodeled);  // not part of the fake data, only added in SICD 1.3

    // validate XML against schema
    const auto schemaPaths = six::testing::getSchemaPaths();
    pComplexData = test_assert_round_trip(*pFakeComplexData, &schemaPaths);
    Unmodeled = get_Unmodeled(*pComplexData, strVersion);
    TEST_ASSERT_NULL(Unmodeled);  // not part of the fake data, only added in SICD 1.3
}

TEST_CASE("test_createFakeComplexData")
{
    test_createFakeComplexData_("1.2.1");
    test_createFakeComplexData_("1.3.0");
}

static void test_assert_unmodeled_(const six::Unmodeled& unmodeled)
{
    TEST_ASSERT_EQ(1.23, unmodeled.Xrow);
    TEST_ASSERT_EQ(4.56, unmodeled.Ycol);
    TEST_ASSERT_EQ(7.89, unmodeled.XrowYcol);

    CHECK(has_value(unmodeled.unmodeledDecorr));
    auto&& unmodeledDecor = value(unmodeled.unmodeledDecorr);
    TEST_ASSERT_EQ(12.34, value(unmodeledDecor.Xrow).corrCoefZero);
    TEST_ASSERT_EQ(56.78, value(unmodeledDecor.Xrow).decorrRate);
    TEST_ASSERT_EQ(123.4, value(unmodeledDecor.Ycol).corrCoefZero);
    TEST_ASSERT_EQ(567.8, value(unmodeledDecor.Ycol).decorrRate);
}
static void test_assert(const six::sicd::ComplexData& complexData)
{
    auto&& errorStatistics = complexData.errorStatistics;
    if (complexData.getVersion() != "1.3.0")
    {
        TEST_ASSERT_NULL(errorStatistics.get());
        return;
    }
    CHECK(errorStatistics.get() != nullptr);
    auto&& unmodeled = errorStatistics->unmodeled;
    CHECK(has_value(unmodeled));
    test_assert_unmodeled_(value(unmodeled));

    // for SICD 1.3, also check the polarization type; this is set either in the fake data or scid130.xml
    const auto txRcvPolarizationProc = complexData.imageFormation->txRcvPolarizationProc;
    TEST_ASSERT_EQ(txRcvPolarizationProc, six::DualPolarizationType::OTHER_OTHER);
    const auto strTxRcvPolarizationProc = six::toString(txRcvPolarizationProc);
    TEST_ASSERT_EQ(strTxRcvPolarizationProc,"OTHER_TxRcvPolarizationProc:OTHER_TxRcvPolarizationProc");
}

static void test_read_sicd_xml(const std::filesystem::path& path)
{
    const auto pathname = six::testing::getSampleXmlPath(std::filesystem::path("six.sicd") / "tests" / "sample_xml", path);

    // NULL schemaPaths, no validation
    auto pComplexData = six::sicd::Utilities::parseDataFromFile(pathname, nullptr /*pSchemaPaths*/);
    test_assert(*pComplexData);

    pComplexData = test_assert_round_trip(*pComplexData, nullptr /*pSchemaPaths*/);
    test_assert(*pComplexData);

    // validate XML against schema
    const auto schemaPaths = six::testing::getSchemaPaths();
    pComplexData = six::sicd::Utilities::parseDataFromFile(pathname, &schemaPaths);
    test_assert(*pComplexData);

    pComplexData = test_assert_round_trip(*pComplexData, &schemaPaths);
    test_assert(*pComplexData);
}

TEST_CASE("test_read_sicd110_xml")
{
    test_read_sicd_xml("sicd110.xml");
}

TEST_CASE("test_read_sicd130_xml")
{
    test_read_sicd_xml("sicd130.xml");
}

// Set SIX_PROFILE_PARSING=N when running the test to profile the tests by
// re-running N-times
#define PROFILE(X) six::testing::EnvProfiler("SIX_PROFILE_PARSING", std::cerr)( [&]() { X; });

// Set SIX_PROFILE_STACKSIZE=1 when running to log the size of the stacktrace
#define SSPROFILE(X, Y)                                                 \
    CHECK_THROWS_AS(six::testing::StackTraceSizeEnvProfiler<Y>( \
        "SIX_PROFILE_STACKSIZE", std::cerr)([&]() { X; }), Y)

#define TEST_BAD_XML(X) PROFILE(SSPROFILE(X, six::DESValidationException));

TEST_CASE("test_read_sicd040_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd040-bad.xml"));
}

TEST_CASE("test_read_sicd041_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd041-bad.xml"));
}

TEST_CASE("test_read_sicd050_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd050-bad.xml"));
}

TEST_CASE("test_read_sicd100_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd100-bad.xml"));
}

TEST_CASE("test_read_sicd101_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd101-bad.xml"));
}

TEST_CASE("test_read_sicd110_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd110-bad.xml"));
}

TEST_CASE("test_read_sicd120_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd120-bad.xml"));
}

TEST_CASE("test_read_sicd121_bad_xml")
{
    TEST_BAD_XML(test_read_sicd_xml("sicd121-bad.xml"));
}
