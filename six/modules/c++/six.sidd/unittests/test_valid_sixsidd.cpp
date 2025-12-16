/* =========================================================================
* This file is part of six.sidd-c++
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
#include <string>
#include <filesystem>
#include <span>

#include <io/FileInputStream.h>
#include <logging/NullLogger.h>
#include <import/sys.h>

#include <six/Utilities.h>
#include <import/six/sidd.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#define TEST_ASSERT_EQ(X, Y) CHECK(X == Y);
#define TEST_ASSERT_ALMOST_EQ(X, Y) CHECK_THAT(X, Catch::Matchers::WithinAbs(Y, 0.0001));
#define TEST_ASSERT_ALMOST_EQ_EPS(X, Y, Z) CHECK_THAT(X, Catch::Matchers::WithinAbs(Y, Z));

#if _MSC_VER
#pragma warning(disable: 4459) //  declaration of '...' hides global declaration
#endif

static inline std::filesystem::path six_sidd_relative_path()
{
    return std::filesystem::path("six") / "modules" / "c++" / "six.sidd";
}
static std::filesystem::path schema_relative_path()
{
    return six_sidd_relative_path() / "conf" / "schema"; // .../conf/schema
}

static std::filesystem::path get_sample_nitf_path(const std::filesystem::path& filename)
{
    static const auto modulePath = six_sidd_relative_path() / "tests" / "sample_nitf";
    return sys::test::findGITModuleFile("six", modulePath, filename);
}
static std::filesystem::path get_sample_xml_path(const std::filesystem::path& filename)
{
    static const auto modulePath = six_sidd_relative_path() / "tests" / "sample_xml";
    return sys::test::findGITModuleFile("six", modulePath, filename);
}

static std::vector<std::filesystem::path> getSchemaPaths()
{
    static const auto xsdPath = sys::test::findGITModuleFile("six", schema_relative_path(), "SICommonTypes_V1.0.xsd");
    static const auto rootSchemaDir = xsdPath.parent_path(); // ".../conf/Schema"
    return std::vector<std::filesystem::path> { rootSchemaDir };
}

static std::unique_ptr<six::sidd::DerivedData> test_assert_round_trip(const six::sidd::DerivedData& derivedData, const std::vector<std::filesystem::path>* pSchemaPaths)
{
    const auto strXML = six::sidd::Utilities::toXMLString(derivedData, pSchemaPaths);
    CHECK_FALSE(strXML.empty());

    const auto xml_ = str::to_native(strXML); // for debugging
    CHECK_FALSE(xml_.empty());

    return six::sidd::Utilities::parseDataFromString(strXML, pSchemaPaths);
}

inline static const six::Unmodeled* get_Unmodeled(const six::sidd::DerivedData& derivedData, six::sidd::Version siddVersion)
{
    if (siddVersion != six::sidd::Version::v3_0_0) // Unmodeled added in SIDD 3.0
    {
        return nullptr;
    }

    if (has_value(derivedData.errorStatistics->unmodeled))
    {
        return &value(derivedData.errorStatistics->unmodeled);
    }

    return nullptr;
}

static void test_createFakeDerivedData_(bool validate,
    six::sidd::Version siddVersion, six::sidd300::ISMVersion ismVersion)
{
    const auto pFakeDerivedData = six::sidd::Utilities::createFakeDerivedData(siddVersion, ismVersion);
    auto Unmodeled = get_Unmodeled(*pFakeDerivedData, siddVersion);
    CHECK(Unmodeled == nullptr); // not part of the fake data, only added in SIDD 3.0

    const auto schemaPaths = getSchemaPaths();
    const std::vector<std::filesystem::path>* pSchemaPaths = validate ? & schemaPaths: nullptr; // NULL schemaPaths, no validation

    auto pDerivedData = test_assert_round_trip(*pFakeDerivedData, pSchemaPaths);
    Unmodeled = get_Unmodeled(*pDerivedData, siddVersion);
    CHECK(Unmodeled == nullptr);  // not part of the fake data, only added in SIDD 3.0
}
TEST_CASE("test_createFakeDerivedData")
{
    test_createFakeDerivedData_(false /*validate*/, six::sidd::Version::v2_0_0, six::sidd300::get(six::sidd300::ISMVersion::current));
    test_createFakeDerivedData_(false /*validate*/, six::sidd::Version::v3_0_0, six::sidd300::ISMVersion::v13);
    test_createFakeDerivedData_(false /*validate*/, six::sidd::Version::v3_0_0, six::sidd300::ISMVersion::v201609);
}
TEST_CASE("test_createFakeDerivedData_validate")
{
    test_createFakeDerivedData_(true /*validate*/, six::sidd::Version::v2_0_0, six::sidd300::get(six::sidd300::ISMVersion::current));
    test_createFakeDerivedData_(true /*validate*/, six::sidd::Version::v3_0_0, six::sidd300::ISMVersion::v13);
    test_createFakeDerivedData_(true /*validate*/, six::sidd::Version::v3_0_0, six::sidd300::ISMVersion::v201609);
}

TEST_CASE("test_read_sidd200_no_LUT")
{
    static const auto pathname = get_sample_nitf_path("2023-07-26-11-37-27_UMBRA-04_SIDD.nitf");

    six::XMLControlRegistry& xml_registry = six::getXMLControlFactory();
    xml_registry.addCreator(six::DataType::DERIVED,
        new six::XMLControlCreatorT<six::sidd::DerivedXMLControl>());

    six::NITFReadControl reader;
    reader.setXMLControlRegistry(xml_registry);
    reader.load(pathname.string());
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
static void test_assert_unmodeled(const six::sidd::DerivedData& derivedData)
{
    auto&& errorStatistics = derivedData.errorStatistics;
    CHECK(errorStatistics.get() != nullptr);
    if (derivedData.getVersion() != "3.0.0")
    {
        return;
    }

    auto unmodeled = errorStatistics->unmodeled;
    CHECK(has_value(unmodeled));
    test_assert_unmodeled_(value(unmodeled));
}

static void test_read_sidd_xml(const std::filesystem::path& path,
    const std::vector<std::filesystem::path>* pSchemaPaths)
{
    const auto pathname = get_sample_xml_path(path);

    auto pDerivedData = six::sidd::Utilities::parseDataFromFile(pathname, pSchemaPaths);
    test_assert_unmodeled(*pDerivedData);

    pDerivedData = test_assert_round_trip(*pDerivedData, pSchemaPaths);
    test_assert_unmodeled(*pDerivedData);
}
static void test_read_sidd_xml(const std::filesystem::path& path)
{
    const std::vector<std::filesystem::path>* pSchemaPaths = nullptr; // NULL schemaPaths, no validation
    test_read_sidd_xml(path, pSchemaPaths);

    // validate XML against schema
    const auto schemaPaths = getSchemaPaths();
    test_read_sidd_xml(path, &schemaPaths);
}
TEST_CASE("test_read_sidd200_xml")
{
    test_read_sidd_xml("sidd200.xml");
}
TEST_CASE("test_read_sidd300_xml")
{
    test_read_sidd_xml("sidd300.xml");
}

TEST_CASE("test_read_sidd300_v13_xml")
{
    test_read_sidd_xml("sidd300_ISM-v13.xml");
}

// Set SIX_PROFILE_PARSING=N when running the test to profile the tests by
// re-running N-times
#define PROFILE(X)                                                         \
    six::testing::EnvProfiler("SIX_PROFILE_PARSING", std::cerr)( \
            [&]() { X; });

// Set SIX_PROFILE_STACKSIZE=1 when running to log the size of the stacktrace
#define SSPROFILE(X, Y)                                                 \
    CHECK_THROWS_AS(six::testing::StackTraceSizeEnvProfiler<Y>( \
                                    "SIX_PROFILE_STACKSIZE",            \
                                    std::cerr)([&]() { X; }),           \
                            Y)

#define TEST_BAD_XML(X) PROFILE(SSPROFILE(X, six::DESValidationException));

TEST_CASE("test_read_sidd100_bad_xml")
{
    const auto schemaPaths = getSchemaPaths();
    TEST_BAD_XML(test_read_sidd_xml("sidd100-bad.xml", &schemaPaths));
}

TEST_CASE("test_read_sidd200_bad_xml")
{
    const auto schemaPaths = getSchemaPaths();
    TEST_BAD_XML(test_read_sidd_xml("sidd200-bad.xml", &schemaPaths));
}

TEST_CASE("test_read_sidd300_bad_xml")
{
    const auto schemaPaths = getSchemaPaths();
    TEST_BAD_XML(test_read_sidd_xml("sidd300-bad.xml", &schemaPaths));
}

TEST_CASE("test_read_sidd300_v13_bad_xml")
{
    const auto schemaPaths = getSchemaPaths();
    TEST_BAD_XML(test_read_sidd_xml("sidd300_ISM-v13-bad.xml",
                                    &schemaPaths));
}
