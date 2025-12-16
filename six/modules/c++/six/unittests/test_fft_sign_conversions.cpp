#include <six/Utilities.h>
#include <six/Enums.h>

#include <catch2/catch_test_macros.hpp>

#define TEST_ASSERT_EQ(X, Y) CHECK(X == Y);

TEST_CASE("testToType")
{
    TEST_ASSERT_EQ(six::toType<six::FFTSign>("+1"),
            six::FFTSign(six::FFTSign::POS));
    TEST_ASSERT_EQ(six::toType<six::FFTSign>("1"),
            six::FFTSign(six::FFTSign::POS));
    TEST_ASSERT_EQ(six::toType<six::FFTSign>("-1"),
            six::FFTSign(six::FFTSign::NEG));

}
