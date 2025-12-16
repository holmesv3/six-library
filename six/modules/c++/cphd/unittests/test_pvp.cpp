/* =========================================================================
 * This file is part of cphd-c++
 * =========================================================================
 *
 * (C) Copyright 2004 - 2019, MDA Information Systems LLC
 *
 * cphd-c++ is free software; you can redistribute it and/or modify
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
#include <memory>

#include <xml/lite/MinidomParser.h>
#include <six/Init.h>

#include <cphd/PVP.h>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("testSimpleEqualityOperatorTrue")
{
    cphd::Pvp pvp1, pvp2;

    pvp1.txTime.setOffset(0);
    pvp2.txTime.setOffset(0);

    CHECK((pvp1 == pvp2));
}

TEST_CASE("testAppend")
{
    cphd::Pvp pvp;
    pvp.append(pvp.txTime);
    pvp.append(pvp.txPos);
    pvp.append(pvp.txVel);
    pvp.append(pvp.ampSF);
    pvp.appendCustomParameter(8, "S8", "AddedParam1");
    pvp.append(pvp.signal);
    pvp.appendTxAnt();
    pvp.appendRcvAnt();

    CHECK(pvp.txTime.getOffset() == 0);
    CHECK(pvp.txPos.getOffset() == 1);
    CHECK(pvp.txVel.getOffset() == 4);
    CHECK(pvp.ampSF.getOffset() == 7);
    CHECK(pvp.addedPVP["AddedParam1"].getOffset() == 8);
    CHECK(pvp.signal.getOffset() == 16);
    CHECK(value(pvp.txAntenna).getOffset() == 17);
    CHECK(value(pvp.rcvAntenna).getOffset() == 25);
}

TEST_CASE("testAddedParamsEqualityOperatorTrue")
{
    cphd::Pvp pvp1;
    pvp1.setCustomParameter(1, 0, "F8", "AddedParam1");
    pvp1.setCustomParameter(1, 1, "F8", "AddedParam2");

    cphd::Pvp pvp2;
    pvp2.setCustomParameter(1, 0, "F8", "AddedParam1");
    pvp2.setCustomParameter(1, 1, "F8", "AddedParam2");

    CHECK((pvp1 == pvp2));
}


TEST_CASE("testSimpleEqualityOperatorFalse")
{
    cphd::Pvp pvp1;
    pvp1.fxN1.setOffset(0);

    cphd::Pvp pvp2;
    pvp2.txTime.setOffset(1);

    CHECK((pvp1 != pvp2));

}

TEST_CASE("testAddedParamsEqualityOperatorFalse")
{
    cphd::Pvp pvp1;
    pvp1.setCustomParameter(1, 0, "F8", "AddedParam1");
    pvp1.setCustomParameter(1, 1, "F8", "AddedParam2");

    cphd::Pvp pvp2;
    pvp2.setCustomParameter(1, 0, "F8", "AddedParam1");

    cphd::Pvp pvp3;
    pvp3.setCustomParameter(1, 0, "F8", "AddedParam1");
    pvp3.setCustomParameter(1, 1, "F8", "AddedParam3");

    CHECK((pvp1 != pvp2));
    CHECK((pvp1 != pvp3));
}

TEST_CASE("testAntennaFailure")
{
    cphd::Pvp pvp;
    pvp.appendTxAnt();
    pvp.appendRcvAnt();
    CHECK_THROWS(pvp.appendTxAnt());
    CHECK_THROWS(pvp.appendRcvAnt());
}
