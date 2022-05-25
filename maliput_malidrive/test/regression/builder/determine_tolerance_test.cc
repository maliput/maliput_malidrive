// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_malidrive/builder/determine_tolerance.h"

#include <optional>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_malidrive/constants.h"
#include "maliput_malidrive/test_utilities/xodr_testing_map_descriptions.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

class DetermineToleranceTest : public ::testing::Test {
 public:
  const std::optional<double> kLoaderNoToleranceCheck{};
};

// @{ Angular tolerance determination tests.
class DetermineLinearToleranceTest : public DetermineToleranceTest {};

// Input validation.
TEST_F(DetermineLinearToleranceTest, NullptrArgument) {
  EXPECT_THROW(DetermineRoadGeometryLinearTolerance(nullptr), maliput::common::assertion_error);
}

// Expects constants::kLinearTolerance because there is one road with a line
// defined and its length is greater than constants::kLinearTolerance. No gaps
// are present in the definition.
TEST_F(DetermineLinearToleranceTest, SingleGeometrySoftConstraintIsUsed) {
  auto db_manager = xodr::LoadDataBaseFromStr(malidrive::test::kXodrSingleGeometry, {kLoaderNoToleranceCheck});
  EXPECT_EQ(constants::kLinearTolerance, DetermineRoadGeometryLinearTolerance(db_manager.get()));
}

// Expects kMinLinearTolerance because there is one road with a line
// defined and its length is smaller than kMinLinearTolerance. No gaps
// are present in the definition.
TEST_F(DetermineLinearToleranceTest, TinySingleGeometrySoftConstraintIsUsed) {
  constexpr const char* kXodrMap = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrMap' version='1.0' date='Tue Oct 20 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="A" length="1e-7" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="1e-7">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

  auto db_manager = xodr::LoadDataBaseFromStr(kXodrMap, {kLoaderNoToleranceCheck});
  EXPECT_EQ(kMinLinearTolerance, DetermineRoadGeometryLinearTolerance(db_manager.get()));
}

// Evaluates that internal gaps in the geometry rule over the non-existent elevation gaps.
TEST_F(DetermineLinearToleranceTest, HardConstraintIsUsedBecauseOfGeometryGaps) {
  constexpr const char* kXodrMap = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrMap' version='1.0' date='Tue Oct 20 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="A" length="11" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
              <line/>
          </geometry>
          <geometry s="10.0" x="11.0" y="0.0" hdg="0.0" length="1">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

  auto db_manager = xodr::LoadDataBaseFromStr(kXodrMap, {kLoaderNoToleranceCheck});
  EXPECT_EQ(1.5, DetermineRoadGeometryLinearTolerance(db_manager.get()));
}

// Evaluates that internal gaps in the elevation rule over the non-existent geometry gaps.
TEST_F(DetermineLinearToleranceTest, HardConstraintIsUsedBecauseOfElevationGaps) {
  constexpr const char* kXodrMap = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrMap' version='1.0' date='Tue Oct 20 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="A" length="11" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="11">
              <line/>
          </geometry>
      </planView>
      <elevationProfile>
        <elevation s="0" a="2" b="0" c="0" d="0"/>
        <elevation s="5.5" a="-2" b="0" c="0" d="0"/>
      </elevationProfile>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

  auto db_manager = xodr::LoadDataBaseFromStr(kXodrMap, {kLoaderNoToleranceCheck});
  EXPECT_EQ(6., DetermineRoadGeometryLinearTolerance(db_manager.get()));
}

// Evaluates that the largest gap rules over the other.
TEST_F(DetermineLinearToleranceTest, HardConstraintIsUsed) {
  constexpr const char* kXodrMap = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrMap' version='1.0' date='Tue Oct 20 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="A" length="11" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
              <line/>
          </geometry>
          <geometry s="10.0" x="11.0" y="0.0" hdg="0.0" length="1">
              <line/>
          </geometry>
      </planView>
      <elevationProfile>
        <elevation s="0" a="2" b="0" c="0" d="0"/>
        <elevation s="5.5" a="-2" b="0" c="0" d="0"/>
      </elevationProfile>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

  auto db_manager = xodr::LoadDataBaseFromStr(kXodrMap, {kLoaderNoToleranceCheck});
  EXPECT_EQ(6., DetermineRoadGeometryLinearTolerance(db_manager.get()));
}

// Evaluates that internal gaps in the geometry rule over the non-existent elevation gaps.
TEST_F(DetermineLinearToleranceTest, HardConstraintIsUsedWithTinyGap) {
  constexpr const char* kXodrMap = R"R(
<?xml version='1.0' standalone='yes'?>
<OpenDRIVE>
  <header revMajor='1.' revMinor='1.' name='XodrMap' version='1.0' date='Tue Oct 20 12:00:00 2020'
    north='0.' south='0.' east='0.' west='0.' vendor='Toyota Research Institute' >
  </header>
  <road name="A" length="11" id="1" junction="-1">
      <link/>
      <planView>
          <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10">
              <line/>
          </geometry>
          <geometry s="10.0" x="10.00001" y="0.0" hdg="0.0" length="1">
              <line/>
          </geometry>
      </planView>
      <lanes>
          <laneSection s="0.0e+0">
              <center>
                  <lane id="0" type="none" level="false">
                  </lane>
              </center>
              <right>
                  <lane id="-1" type="driving" level="false">
                      <width sOffset="0.0e+0" a="3.5e+0" b="0.0e+0" c="0.0e+0" d="0.0e+0"/>
                  </lane>
              </right>
          </laneSection>
      </lanes>
  </road>
</OpenDRIVE>
)R";

  auto db_manager = xodr::LoadDataBaseFromStr(kXodrMap, {kLoaderNoToleranceCheck});
  EXPECT_EQ(kMinLinearTolerance, DetermineRoadGeometryLinearTolerance(db_manager.get()));
}
// @}

// @{ Angular tolerance determination tests.
class DetermineAngularToleranceTest : public DetermineToleranceTest {};

// Input validation.
TEST_F(DetermineAngularToleranceTest, NullptrArgument) {
  EXPECT_THROW(DetermineRoadGeometryAngularTolerance(nullptr), maliput::common::assertion_error);
}

// Return value.
TEST_F(DetermineAngularToleranceTest, ReturnValue) {
  EXPECT_EQ(constants::kAngularTolerance,
            DetermineRoadGeometryAngularTolerance(reinterpret_cast<const xodr::DBManager*>(0xDeadC0de)));
}
// @}

// @{ Scale length determination tests.
class DetermineScaleLengthTest : public DetermineToleranceTest {
 protected:
  const double kPositiveTolerance{1.};
  const double kNegativeTolerance{-1.};
  const double kZeroTolerance{0.};
  const xodr::DBManager* kDBManagerPtr{reinterpret_cast<const xodr::DBManager*>(0xDeadC0de)};
};

// Input validation.
TEST_F(DetermineScaleLengthTest, NullptrArgument) {
  EXPECT_THROW(DetermineRoadGeometryScaleLength(nullptr, kPositiveTolerance, kPositiveTolerance),
               maliput::common::assertion_error);
}

// Input validation.
TEST_F(DetermineScaleLengthTest, NonPositiveLinearTolerance) {
  EXPECT_THROW(DetermineRoadGeometryScaleLength(kDBManagerPtr, kZeroTolerance, kPositiveTolerance),
               maliput::common::assertion_error);
  EXPECT_THROW(DetermineRoadGeometryScaleLength(kDBManagerPtr, kNegativeTolerance, kPositiveTolerance),
               maliput::common::assertion_error);
}

// Input validation.
TEST_F(DetermineScaleLengthTest, NonPositiveAngularTolerance) {
  EXPECT_THROW(DetermineRoadGeometryScaleLength(kDBManagerPtr, kPositiveTolerance, kZeroTolerance),
               maliput::common::assertion_error);
  EXPECT_THROW(DetermineRoadGeometryScaleLength(kDBManagerPtr, kPositiveTolerance, kNegativeTolerance),
               maliput::common::assertion_error);
}

// Return value.
TEST_F(DetermineScaleLengthTest, ReturnValue) {
  EXPECT_EQ(constants::kScaleLength,
            DetermineRoadGeometryScaleLength(kDBManagerPtr, kPositiveTolerance, kPositiveTolerance));
}
// @}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
